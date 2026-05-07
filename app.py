import streamlit as st
import folium
from streamlit_folium import st_folium, folium_static
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
import pandas as pd
from dataclasses import dataclass, field

# ==================== 配置常量 ====================
@dataclass
class Config:
    """系统配置类"""
    SCHOOL_CENTER_GCJ: List[float] = field(default_factory=lambda: [118.7490, 32.2340])
    DEFAULT_A_GCJ: List[float] = field(default_factory=lambda: [118.746956, 32.232945])
    DEFAULT_B_GCJ: List[float] = field(default_factory=lambda: [118.751589, 32.235204])
    
    CONFIG_FILE: str = "obstacle_config.json"
    BACKUP_DIR: str = "backups"
    DEFAULT_SAFETY_RADIUS_METERS: int = 15
    MAX_BACKUP_FILES: int = 10
    
    BASE_SPEED_MPS: float = 5.0
    HEARTBEAT_INTERVAL: float = 0.2
    VOLTAGE_VARIATION: float = 0.5
    SAT_RANGE: Tuple[int, int] = (8, 14)
    
    GAODE_SATELLITE_URL: str = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
    GAODE_VECTOR_URL: str = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
    
    # 绕行参数
    SAFETY_MARGIN_MULTIPLIER: float = 1.5  # 安全边距倍数

config = Config()
os.makedirs(config.BACKUP_DIR, exist_ok=True)

# ==================== 几何函数（修正版） ====================
def calculate_distance_meters(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """
    计算两点间距离（米）
    输入格式: (纬度, 经度)
    """
    lat1, lng1 = point1
    lat2, lng2 = point2
    R = 6371000  # 地球半径（米）
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def distance_deg(point1: List[float], point2: List[float]) -> float:
    """计算两点经纬度差值距离（用于比例计算）"""
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def point_in_polygon(point: Tuple[float, float], polygon: List[List[float]]) -> bool:
    """
    判断点是否在多边形内
    point: (lat, lng)
    polygon: [[lat, lng], ...]
    """
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def segments_intersect(p1: Tuple[float, float], p2: Tuple[float, float], 
                       p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
    """判断两条线段是否相交"""
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    
    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False

def line_intersects_polygon(line_start: Tuple[float, float], line_end: Tuple[float, float], 
                            polygon: List[List[float]]) -> bool:
    """判断线段是否与多边形相交"""
    if point_in_polygon(line_start, polygon) or point_in_polygon(line_end, polygon):
        return True
    
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(line_start, line_end, p1, p2):
            return True
    return False

def get_polygon_bounds(polygon: List[List[float]]) -> Tuple[float, float, float, float]:
    """获取多边形边界 (min_lat, max_lat, min_lng, max_lng)"""
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return min(lats), max(lats), min(lngs), max(lngs)

def get_polygon_center(polygon: List[List[float]]) -> Tuple[float, float]:
    """获取多边形中心点"""
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return sum(lats)/len(lats), sum(lngs)/len(lngs)

def meters_to_deg_lat(meters: float) -> float:
    """米转纬度度数"""
    return meters / 111320

def meters_to_deg_lng(meters: float, lat: float) -> float:
    """米转经度度数"""
    return meters / (111320 * math.cos(math.radians(lat)))

def get_intersection_point(p1: Tuple[float, float], p2: Tuple[float, float], 
                           p3: Tuple[float, float], p4: Tuple[float, float]) -> Optional[Tuple[float, float]]:
    """计算两条线的交点"""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        return (x, y)
    return None

def line_circle_intersection(line_start: Tuple[float, float], line_end: Tuple[float, float],
                              center: Tuple[float, float], radius_meters: float) -> List[Tuple[float, float]]:
    """计算线段与圆的交点（用于安全边界）"""
    # 将圆心转换为米制坐标（近似）
    ref_lat = center[0]
    lng_scale = 111320 * math.cos(math.radians(ref_lat))
    lat_scale = 111320
    
    def to_meters(p):
        return (p[1] * lng_scale, p[0] * lat_scale)
    
    def to_deg(p):
        return (p[1] / lat_scale, p[0] / lng_scale)
    
    x1, y1 = to_meters(line_start)
    x2, y2 = to_meters(line_end)
    cx, cy = to_meters(center)
    
    dx = x2 - x1
    dy = y2 - y1
    
    a = dx*dx + dy*dy
    if a < 1e-10:
        return []
    
    b = 2 * (dx*(x1 - cx) + dy*(y1 - cy))
    c = (x1 - cx)**2 + (y1 - cy)**2 - radius_meters**2
    
    discriminant = b*b - 4*a*c
    
    if discriminant < 0:
        return []
    
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2*a)
    t2 = (-b + sqrt_disc) / (2*a)
    
    intersections = []
    for t in [t1, t2]:
        if 0 <= t <= 1:
            x = x1 + t * dx
            y = y1 + t * dy
            intersections.append(to_deg((x, y)))
    
    return intersections

# ==================== 路径安全检查 ====================
def is_path_safe(start: Tuple[float, float], end: Tuple[float, float], 
                 obstacles: List[Dict], flight_altitude: float) -> bool:
    """判断路径是否安全"""
    for obs in obstacles:
        if obs.get("height", 0) >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                return False
    return True

def get_blocking_obstacles(start: Tuple[float, float], end: Tuple[float, float],
                           obstacles: List[Dict], flight_altitude: float) -> List[Dict]:
    """获取阻挡路径的障碍物"""
    blocking = []
    for obs in obstacles:
        if obs.get("height", 0) >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                blocking.append(obs)
    return blocking

import math
from typing import List, Dict, Tuple, Optional

# ==================== 辅助函数 ====================

def meters_to_deg_lng(meters: float, lat: float) -> float:
    """将米转换为经度度数（依赖纬度）"""
    return meters / (111000 * math.cos(math.radians(lat)))

def meters_to_deg_lat(meters: float) -> float:
    """将米转换为纬度度数"""
    return meters / 111000

def distance_meters(point1: List[float], point2: List[float]) -> float:
    """计算两点间距离（米）- 使用Haversine公式
    point格式: [lng, lat]
    """
    lng1, lat1 = point1[0], point1[1]
    lng2, lat2 = point2[0], point2[1]
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlng/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return 6371000 * c  # 地球半径（米）

def line_intersects_polygon_accurate(start: List[float], end: List[float], polygon: List[List[float]]) -> bool:
    """
    检查线段是否与多边形相交（精确版本）
    point格式: [lng, lat]
    polygon格式: [[lng, lat], ...]
    """
    def on_segment(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))
    
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-10:
            return 0
        return 1 if val > 0 else 2
    
    def segments_intersect(p1, p2, p3, p4):
        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)
        
        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and on_segment(p1, p3, p2):
            return True
        if o2 == 0 and on_segment(p1, p4, p2):
            return True
        if o3 == 0 and on_segment(p3, p1, p4):
            return True
        if o4 == 0 and on_segment(p3, p2, p4):
            return True
        return False
    
    # 检查端点是否在多边形内（简化：使用射线法）
    def point_in_polygon(point, poly):
        x, y = point
        inside = False
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
        return inside
    
    if point_in_polygon(start, polygon) or point_in_polygon(end, polygon):
        return True
    
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(start, end, p1, p2):
            return True
    
    return False


# ==================== 障碍物处理 ====================

class ObstacleBounds:
    """障碍物边界信息"""
    def __init__(self):
        self.min_lng = float('inf')
        self.max_lng = -float('inf')
        self.min_lat = float('inf')
        self.max_lat = -float('inf')
        self.max_height = 0.0
        self.has_obstacle = False
    
    def update(self, obstacle: Dict):
        """更新边界"""
        polygon = obstacle.get('polygon', [])
        if not polygon:
            return
        
        self.has_obstacle = True
        height = obstacle.get('height', 0)
        self.max_height = max(self.max_height, height)
        
        for point in polygon:
            self.min_lng = min(self.min_lng, point[0])
            self.max_lng = max(self.max_lng, point[0])
            self.min_lat = min(self.min_lat, point[1])
            self.max_lat = max(self.max_lat, point[1])
    
    @property
    def width_meters(self) -> float:
        """障碍物宽度（米）"""
        if not self.has_obstacle:
            return 0
        lat_rad = math.radians((self.min_lat + self.max_lat) / 2)
        lng_scale = 111000 * math.cos(lat_rad)
        return (self.max_lng - self.min_lng) * lng_scale
    
    @property
    def height_meters(self) -> float:
        """障碍物高度（米）"""
        return self.max_height
    
    @property
    def center(self) -> Tuple[float, float]:
        """障碍物中心点 (lng, lat)"""
        return ((self.min_lng + self.max_lng) / 2, (self.min_lat + self.max_lat) / 2)


def get_blocking_obstacles(start: List[float], end: List[float], 
                           obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    """
    获取阻挡航线的障碍物
    - 高度大于飞行高度
    - 与航线有交点
    """
    blocking = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon_accurate(start, end, coords):
                blocking.append(obs)
    return blocking


def get_obstacles_bounds(obstacles: List[Dict]) -> ObstacleBounds:
    """计算障碍物集合的整体边界"""
    bounds = ObstacleBounds()
    for obs in obstacles:
        bounds.update(obs)
    return bounds


# ==================== 绕行策略 ====================

def find_above_path(start: List[float], end: List[float], 
                    obstacles_gcj: List[Dict], flight_altitude: float,
                    safety_radius: float = 5, climb_ratio: float = 1.5) -> List[List[float]]:
    """
    向上绕行策略：飞越障碍物顶部
    
    路径结构：
    起点 → 爬升点（水平前进同时爬升）→ 障碍物上方平飞 → 下降点 → 终点
    
    Args:
        climb_ratio: 爬升高度倍数（相对于障碍物高度）
        注意：实际高度在飞行参数中设置，这里只生成平面路径点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking_obs:
        return [start, end]
    
    bounds = get_obstacles_bounds(blocking_obs)
    
    # 计算进入和退出点（障碍物前后偏移安全距离）
    safe_meters = safety_radius * 3
    lat_rad = math.radians((start[1] + end[1]) / 2)
    lng_scale = 111000 * math.cos(lat_rad)
    lat_scale = 111000
    
    # 获取交点比例
    entry_ratio = get_intersection_entry_ratio(start, end, bounds)
    exit_ratio = get_intersection_exit_ratio(start, end, bounds)
    
    # 爬升点（在到达障碍物前开始爬升）
    climb_point = [
        start[0] + (end[0] - start[0]) * max(0.05, entry_ratio - safe_meters / lng_scale),
        start[1] + (end[1] - start[1]) * max(0.05, entry_ratio - safe_meters / lat_scale)
    ]
    
    # 下降点（飞越障碍物后开始下降）
    descend_point = [
        start[0] + (end[0] - start[0]) * min(0.95, exit_ratio + safe_meters / lng_scale),
        start[1] + (end[1] - start[1]) * min(0.95, exit_ratio + safe_meters / lat_scale)
    ]
    
    return [start, climb_point, descend_point, end]


def find_horizontal_path(start: List[float], end: List[float],
                         obstacles_gcj: List[Dict], flight_altitude: float,
                         direction: str, safety_radius: float = 5) -> List[List[float]]:
    """
    水平绕行策略：从左侧或右侧绕过障碍物
    
    Args:
        direction: "left" 或 "right"
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    if not blocking_obs:
        return [start, end]
    
    bounds = get_obstacles_bounds(blocking_obs)
    
    # 航线方向向量
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.sqrt(dx * dx + dy * dy)
    
    if length < 1e-10:
        return [start, end]
    
    # 归一化方向向量
    nx = dx / length
    ny = dy / length
    
    # 垂直向量（垂直于航线方向）
    # 左侧: (-ny, nx), 右侧: (ny, -nx)
    if direction == "left":
        perp_x = -ny
        perp_y = nx
    else:  # right
        perp_x = ny
        perp_y = -nx
    
    # 计算需要偏移的距离（障碍物半宽 + 安全半径 × 3）
    offset_meters = bounds.width_meters / 2 + safety_radius * 3
    
    # 转换为经纬度偏移
    mid_lat = (start[1] + end[1]) / 2
    lng_scale = 111000 * math.cos(math.radians(mid_lat))
    lat_scale = 111000
    
    offset_lng = perp_x * offset_meters / lng_scale
    offset_lat = perp_y * offset_meters / lat_scale
    
    # 障碍物前后绕行点比例
    entry_ratio = get_intersection_entry_ratio(start, end, bounds)
    exit_ratio = get_intersection_exit_ratio(start, end, bounds)
    
    safe_meters = safety_radius * 2
    flight_length_meters = distance_meters(start, end)
    entry_ratio = max(0.05, entry_ratio - safe_meters / flight_length_meters)
    exit_ratio = min(0.95, exit_ratio + safe_meters / flight_length_meters)
    
    # 进入绕行点
    entry_point = [
        start[0] + (end[0] - start[0]) * entry_ratio,
        start[1] + (end[1] - start[1]) * entry_ratio
    ]
    
    # 绕行中间点（偏移后）
    waypoint = [entry_point[0] + offset_lng, entry_point[1] + offset_lat]
    
    # 退出绕行点
    exit_point = [
        start[0] + (end[0] - start[0]) * exit_ratio,
        start[1] + (end[1] - start[1]) * exit_ratio
    ]
    exit_point_offset = [exit_point[0] + offset_lng, exit_point[1] + offset_lat]
    
    return [start, entry_point, waypoint, exit_point_offset, end]


def get_intersection_entry_ratio(start: List[float], end: List[float], bounds: ObstacleBounds) -> float:
    """
    计算航线与障碍物相交的进入点比例（0~1）
    返回航线从起点到开始与障碍物相交的比例
    """
    if not bounds.has_obstacle:
        return 0.5
    
    total_dx = end[0] - start[0]
    total_dy = end[1] - start[1]
    total_length_sq = total_dx * total_dx + total_dy * total_dy
    
    if total_length_sq < 1e-10:
        return 0.5
    
    # 计算障碍物四个角点在航线上的投影比例，取最小值
    corners = [
        (bounds.min_lng, bounds.min_lat),
        (bounds.min_lng, bounds.max_lat),
        (bounds.max_lng, bounds.min_lat),
        (bounds.max_lng, bounds.max_lat)
    ]
    
    min_t = 1.0
    for corner_lng, corner_lat in corners:
        dx_corner = corner_lng - start[0]
        dy_corner = corner_lat - start[1]
        t = (dx_corner * total_dx + dy_corner * total_dy) / total_length_sq
        t = max(0, min(1, t))
        min_t = min(min_t, t)
    
    return max(0.05, min_t - 0.1)


def get_intersection_exit_ratio(start: List[float], end: List[float], bounds: ObstacleBounds) -> float:
    """
    计算航线与障碍物相交的退出点比例（0~1）
    返回航线从起点到结束与障碍物相交的比例
    """
    if not bounds.has_obstacle:
        return 0.5
    
    total_dx = end[0] - start[0]
    total_dy = end[1] - start[1]
    total_length_sq = total_dx * total_dx + total_dy * total_dy
    
    if total_length_sq < 1e-10:
        return 0.5
    
    # 计算障碍物四个角点在航线上的投影比例，取最大值
    corners = [
        (bounds.min_lng, bounds.min_lat),
        (bounds.min_lng, bounds.max_lat),
        (bounds.max_lng, bounds.min_lat),
        (bounds.max_lng, bounds.max_lat)
    ]
    
    max_t = 0.0
    for corner_lng, corner_lat in corners:
        dx_corner = corner_lng - start[0]
        dy_corner = corner_lat - start[1]
        t = (dx_corner * total_dx + dy_corner * total_dy) / total_length_sq
        t = max(0, min(1, t))
        max_t = max(max_t, t)
    
    return min(0.95, max_t + 0.1)


def calculate_path_length_meters(path: List[List[float]]) -> float:
    """计算路径总长度（米）"""
    total = 0.0
    for i in range(len(path) - 1):
        total += distance_meters(path[i], path[i + 1])
    return total


def find_best_path(start: List[float], end: List[float], 
                   obstacles_gcj: List[Dict], flight_altitude: float,
                   safety_radius: float = 5) -> Tuple[List[List[float]], str]:
    """
    找到最佳绕行路径（比较左右绕行和飞越）
    
    Returns:
        (路径, 策略名称)
    """
    left_path = find_horizontal_path(start, end, obstacles_gcj, flight_altitude, "left", safety_radius)
    right_path = find_horizontal_path(start, end, obstacles_gcj, flight_altitude, "right", safety_radius)
    above_path = find_above_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    left_len = calculate_path_length_meters(left_path)
    right_len = calculate_path_length_meters(right_path)
    above_len = calculate_path_length_meters(above_path)
    
    # 找出最短路径
    if left_len <= right_len and left_len <= above_len:
        return left_path, "向左绕行"
    elif right_len <= left_len and right_len <= above_len:
        return right_path, "向右绕行"
    else:
        return above_path, "向上飞越"


def create_avoidance_path(start: List[float], end: List[float], 
                          obstacles_gcj: List[Dict], flight_altitude: float,
                          direction: str = "最佳航线", safety_radius: float = 5) -> List[List[float]]:
    """
    创建避障路径的统一入口
    
    Args:
        direction: "向左绕行"、"向右绕行"、"向上飞越" 或 "最佳航线"（自动选择最短路径）
        safety_radius: 安全半径（米）
    """
    # 过滤出需要避让的障碍物（高度大于飞行高度）
    relevant_obstacles = [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude]
    
    # 如果没有需要避让的障碍物，直接返回直线路径
    if not relevant_obstacles:
        return [start, end]
    
    # 检查直线是否安全
    if not get_blocking_obstacles(start, end, relevant_obstacles, flight_altitude):
        return [start, end]
    
    if direction == "向左绕行":
        return find_horizontal_path(start, end, relevant_obstacles, flight_altitude, "left", safety_radius)
    elif direction == "向右绕行":
        return find_horizontal_path(start, end, relevant_obstacles, flight_altitude, "right", safety_radius)
    elif direction == "向上飞越":
        return find_above_path(start, end, relevant_obstacles, flight_altitude, safety_radius)
    else:  # 最佳航线
        path, _ = find_best_path(start, end, relevant_obstacles, flight_altitude, safety_radius)
        return path


# ==================== 兼容原代码的包装函数 ====================

def get_blocking_obstacles_original(start: List[float], end: List[float], 
                                     obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    """与原代码接口兼容的包装函数"""
    return get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)


def calculate_path_length_original(path: List[List[float]]) -> float:
    """与原代码接口兼容的路径长度计算"""
    return calculate_path_length_meters(path)


def distance_deg(point1: List[float], point2: List[float]) -> float:
    """经纬度差值距离（用于原代码中的比例计算）"""
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)


# ==================== 心跳包模拟器 ====================
@dataclass
class HeartbeatData:
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float

class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history: List[HeartbeatData] = []
        self.current_pos: List[float] = start_point_gcj.copy()
        self.path: List[List[float]] = [start_point_gcj.copy()]
        self.path_index: int = 0
        self.simulating: bool = False
        self.flight_altitude: float = 50
        self.speed: int = 50
        self.progress: float = 0.0
        self.total_distance: float = 0.0
        self.distance_traveled: float = 0.0
        self.safety_radius: float = config.DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation: bool = False
        self.start_time: Optional[datetime] = None
        self.flight_log: List[HeartbeatData] = []
        self.last_update_time: Optional[float] = None
        
    def set_path(self, path: List[List[float]], altitude: float = 50, speed: int = 50, safety_radius: float = 15):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += calculate_distance_meters(
                (path[i][1], path[i][0]), 
                (path[i+1][1], path[i+1][0])
            )
    
    def update_and_generate(self, obstacles_gcj: List[Dict]) -> Optional[HeartbeatData]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
            return None
        
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = config.HEARTBEAT_INTERVAL
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = calculate_distance_meters((start[1], start[0]), (end[1], end[0]))
        
        speed_m_per_s = config.BASE_SPEED_MPS * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            self.progress = min(1.0, self.distance_traveled / self.total_distance)
        
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        safe, _, _ = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius)
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        remaining = 0
        if self.path_index < len(self.path) - 1:
            remaining += calculate_distance_meters(
                (self.current_pos[1], self.current_pos[0]),
                (self.path[self.path_index + 1][1], self.path[self.path_index + 1][0])
            )
            for i in range(self.path_index + 1, len(self.path) - 1):
                remaining += calculate_distance_meters(
                    (self.path[i][1], self.path[i][0]),
                    (self.path[i+1][1], self.path[i+1][0])
                )
        
        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-config.VOLTAGE_VARIATION, config.VOLTAGE_VARIATION), 1),
            satellites=random.randint(*config.SAT_RANGE),
            speed=round(config.BASE_SPEED_MPS * (self.speed / 100), 1),
            progress=self.progress,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=remaining
        )
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        
        return heartbeat
    
    def export_flight_data(self) -> pd.DataFrame:
        if not self.flight_log:
            return pd.DataFrame()
        
        data = [{
            'timestamp': h.timestamp,
            'flight_time': h.flight_time,
            'lat': h.lat,
            'lng': h.lng,
            'altitude': h.altitude,
            'voltage': h.voltage,
            'satellites': h.satellites,
            'speed': h.speed,
            'progress': h.progress,
            'arrived': h.arrived,
            'safety_violation': h.safety_violation,
            'remaining_distance': h.remaining_distance
        } for h in self.flight_log]
        
        return pd.DataFrame(data)

# ==================== 地图创建 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, obstacles_gcj: List[Dict], 
                        flight_history: Optional[List] = None, planned_path: Optional[List] = None, 
                        map_type: str = "satellite", straight_blocked: bool = True, 
                        flight_altitude: float = 50, drone_pos: Optional[List] = None, 
                        direction: str = "最佳航线", safety_radius: float = 15) -> folium.Map:
    """创建规划地图"""
    if map_type == "satellite":
        tiles = config.GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = config.GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 
                                  'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 
                      'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    # 绘制障碍物
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, 
                          fill=True, fill_color=color, fill_opacity=0.4, 
                          popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    # 绘制起点终点
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", 
                     icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", 
                     icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    # 绘制规划路径
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        if "向左" in direction:
            line_color = "purple"
        elif "向右" in direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(path_locations, color=line_color, weight=5, opacity=0.9, 
                       popup=f"✈️ {direction}").add_to(m)
        
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color=line_color, 
                               fill=True, fill_color="white", fill_opacity=0.8, 
                               popup=f"航点 {i+1}").add_to(m)
    
    # 绘制直线
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], 
                            [points_gcj['B'][1], points_gcj['B'][0]]], 
                           color="blue", weight=2, opacity=0.5, dash_array='5, 5', 
                           popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], 
                            [points_gcj['B'][1], points_gcj['B'][0]]], 
                           color="gray", weight=2, opacity=0.4, dash_array='5, 5', 
                           popup="⚠️ 直线被阻挡").add_to(m)
    
    # 绘制安全半径
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]], 
                     color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2, 
                     popup=f"🛡️ 安全半径: {safety_radius}米").add_to(m)
    
    # 绘制历史轨迹
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    
    return m

# ==================== 辅助UI函数 ====================
def init_session_state():
    """初始化Session State"""
    defaults = {
        'points_gcj': {'A': config.DEFAULT_A_GCJ.copy(), 'B': config.DEFAULT_B_GCJ.copy()},
        'obstacles_gcj': load_obstacles(),
        'heartbeat_sim': HeartbeatSimulator(config.DEFAULT_A_GCJ.copy()),
        'last_hb_time': time.time(),
        'simulation_running': False,
        'flight_history': [],
        'planned_path': None,
        'last_flight_altitude': 50,
        'pending_obstacle': None,
        'current_direction': "最佳航线",
        'safety_radius': config.DEFAULT_SAFETY_RADIUS_METERS,
        'auto_backup': True,
        'show_rename_dialog': False,
        'mission_active': False,
        'mission_paused': False,
        'mission_start_time': None,
        'flight_speed': 8.5,
        'battery_level': 100,
        'flight_log': [],
        'heartbeat_history': [],
        'heartbeat_running': False,
        'gcs_status': "在线",
        'obc_status': "在线",
        'fcu_status': "在线"
    }
    
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value
    
    for obs in st.session_state.obstacles_gcj:
        if 'height' not in obs:
            obs['height'] = 30
        if 'selected' not in obs:
            obs['selected'] = False

def check_straight_blocked(points_gcj: Dict, obstacles_gcj: List[Dict], flight_altitude: float) -> Tuple[bool, int]:
    """检查直线是否被阻挡"""
    blocked = False
    high_count = 0
    
    start = (points_gcj['A'][1], points_gcj['A'][0])
    end = (points_gcj['B'][1], points_gcj['B'][0])
    
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            high_count += 1
            coords = obs.get('polygon', [])
            if coords:
                polygon = [[c[1], c[0]] for c in coords]
                if line_intersects_polygon(start, end, polygon):
                    blocked = True
    
    return blocked, high_count

def render_sidebar() -> Tuple[str, str, int, float, bool]:
    """渲染侧边栏"""
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=5, max_value=30, 
                                      value=st.session_state.safety_radius, step=1)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    
    return page, map_type, drone_speed, flight_alt, auto_save

# ==================== 页面渲染函数 ====================
def render_planning_page(map_type: str, drone_speed: int, flight_alt: float, auto_save: bool):
    """渲染航线规划页面"""
    st.header("🗺️ 航线规划 - 智能避障")
    
    straight_blocked, high_obstacles = check_straight_blocked(
        st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_alt
    )
    
    if straight_blocked:
        st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
    else:
        st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
    
    st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
    
    col1, col2 = st.columns([1, 1.5])
    
    with col1:
        render_planning_controls(flight_alt, drone_speed, auto_save)
    
    with col2:
        render_planning_map_view(map_type, flight_alt, straight_blocked)

def render_planning_controls(flight_alt: float, drone_speed: int, auto_save: bool):
    """渲染规划控制面板"""
    st.subheader("🎮 控制面板")
    
    with st.expander("📍 起点/终点设置", expanded=True):
        render_point_settings()
    
    with st.expander("🤖 路径规划策略", expanded=True):
        render_path_strategy(flight_alt)
    
    with st.expander("✈️ 飞行控制", expanded=True):
        render_flight_controls(flight_alt, drone_speed)
    
    st.markdown("### 📍 当前坐标")
    st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
    st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
    
    a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
    dist = calculate_distance_meters((a[1], a[0]), (b[1], b[0]))
    st.caption(f"📏 直线距离: {dist:.0f} 米")
    
    # 路径验证信息
    if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
        is_safe, collisions = validate_path_safety(
            st.session_state.planned_path, 
            st.session_state.obstacles_gcj, 
            flight_alt
        )
        if is_safe:
            st.success("✅ 当前规划路径安全")
        else:
            st.error("⚠️ 路径验证失败！")
            for coll in collisions[:3]:
                st.caption(f"• {coll}")

def render_point_settings():
    """渲染点设置"""
    st.markdown("#### 🟢 起点 A")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], 
                                format="%.6f", key="a_lat", step=0.000001)
    with col_a2:
        a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], 
                                format="%.6f", key="a_lng", step=0.000001)
    
    if st.button("📍 设置 A 点", use_container_width=True):
        st.session_state.points_gcj['A'] = [a_lng, a_lat]
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        st.rerun()
    
    st.markdown("#### 🔴 终点 B")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], 
                                format="%.6f", key="b_lat", step=0.000001)
    with col_b2:
        b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], 
                                format="%.6f", key="b_lng", step=0.000001)
    
    if st.button("📍 设置 B 点", use_container_width=True):
        st.session_state.points_gcj['B'] = [b_lng, b_lat]
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        st.rerun()

def render_path_strategy(flight_alt: float):
    """渲染路径规划策略"""
    st.markdown("**选择绕行方向：**")
    col_dir1, col_dir2, col_dir3 = st.columns(3)
    
    with col_dir1:
        if st.button("🔄 最佳航线", use_container_width=True, 
                    type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
            st.session_state.current_direction = "最佳航线"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "最佳航线",
                st.session_state.safety_radius
            )
            st.success("已切换到最佳航线模式")
            st.rerun()
    
    with col_dir2:
        if st.button("⬅️ 向左绕行", use_container_width=True,
                    type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
            st.session_state.current_direction = "向左绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向左绕行",
                st.session_state.safety_radius
            )
            st.success("已切换到向左绕行模式")
            st.rerun()
    
    with col_dir3:
        if st.button("➡️ 向右绕行", use_container_width=True,
                    type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
            st.session_state.current_direction = "向右绕行"
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, "向右绕行",
                st.session_state.safety_radius
            )
            st.success("已切换到向右绕行模式")
            st.rerun()
    
    st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
    
    if st.button("🔄 重新规划路径", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
        if st.session_state.planned_path:
            waypoint_count = len(st.session_state.planned_path) - 2
            total_dist = calculate_path_length_meters(st.session_state.planned_path)
            st.success(f"已按照「{st.session_state.current_direction}」规划路径，{waypoint_count}个绕行点，总长 {total_dist:.0f}米")
        st.rerun()

def render_flight_controls(flight_alt: float, drone_speed: int):
    """渲染飞行控制"""
    col_met1, col_met2, col_met3 = st.columns(3)
    with col_met1:
        st.metric("当前飞行高度", f"{flight_alt} m")
    with col_met2:
        st.metric("速度系数", f"{drone_speed}%")
    with col_met3:
        st.metric("🛡️ 安全半径", f"{st.session_state.safety_radius} 米")
    
    if st.session_state.planned_path:
        waypoint_count = len(st.session_state.planned_path) - 2
        st.metric("🎯 绕行点数量", waypoint_count)
        total_dist = calculate_path_length_meters(st.session_state.planned_path)
        st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, st.session_state.safety_radius)
                st.session_state.simulation_running = True
                st.session_state.flight_history = []
                waypoint_count = len(path) - 2
                st.success(f"🚁 飞行已开始！{'路径中有' + str(waypoint_count) + '个绕行点' if waypoint_count > 0 else '直线飞行'}")
                st.rerun()
            else:
                st.error("请先设置起点和终点")
    
    with col_btn2:
        if st.button("⏹️ 停止飞行", use_container_width=True):
            st.session_state.simulation_running = False
            st.session_state.heartbeat_sim.simulating = False
            st.info("飞行已停止")

def render_planning_map_view(map_type: str, flight_alt: float, straight_blocked: bool):
    """渲染规划地图视图"""
    st.subheader("🗺️ 规划地图")
    if straight_blocked:
        st.caption(f"当前避障策略: {st.session_state.current_direction}")
    st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色圆圈=安全半径")
    
    flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history[:20]]
    center = st.session_state.points_gcj['A'] or config.SCHOOL_CENTER_GCJ
    
    if st.session_state.planned_path is None:
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj, flight_alt,
            st.session_state.current_direction, st.session_state.safety_radius
        )
    
    drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
    
    m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj,
                           flight_trail, st.session_state.planned_path, map_type,
                           straight_blocked, flight_alt, drone_pos,
                           st.session_state.current_direction, st.session_state.safety_radius)
    
    output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
    
    handle_drawing_output(output)

def handle_drawing_output(output: Any):
    """处理绘图输出"""
    if output and output.get("last_active_drawing"):
        last = output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()

def render_obstacle_dialog():
    """渲染障碍物添加对话框"""
    st.markdown("---")
    st.subheader("📝 添加新障碍物")
    st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
    
    col_name1, col_name2 = st.columns(2)
    with col_name1:
        new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
    with col_name2:
        new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, value=30, step=5, key="height_input")
    
    col_ok, col_cancel = st.columns(2)
    with col_ok:
        if st.button("✅ 确认添加", use_container_width=True, type="primary"):
            new_obstacle = {
                "name": new_name,
                "polygon": st.session_state.pending_obstacle,
                "height": new_height,
                "selected": False,
                "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
                "created_time": datetime.now().isoformat()
            }
            st.session_state.obstacles_gcj.append(new_obstacle)
            if st.session_state.auto_backup:
                save_obstacles(st.session_state.obstacles_gcj)
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, st.session_state.last_flight_altitude,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.session_state.pending_obstacle = None
            st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
            st.rerun()
    with col_cancel:
        if st.button("❌ 取消", use_container_width=True):
            st.session_state.pending_obstacle = None
            st.rerun()

# ==================== 飞行监控页面 ====================
def render_flight_monitoring_page(map_type: str, flight_alt: float, drone_speed: int):
    """渲染飞行监控页面"""
    st.header("📡 飞行监控 - 实时心跳包")
    
    update_flight_simulation()
    
    if st.session_state.heartbeat_sim.history:
        latest = st.session_state.heartbeat_sim.history[0]
        
        current_waypoint = 0
        total_waypoints = 0
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            total_waypoints = len(st.session_state.planned_path) - 2
            if latest.progress < 1.0:
                segment_index = int(latest.progress * (len(st.session_state.planned_path) - 1))
                current_waypoint = max(0, min(total_waypoints, segment_index))
            else:
                current_waypoint = total_waypoints
        
        estimated_arrival = "计算中..."
        if latest.speed > 0 and latest.remaining_distance > 0:
            eta_seconds = latest.remaining_distance / latest.speed
            if eta_seconds < 60:
                estimated_arrival = f"{eta_seconds:.0f}秒"
            elif eta_seconds < 3600:
                estimated_arrival = f"{eta_seconds/60:.1f}分钟"
            else:
                estimated_arrival = f"{eta_seconds/3600:.1f}小时"
        elif latest.arrived:
            estimated_arrival = "已到达"
        
        max_flight_time = 1800
        battery_percentage = max(0, min(100, (1 - latest.flight_time / max_flight_time) * 100))
        if latest.voltage:
            voltage_percentage = ((latest.voltage - 21.0) / (22.2 - 21.0)) * 100
            battery_percentage = max(0, min(100, (battery_percentage + voltage_percentage) / 2))
        
        st.markdown("### ✈️ 飞行进度")
        progress_percent = int(latest.progress * 100)
        st.progress(latest.progress, text=f"飞行进度：{progress_percent}%")
        
        st.markdown("### 📊 实时飞行数据")
        
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("🎯 当前航点", f"{current_waypoint} / {total_waypoints}")
            if total_waypoints > 0:
                waypoint_progress = current_waypoint / total_waypoints
                st.progress(waypoint_progress, text=f"航点进度: {int(waypoint_progress*100)}%")
        
        with col2:
            st.metric("💨 飞行速度", f"{latest.speed:.1f} m/s", delta=f"{drone_speed}% 系数")
            speed_kmh = latest.speed * 3.6
            st.caption(f"≈ {speed_kmh:.1f} km/h")
        
        with col3:
            st.metric("⏰ 已用时间", f"{int(latest.flight_time // 60):02d}:{int(latest.flight_time % 60):02d}")
        
        col4, col5, col6 = st.columns(3)
        with col4:
            remaining_km = latest.remaining_distance / 1000
            distance_text = f"{remaining_km:.2f} km" if remaining_km >= 1 else f"{latest.remaining_distance:.0f} m"
            st.metric("📏 剩余距离", distance_text)
        
        with col5:
            st.metric("🕐 预计到达", estimated_arrival)
            if latest.remaining_distance < 100 and not latest.arrived:
                st.info("🏁 即将到达目的地！")
        
        with col6:
            battery_color = "🟢" if battery_percentage > 50 else "🟡" if battery_percentage > 20 else "🔴"
            st.metric("🔋 电量模拟", f"{battery_color} {battery_percentage:.0f}%", delta=f"{latest.voltage:.1f}V")
            if battery_percentage < 20:
                st.warning("⚠️ 电量不足，请尽快返航！")
        
        if latest.safety_violation:
            st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
        
        if latest.arrived:
            st.success("🎉 无人机已到达目的地！飞行任务完成！")
        
        st.markdown("---")
        
        st.markdown("### 🗺️ 实时位置追踪")
        display_monitor_map(map_type, latest, flight_alt)
        
        st.markdown("---")
        
        st.markdown("### 📋 飞行日志记录")
        display_flight_history()
        
        col_export1, col_export2 = st.columns(2)
        with col_export1:
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                if not df.empty:
                    csv = df.to_csv(index=False)
                    st.download_button(
                        label="📥 下载CSV文件",
                        data=csv,
                        file_name=f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                        mime="text/csv",
                        use_container_width=True
                    )
        
        with col_export2:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
                st.success("飞行已停止")
                st.rerun()
                
    else:
        st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            st.markdown("---")
            st.subheader("🗺️ 规划航线预览")
            total_waypoints = len(st.session_state.planned_path) - 2
            total_dist = calculate_path_length_meters(st.session_state.planned_path)
            st.success(f"📌 已规划 {total_waypoints} 个航点，总长 {total_dist:.0f}米")
            
            with st.expander("📋 查看详细航点列表"):
                waypoint_table = []
                for i, wp in enumerate(st.session_state.planned_path):
                    wp_type = "起点" if i == 0 else "终点" if i == len(st.session_state.planned_path)-1 else f"航点{i}"
                    waypoint_table.append({
                        "序号": i,
                        "类型": wp_type,
                        "经度": f"{wp[0]:.6f}",
                        "纬度": f"{wp[1]:.6f}"
                    })
                st.table(pd.DataFrame(waypoint_table))

def display_monitor_map(map_type: str, latest, flight_alt: float):
    """显示监控地图"""
    tiles = config.GAODE_SATELLITE_URL if map_type == "satellite" else config.GAODE_VECTOR_URL
    monitor_map = folium.Map(location=[latest.lat, latest.lng], zoom_start=18, 
                             tiles=tiles, attr="高德地图")
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_alt else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, 
                          weight=2, fill=True, fill_opacity=0.3,
                          popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(monitor_map)
    
    if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
        if "向左" in st.session_state.current_direction:
            line_color = "purple"
        elif "向右" in st.session_state.current_direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], 
                       color=line_color, weight=3, opacity=0.7,
                       popup=f"规划航线 - {st.session_state.current_direction}").add_to(monitor_map)
    
    folium.Circle(
        radius=st.session_state.safety_radius, 
        location=[latest.lat, latest.lng],
        color="blue", weight=2, fill=True,
        fill_color="blue", fill_opacity=0.2,
        popup=f"🛡️ 安全半径: {st.session_state.safety_radius}米"
    ).add_to(monitor_map)
    
    trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:50] 
             if hb.lat and hb.lng]
    if len(trail) > 1:
        folium.PolyLine(trail, color="orange", weight=2, opacity=0.6,
                       popup="历史飞行轨迹").add_to(monitor_map)
    
    folium.Marker(
        [latest.lat, latest.lng], 
        popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s", 
        icon=folium.Icon(color='red', icon='plane', prefix='fa')
    ).add_to(monitor_map)
    
    if st.session_state.points_gcj['A']:
        folium.Marker(
            [st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
            popup="起点 A", 
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(monitor_map)
    
    if st.session_state.points_gcj['B']:
        folium.Marker(
            [st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
            popup="终点 B", 
            icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
        ).add_to(monitor_map)
    
    if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
        for i, point in enumerate(st.session_state.planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]], 
                radius=4, color="yellow", fill=True,
                fill_color="yellow", fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(monitor_map)
    
    folium_static(monitor_map, width=900, height=500)

def display_flight_history():
    """显示飞行历史记录"""
    history_df = st.session_state.heartbeat_sim.export_flight_data()
    
    if not history_df.empty:
        display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites', 'remaining_distance']
        display_cols = [col for col in display_cols if col in history_df.columns]
        
        recent_df = history_df[display_cols].head(10)
        
        column_names = {
            'timestamp': '时间', 'flight_time': '飞行时间(s)', 'lat': '纬度',
            'lng': '经度', 'altitude': '高度(m)', 'speed': '速度(m/s)',
            'voltage': '电压(V)', 'satellites': '卫星数', 'remaining_distance': '剩余距离(m)'
        }
        recent_df = recent_df.rename(columns=column_names)
        
        st.dataframe(recent_df, use_container_width=True)
        
        st.markdown("### 📊 飞行统计")
        col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
        
        with col_stat1:
            max_speed = history_df['speed'].max() if 'speed' in history_df.columns else 0
            st.metric("🏁 最高速度", f"{max_speed:.1f} m/s")
        
        with col_stat2:
            avg_speed = history_df['speed'].mean() if 'speed' in history_df.columns else 0
            st.metric("📈 平均速度", f"{avg_speed:.1f} m/s")
        
        with col_stat3:
            max_alt = history_df['altitude'].max() if 'altitude' in history_df.columns else 0
            st.metric("⛰️ 最高高度", f"{max_alt:.0f} m")
        
        with col_stat4:
            total_time = history_df['flight_time'].max() if 'flight_time' in history_df.columns else 0
            st.metric("⏱️ 总飞行时间", f"{total_time:.1f} s")
    else:
        st.info("暂无飞行数据")

def update_flight_simulation():
    """更新飞行模拟"""
    current_time = time.time()
    if st.session_state.simulation_running:
        if current_time - st.session_state.last_hb_time >= config.HEARTBEAT_INTERVAL:
            try:
                new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
                if new_hb:
                    st.session_state.last_hb_time = current_time
                    st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
                    if len(st.session_state.flight_history) > 200:
                        st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
                        st.success("🏁 无人机已安全到达目的地！")
                    st.rerun()
            except Exception as e:
                st.error(f"更新心跳时出错: {e}")
    else:
        st.session_state.last_hb_time = current_time

# ==================== 障碍物管理页面 ====================
def render_obstacle_management_page(flight_alt: float):
    """渲染障碍物管理页面"""
    st.header("🚧 障碍物管理")
    
    col_status1, col_status2, col_status3, col_status4 = st.columns(4)
    with col_status1:
        st.info(f"📊 当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
    with col_status2:
        st.info(f"🛡️ 安全半径: {st.session_state.safety_radius}米")
    with col_status3:
        if os.path.exists(config.CONFIG_FILE):
            try:
                with open(config.CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    save_time = data.get('save_time', '未知')
                    st.info(f"💾 最后保存: {save_time}")
            except:
                st.info("💾 未保存")
        else:
            st.info("💾 未保存")
    with col_status4:
        backup_count = len([f for f in os.listdir(config.BACKUP_DIR) if f.startswith(config.CONFIG_FILE) and f.endswith('.bak')])
        st.info(f"📦 备份数量: {backup_count}")
    
    st.markdown("---")
    
    col_data1, col_data2, col_data3, col_data4, col_data5 = st.columns(5)
    
    with col_data1:
        if st.button("💾 保存配置", use_container_width=True, type="primary"):
            if save_obstacles(st.session_state.obstacles_gcj):
                st.success(f"✅ 已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
                st.balloons()
                time.sleep(0.5)
                st.rerun()
    
    with col_data2:
        if st.button("📂 加载配置", use_container_width=True):
            loaded = load_obstacles()
            if loaded:
                st.session_state.obstacles_gcj = loaded
                update_path_after_obstacle_change(flight_alt)
                st.success(f"✅ 已加载 {len(loaded)} 个障碍物")
                st.rerun()
            else:
                st.warning("⚠️ 未找到配置文件")
    
    with col_data3:
        if st.session_state.obstacles_gcj:
            config_data = {
                'obstacles': st.session_state.obstacles_gcj,
                'count': len(st.session_state.obstacles_gcj),
                'export_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'version': 'v3.0'
            }
            json_str = json.dumps(config_data, ensure_ascii=False, indent=2)
            st.download_button(
                label="📥 导出配置",
                data=json_str,
                file_name=f"obstacles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                mime="application/json",
                use_container_width=True
            )
        else:
            st.button("📥 导出配置", use_container_width=True, disabled=True)
    
    with col_data4:
        latest_backup = get_latest_backup()
        if latest_backup:
            if st.button("🔄 恢复备份", use_container_width=True):
                if restore_from_backup(latest_backup):
                    st.session_state.obstacles_gcj = load_obstacles()
                    update_path_after_obstacle_change(flight_alt)
                    st.success("✅ 已从备份恢复")
                    st.rerun()
                else:
                    st.error("❌ 恢复失败")
        else:
            st.button("🔄 恢复备份", use_container_width=True, disabled=True)
    
    with col_data5:
        if st.button("🗑️ 清除全部", use_container_width=True):
            if st.session_state.auto_backup:
                backup_config()
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            update_path_after_obstacle_change(flight_alt)
            st.success("✅ 已清除所有障碍物")
            st.rerun()
    
    st.markdown("---")
    
    col_stats1, col_stats2, col_stats3, col_stats4 = st.columns(4)
    with col_stats1:
        high_obs = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
        st.metric("🔴 需避让障碍物", high_obs)
    with col_stats2:
        safe_obs = len(st.session_state.obstacles_gcj) - high_obs
        st.metric("🟠 安全障碍物", safe_obs)
    with col_stats3:
        total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
        st.metric("📍 总顶点数", total_vertices)
    with col_stats4:
        avg_height = sum(obs.get('height', 30) for obs in st.session_state.obstacles_gcj) / max(1, len(st.session_state.obstacles_gcj))
        st.metric("📏 平均高度", f"{avg_height:.1f}m")
    
    st.markdown("---")
    
    tab_list, tab_map = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
    
    with tab_list:
        render_obstacle_list_view(flight_alt)
    
    with tab_map:
        render_obstacle_map_view(flight_alt)

def render_obstacle_list_view(flight_alt: float):
    """渲染障碍物列表视图"""
    st.subheader("📝 障碍物列表")
    st.caption("💡 提示：点击删除按钮移除障碍物")
    
    if st.session_state.obstacles_gcj:
        for idx, obs in enumerate(st.session_state.obstacles_gcj):
            with st.container(border=True):
                height = obs.get('height', 30)
                color = "🔴" if height > flight_alt else "🟠"
                name = obs.get('name', f'障碍物{idx+1}')
                
                col_name, col_height, col_btn = st.columns([3, 2, 1])
                with col_name:
                    st.markdown(f"**{color} {name}**")
                with col_height:
                    new_h = st.number_input("高度(m)", value=height, min_value=1, max_value=200, 
                                           step=5, key=f"height_{idx}", label_visibility="collapsed")
                    if new_h != height:
                        obs['height'] = new_h
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        update_path_after_obstacle_change(flight_alt)
                        st.rerun()
                with col_btn:
                    if st.button("🗑️", key=f"delete_{idx}"):
                        st.session_state.obstacles_gcj.pop(idx)
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        update_path_after_obstacle_change(flight_alt)
                        st.rerun()
                
                st.caption(f"📍 顶点数: {len(obs.get('polygon', []))}")
    else:
        st.info("📭 暂无任何障碍物，可以在「地图视图」中绘制添加")

def render_obstacle_map_view(flight_alt: float):
    """渲染障碍物地图视图"""
    st.subheader("🗺️ 地图视图")
    st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")
    
    map_view_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
    map_type_view = "satellite" if map_view_type == "卫星影像" else "vector"
    
    tiles = config.GAODE_SATELLITE_URL if map_type_view == "satellite" else config.GAODE_VECTOR_URL
    obs_map = folium.Map(location=[config.SCHOOL_CENTER_GCJ[1], config.SCHOOL_CENTER_GCJ[0]], 
                         zoom_start=16, tiles=tiles, attr="高德地图")
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 
                                  'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 
                      'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    obs_map.add_child(draw)
    
    for obs in st.session_state.obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        color = "red" if height > flight_alt else "orange"
        if coords and len(coords) >= 3:
            popup_text = f"""
            <div style="font-family: sans-serif;">
                <b>🏢 {obs.get('name')}</b><br>
                高度: {height} 米<br>
                ID: {obs.get('id', 'N/A')}<br>
            </div>
            """
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, 
                          fill=True, fill_color=color, fill_opacity=0.5, 
                          popup=folium.Popup(popup_text, max_width=300)).add_to(obs_map)
    
    folium.Marker([config.DEFAULT_A_GCJ[1], config.DEFAULT_A_GCJ[0]], popup="起点", 
                 icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
    folium.Marker([config.DEFAULT_B_GCJ[1], config.DEFAULT_B_GCJ[0]], popup="终点", 
                 icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(obs_map)
    
    map_output = st_folium(obs_map, width=800, height=550, key="obstacle_map_view", 
                          returned_objects=["last_active_drawing"])
    
    if map_output and map_output.get("last_active_drawing"):
        last = map_output["last_active_drawing"]
        if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
            coords = last["geometry"].get("coordinates", [])
            if coords and len(coords) > 0:
                poly = [[p[0], p[1]] for p in coords[0]]
                if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                    if validate_polygon(poly):
                        st.session_state.pending_obstacle = poly
                        st.rerun()
    
    if st.session_state.pending_obstacle is not None:
        render_obstacle_dialog()

def update_path_after_obstacle_change(flight_alt: float):
    """障碍物变更后更新路径"""
    st.session_state.planned_path = create_avoidance_path(
        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj, flight_alt,
        st.session_state.current_direction, st.session_state.safety_radius
    )

# ==================== 主程序 ====================
def main():
    st.set_page_config(page_title="无人机地面站系统", layout="wide")
    
    init_session_state()
    
    st.title("🚁 无人机地面站系统")
    st.markdown("---")
    
    page, map_type, drone_speed, flight_alt, auto_save = render_sidebar()
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt,
                st.session_state.current_direction, st.session_state.safety_radius
            )
            st.rerun()
    
    if page == "🗺️ 航线规划":
        render_planning_page(map_type, drone_speed, flight_alt, auto_save)
    elif page == "📡 飞行监控":
        render_flight_monitoring_page(map_type, flight_alt, drone_speed)
    elif page == "🚧 障碍物管理":
        render_obstacle_management_page(flight_alt)

if __name__ == "__main__":
    main()
