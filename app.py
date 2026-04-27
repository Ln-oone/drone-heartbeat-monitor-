import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
import shutil
import logging
from datetime import datetime
from typing import List, Dict, Tuple, Optional
import pandas as pd
import numpy as np

# ==================== 日志配置 ====================
if not logging.getLogger().handlers:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('flight_system.log'),
            logging.StreamHandler()
        ]
    )
logger = logging.getLogger(__name__)

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="南京科技职业学院 - 无人机地面站系统", 
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==================== 常量定义 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

CONFIG_FILE = "obstacle_config.json"
BACKUP_DIR = "backups"
DEFAULT_SAFETY_RADIUS_METERS = 5
DEFAULT_FLIGHT_ALTITUDE = 50
MAX_FLIGHT_ALTITUDE = 200
MIN_FLIGHT_ALTITUDE = 10
MAX_HISTORY_SIZE = 100
MAX_FLIGHT_LOG_SIZE = 1000
MAX_BACKUP_FILES = 10

GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

os.makedirs(BACKUP_DIR, exist_ok=True)

# ==================== 坐标验证 ====================
def validate_coordinates(lng: float, lat: float) -> bool:
    if not (-180 <= lng <= 180):
        raise ValueError(f"经度超出有效范围: {lng}")
    if not (-90 <= lat <= 90):
        raise ValueError(f"纬度超出有效范围: {lat}")
    return True

def validate_polygon(polygon: List[List[float]]) -> bool:
    if len(polygon) < 3:
        return False
    for i in range(len(polygon)):
        for j in range(i + 2, len(polygon)):
            if i == 0 and j == len(polygon) - 1:
                continue
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            p3 = polygon[j]
            p4 = polygon[(j + 1) % len(polygon)]
            if segments_intersect(p1, p2, p3, p4):
                return False
    return True

# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2

def segments_intersect(p1: List[float], p2: List[float], 
                       p3: List[float], p4: List[float]) -> bool:
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

def line_intersects_polygon(p1: List[float], p2: List[float], 
                           polygon: List[List[float]]) -> bool:
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def distance(p1: List[float], p2: List[float]) -> float:
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
    if not polygon:
        return None
    min_lng = min(p[0] for p in polygon)
    max_lng = max(p[0] for p in polygon)
    min_lat = min(p[1] for p in polygon)
    max_lat = max(p[1] for p in polygon)
    return {
        'min_lng': min_lng, 'max_lng': max_lng,
        'min_lat': min_lat, 'max_lat': max_lat,
        'center_lng': (min_lng + max_lng) / 2,
        'center_lat': (min_lat + max_lat) / 2
    }

def point_to_segment_distance_meters(point: List[float], 
                                     seg_start: List[float], 
                                     seg_end: List[float]) -> float:
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    
    if len_sq < 1e-10:
        dist_deg = math.sqrt((px-x1)**2 + (py-y1)**2)
    else:
        t = ((px - x1) * dx + (py - y1) * dy) / len_sq
        t = max(0, min(1, t))
        
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        dist_deg = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    
    lat_mid = (point[1] + seg_start[1] + seg_end[1]) / 3
    return dist_deg * (111000 * math.cos(math.radians(lat_mid)))

def check_safety_radius(drone_pos: List[float], 
                       obstacles_gcj: List[Dict], 
                       flight_altitude: float, 
                       safety_radius: float) -> Tuple[bool, Optional[float], Optional[str]]:
    if not drone_pos:
        return True, None, None
    
    min_distance = float('inf')
    danger_name = None
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        obs_height = obs.get('height', 30)
        
        if obs_height <= flight_altitude:
            continue
        
        if coords and len(coords) >= 3:
            for i in range(len(coords)):
                p1 = coords[i]
                p2 = coords[(i + 1) % len(coords)]
                
                dist_m = point_to_segment_distance_meters(drone_pos, p1, p2)
                
                if dist_m < min_distance:
                    min_distance = dist_m
                    danger_name = obs.get('name', '障碍物')
    
    if min_distance < safety_radius:
        return False, min_distance, danger_name
    return True, min_distance if min_distance != float('inf') else None, None

def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
    lat_rad = math.radians(lat)
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(lat_rad))
    return lng_deg, lat_deg

# ==================== 避障算法 ====================
def get_blocking_obstacles(start: List[float], end: List[float], 
                          obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                if line_intersects_polygon(start, end, coords):
                    blocking_obs.append(obs)
    return blocking_obs

def get_obstacle_bounds(obstacles: List[Dict]) -> Optional[Dict]:
    if not obstacles:
        return None
    
    all_points = []
    for obs in obstacles:
        all_points.extend(obs.get('polygon', []))
    
    if not all_points:
        return None
    
    return {
        'min_lng': min(p[0] for p in all_points),
        'max_lng': max(p[0] for p in all_points),
        'min_lat': min(p[1] for p in all_points),
        'max_lat': max(p[1] for p in all_points),
    }

def is_path_clear(p1: List[float], p2: List[float], obstacles: List[Dict]) -> bool:
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords and line_intersects_polygon(p1, p2, coords):
            return False
    return True

def validate_path_does_not_cross_obstacles(path: List[List[float]], 
                                           obstacles: List[Dict], 
                                           flight_altitude: float) -> bool:
    """
    验证路径是否穿过任何高于飞行高度的障碍物
    返回True表示路径安全，False表示有碰撞
    """
    if not path or len(path) < 2:
        return False
    
    # 只考虑高于飞行高度的障碍物
    high_obstacles = [obs for obs in obstacles if obs.get('height', 30) > flight_altitude]
    
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        
        for obs in high_obstacles:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(p1, p2, coords):
                logger.error(f"路径段 {i} 穿过障碍物 {obs.get('name')}!")
                logger.error(f"  从 ({p1[0]:.6f}, {p1[1]:.6f}) 到 ({p2[0]:.6f}, {p2[1]:.6f})")
                return False
    
    return True

def find_left_boundary_path(start: List[float], end: List[float], 
                           obstacles: List[Dict], safety_radius: float = 5) -> List[List[float]]:
    """
    向左绕行 - 严格沿障碍物左侧边界绕行，绝不触碰障碍物
    
    路径策略：
    1. 从起点向左水平移动到障碍物左侧安全位置
    2. 垂直移动到超过障碍物范围的Y坐标
    3. 向右水平移动到终点X坐标
    4. 垂直移动到终点
    
    这样可以确保完全不穿过障碍物
    """
    if not obstacles:
        return [start, end]
    
    # 收集所有障碍物的边界点
    all_boundary_points = []
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords:
            all_boundary_points.extend(coords)
    
    if not all_boundary_points:
        return [start, end]
    
    # 计算安全偏移量（保持与障碍物的安全距离）
    offset_lng, offset_lat = meters_to_deg(safety_radius * 1.5, start[1])
    
    # 获取障碍物最左侧的X坐标（经度最小值）
    leftmost_x = min(p[0] for p in all_boundary_points)
    # 航线X坐标 = 最左侧X - 安全偏移（更左边）
    boundary_x = leftmost_x - offset_lng
    
    # 获取障碍物的Y范围（纬度范围）
    min_y = min(p[1] for p in all_boundary_points)
    max_y = max(p[1] for p in all_boundary_points)
    
    # 扩展Y范围，确保完全绕过
    extended_min_y = min_y - offset_lat * 2
    extended_max_y = max_y + offset_lat * 2
    
    # 构建绕过路径
    full_path = []
    
    # 起点
    full_path.append(start.copy())
    
    # 步骤1: 从起点水平向左移动到左侧边界
    waypoint1 = [boundary_x, start[1]]
    full_path.append(waypoint1)
    
    # 步骤2: 沿着左侧边界垂直移动，绕过整个障碍物范围
    # 判断起点和终点的Y坐标关系，选择向上还是向下绕过
    if end[1] > start[1]:
        # 终点在下方，需要向下绕过
        # 先移动到障碍物底部下方
        waypoint2 = [boundary_x, extended_max_y]
        full_path.append(waypoint2)
    else:
        # 终点在上方，需要向上绕过
        waypoint2 = [boundary_x, extended_min_y]
        full_path.append(waypoint2)
    
    # 步骤3: 水平向右移动到终点的X坐标
    waypoint3 = [end[0], waypoint2[1]]
    full_path.append(waypoint3)
    
    # 步骤4: 垂直移动到终点
    waypoint4 = [end[0], end[1]]
    full_path.append(waypoint4)
    
    # 去重：移除相邻的重复点
    unique_path = []
    for point in full_path:
        if not unique_path:
            unique_path.append(point)
        else:
            last = unique_path[-1]
            # 如果新点与上一点不同（考虑浮点误差）
            if abs(last[0] - point[0]) > 1e-10 or abs(last[1] - point[1]) > 1e-10:
                unique_path.append(point)
    
    # 验证路径是否穿过障碍物，如果穿过则增加偏移量重试
    max_retries = 3
    retry_offset_multiplier = 2
    for retry in range(max_retries):
        collision_found = False
        for i in range(len(unique_path) - 1):
            if line_intersects_polygon(unique_path[i], unique_path[i+1], all_boundary_points):
                collision_found = True
                # 增加偏移量
                larger_offset = offset_lng * (retry_offset_multiplier * (retry + 1))
                new_boundary_x = leftmost_x - larger_offset
                logger.warning(f"路径穿过障碍物，增加偏移量到 {new_boundary_x}")
                # 更新路径中的X坐标
                for j in range(1, len(unique_path) - 1):
                    if j == 1 or j == 2:  # 更新左侧边界点
                        unique_path[j][0] = new_boundary_x
                break
        
        if not collision_found:
            break
    
    # 计算路径长度用于调试
    total_length = 0
    for i in range(len(unique_path) - 1):
        total_length += distance(unique_path[i], unique_path[i+1]) * 111000
    
    logger.info(f"向左绕行路径生成: {len(unique_path)}个点, 总长约{total_length:.1f}米")
    for i, p in enumerate(unique_path):
        logger.info(f"  点{i}: ({p[0]:.6f}, {p[1]:.6f})")
    
    return unique_path

def find_right_path(start: List[float], end: List[float], 
                   obstacles_gcj: List[Dict], flight_altitude: float, 
                   safety_radius: float = 5) -> List[List[float]]:
    """向右绕行 - 从右侧绕过障碍物"""
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    bounds = get_obstacle_bounds(blocking_obs)
    if not bounds:
        return [start, end]
    
    obstacle_boundary_points = []
    for obs in blocking_obs:
        coords = obs.get('polygon', [])
        if coords:
            obstacle_boundary_points.extend(coords)
    
    rightmost_x = max(p[0] for p in obstacle_boundary_points) if obstacle_boundary_points else bounds['max_lng']
    
    offset_lng, offset_lat = meters_to_deg(safety_radius * 2.5, start[1])
    right_x = rightmost_x + offset_lng
    
    candidates = []
    
    waypoint1 = [right_x, start[1]]
    if is_path_clear(start, waypoint1, blocking_obs) and is_path_clear(waypoint1, end, blocking_obs):
        candidates.append(waypoint1)
    
    waypoint2 = [right_x, (start[1] + end[1]) / 2 + offset_lat]
    if is_path_clear(start, waypoint2, blocking_obs) and is_path_clear(waypoint2, end, blocking_obs):
        candidates.append(waypoint2)
    
    waypoint3 = [right_x, (start[1] + end[1]) / 2 - offset_lat]
    if is_path_clear(start, waypoint3, blocking_obs) and is_path_clear(waypoint3, end, blocking_obs):
        candidates.append(waypoint3)
    
    if obstacle_boundary_points:
        boundary_y = (min(p[1] for p in obstacle_boundary_points) + max(p[1] for p in obstacle_boundary_points)) / 2
        waypoint4 = [right_x, boundary_y]
        if is_path_clear(start, waypoint4, blocking_obs) and is_path_clear(waypoint4, end, blocking_obs):
            candidates.append(waypoint4)
    
    if candidates:
        best = min(candidates, key=lambda wp: distance(start, wp) + distance(wp, end))
        return [start, best, end]
    
    larger_offset_lng, larger_offset_lat = meters_to_deg(safety_radius * 5, start[1])
    right_x = rightmost_x + larger_offset_lng
    waypoint = [right_x, (start[1] + end[1]) / 2]
    
    if not (is_path_clear(start, waypoint, blocking_obs) and is_path_clear(waypoint, end, blocking_obs)):
        logger.warning("向右绕行失败，返回直线路径")
        return [start, end]
    
    return [start, waypoint, end]

def create_avoidance_path(start: List[float], end: List[float], 
                         obstacles_gcj: List[Dict], flight_altitude: float, 
                         direction: str, safety_radius: float = 5) -> List[List[float]]:
    """创建避障路径"""
    high_obstacles = [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude]
    blocking_obs = get_blocking_obstacles(start, end, high_obstacles, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    if direction == "向左绕行":
        path = find_left_boundary_path(start, end, blocking_obs, safety_radius)
    elif direction == "向右绕行":
        path = find_right_path(start, end, high_obstacles, flight_altitude, safety_radius)
    else:  # 最佳航线
        left_path = find_left_boundary_path(start, end, blocking_obs, safety_radius)
        right_path = find_right_path(start, end, high_obstacles, flight_altitude, safety_radius)
        
        left_len = sum(distance(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
        right_len = sum(distance(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))
        
        path = left_path if left_len <= right_len else right_path
    
    # 验证路径安全性
    if not validate_path_does_not_cross_obstacles(path, high_obstacles, flight_altitude):
        logger.error("生成的路径仍然穿过障碍物！使用备用方案...")
        # 备用方案：使用更大的偏移量
        all_points = []
        for obs in blocking_obs:
            all_points.extend(obs.get('polygon', []))
        if all_points:
            leftmost_x = min(p[0] for p in all_points)
            larger_offset_lng, _ = meters_to_deg(safety_radius * 5, start[1])
            safe_x = leftmost_x - larger_offset_lng
            path = [start, [safe_x, start[1]], [safe_x, end[1]], end]
        else:
            path = [start, end]
    
    return path

# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)]
        if len(backup_files) > MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-MAX_BACKUP_FILES]:
                os.remove(os.path.join(BACKUP_DIR, old_file))
                logger.info(f"已删除旧备份: {old_file}")
    except Exception as e:
        logger.error(f"清理备份文件失败: {e}")

def backup_config():
    if os.path.exists(CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{BACKUP_DIR}/{CONFIG_FILE}.{timestamp}.bak"
        try:
            shutil.copy(CONFIG_FILE, backup_name)
            logger.info(f"配置文件已备份到: {backup_name}")
            cleanup_old_backups()
            return backup_name
        except Exception as e:
            logger.error(f"备份失败: {e}")
    return None

def load_obstacles() -> List[Dict]:
    if not os.path.exists(CONFIG_FILE):
        logger.warning(f"配置文件 {CONFIG_FILE} 不存在")
        return []
    
    try:
        with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
            obstacles = data.get('obstacles', [])
            
            if not obstacles:
                logger.info("配置文件中没有障碍物数据")
                return []
            
            valid_obstacles = []
            for i, obs in enumerate(obstacles):
                if 'height' not in obs:
                    obs['height'] = 30
                    logger.warning(f"障碍物 {i+1} 缺少高度字段，使用默认值30m")
                
                if 'name' not in obs:
                    obs['name'] = f'未命名障碍物{i+1}'
                    logger.warning(f"障碍物 {i+1} 缺少名称，使用默认名称")
                
                if 'polygon' not in obs or len(obs['polygon']) < 3:
                    logger.warning(f"障碍物 {obs.get('name')} 多边形无效（顶点数不足），已跳过")
                    continue
                
                if not validate_polygon(obs['polygon']):
                    logger.warning(f"障碍物 {obs.get('name')} 多边形自相交，已跳过")
                    continue
                
                valid_obstacles.append(obs)
            
            logger.info(f"成功加载 {len(valid_obstacles)} 个有效障碍物（共{len(obstacles)}个）")
            return valid_obstacles
            
    except json.JSONDecodeError as e:
        logger.error(f"JSON解析失败: {e}")
        st.error(f"配置文件格式错误: {e}")
        return []
    except Exception as e:
        logger.error(f"加载配置文件失败: {e}")
        st.error(f"加载失败: {e}")
        return []

def save_obstacles(obstacles: List[Dict]):
    try:
        backup_config()
        
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v27.5'
        }
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        logger.info(f"成功保存 {len(obstacles)} 个障碍物")
    except Exception as e:
        logger.error(f"保存配置文件失败: {e}")
        raise

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = DEFAULT_FLIGHT_ALTITUDE
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None
        
    def set_path(self, path: List[List[float]], altitude: float = 50, 
                 speed: float = 50, safety_radius: float = 5):
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
            self.total_distance += distance(path[i], path[i + 1])
        
        logger.info(f"飞行任务开始: 总距离={self.total_distance*111000:.1f}m, 高度={altitude}m")
    
    def update_and_generate(self, obstacles_gcj: List[Dict]) -> Optional[Dict]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
                logger.info("飞行任务完成")
            return None
        
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = 0.2
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        
        base_speed = 5
        speed_m_per_s = base_speed * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            self.progress = self.distance_traveled / self.total_distance
            self.progress = min(1.0, self.progress)
        
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                logger.info("到达目的地")
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = self.distance_traveled / segment_distance
                t = min(1.0, max(0.0, t))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        safe, min_dist, danger = check_safety_radius(
            self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius
        )
        if not safe:
            self.safety_violation = True
            logger.warning(f"安全半径违规！距离{danger}: {min_dist:.1f}米")
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> Dict:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        heartbeat = {
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'flight_time': flight_time,
            'lat': self.current_pos[1],
            'lng': self.current_pos[0],
            'altitude': self.flight_altitude,
            'voltage': round(22.2 + random.uniform(-0.5, 0.5), 1),
            'satellites': random.randint(8, 14),
            'speed': round(5 * (self.speed / 100), 1),
            'progress': self.progress if self.total_distance > 0 else 0,
            'arrived': arrived,
            'safety_violation': self.safety_violation,
            'remaining_distance': max(0, self.total_distance - self.distance_traveled) * 111000
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > MAX_HISTORY_SIZE:
            self.history.pop()
        
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > MAX_FLIGHT_LOG_SIZE:
            self.flight_log.pop(0)
        
        return heartbeat
    
    def export_flight_data(self) -> pd.DataFrame:
        return pd.DataFrame(self.flight_log)

# ==================== 创建地图 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, 
                       obstacles_gcj: List[Dict], flight_history: List = None, 
                       planned_path: List = None, map_type: str = "satellite", 
                       straight_blocked: bool = True, flight_altitude: float = 50, 
                       drone_pos: List = None, direction: str = "最佳航线", 
                       safety_radius: float = 5) -> folium.Map:
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=17, tiles=tiles, attr=attr)
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 
                                 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 
                      'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            if height > flight_altitude:
                color = "red"
                fill_opacity = 0.5
                popup = f"🚧 {obs.get('name')}\n⚠️ 高度: {height}m (需避让)"
            else:
                color = "orange"
                fill_opacity = 0.3
                popup = f"🏢 {obs.get('name')}\n高度: {height}m (安全)"
            
            folium.Polygon([[c[1], c[0]] for c in coords], 
                          color=color, weight=3, fill=True, 
                          fill_color=color, fill_opacity=fill_opacity, 
                          popup=popup).add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], 
                     popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], 
                     popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        
        if "向左" in direction:
            line_color = "purple"
            waypoint_count = len(planned_path) - 2
            line_label = f"向左绕行（沿边界，{waypoint_count}个绕行点）"
        elif "向右" in direction:
            line_color = "orange"
            waypoint_count = len(planned_path) - 2
            line_label = f"向右绕行（{waypoint_count}个绕行点）"
        else:
            line_color = "green"
            line_label = "最佳航线"
        
        folium.PolyLine(path_locations, color=line_color, weight=4, opacity=0.9, 
                       popup=f"✈️ {line_label}").add_to(m)
        
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=7, color=line_color, 
                              fill=True, fill_color="white", fill_opacity=0.9, 
                              popup=f"绕行点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], 
                           [points_gcj['B'][1], points_gcj['B'][0]]], 
                          color="blue", weight=2, opacity=0.5, dash_array='5, 5', 
                          popup="直线航线（畅通）").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], 
                           [points_gcj['B'][1], points_gcj['B'][0]]], 
                          color="gray", weight=2, opacity=0.4, dash_array='5, 5', 
                          popup="⚠️ 直线航线被阻挡").add_to(m)
    
    # 添加左侧边界辅助线（调试用）
    if direction == "向左绕行" and planned_path and obstacles_gcj:
        all_points = []
        for obs in obstacles_gcj:
            if obs.get('height', 30) > flight_altitude:
                all_points.extend(obs.get('polygon', []))
        if all_points:
            leftmost_x = min(p[0] for p in all_points)
            offset_lng, _ = meters_to_deg(safety_radius * 1.5, center_gcj[1])
            boundary_x = leftmost_x - offset_lng
            
            # 绘制左侧边界辅助线（调试用）
            folium.PolyLine(
                [[boundary_x, center_gcj[1] - 0.002], 
                 [boundary_x, center_gcj[1] + 0.002]],
                color="blue", weight=2, opacity=0.6, dash_array='5, 5',
                popup="左侧安全边界"
            ).add_to(m)
    
    if drone_pos:
        folium.Circle(
            radius=safety_radius,
            location=[drone_pos[1], drone_pos[0]],
            color="blue",
            weight=2,
            fill=True,
            fill_color="blue",
            fill_opacity=0.2,
            popup=f"🛡️ 安全半径: {safety_radius}米"
        ).add_to(m)
    
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.markdown("""
    <style>
    .stButton button { width: 100%; transition: all 0.3s ease; }
    .stButton button:hover { transform: translateY(-2px); box-shadow: 0 2px 5px rgba(0,0,0,0.2); }
    </style>
    """, unsafe_allow_html=True)
    
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    # 初始化session state
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()}
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = load_obstacles()
        for obs in st.session_state.obstacles_gcj:
            if 'height' not in obs:
                obs['height'] = 30
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    if "planned_path" not in st.session_state:
        st.session_state.planned_path = None
    if "last_flight_altitude" not in st.session_state:
        st.session_state.last_flight_altitude = DEFAULT_FLIGHT_ALTITUDE
    if "pending_obstacle" not in st.session_state:
        st.session_state.pending_obstacle = None
    if "current_direction" not in st.session_state:
        st.session_state.current_direction = "最佳航线"
    if "safety_radius" not in st.session_state:
        st.session_state.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
    if "auto_backup" not in st.session_state:
        st.session_state.auto_backup = True
    if "rerun_flag" not in st.session_state:
        st.session_state.rerun_flag = False
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=MIN_FLIGHT_ALTITUDE, 
                                   max_value=MAX_FLIGHT_ALTITUDE, value=DEFAULT_FLIGHT_ALTITUDE, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, 
                                     value=st.session_state.safety_radius, step=1)
    
    if safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = safety_radius
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude and not st.session_state.rerun_flag:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,
                flight_alt,
                st.session_state.current_direction,
                safety_radius
            )
            st.session_state.rerun_flag = True
            st.rerun()
    
    if st.session_state.rerun_flag:
        st.session_state.rerun_flag = False
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    
    straight_blocked = False
    high_obstacles = 0
    for obs in st.session_state.obstacles_gcj:
        if obs.get('height', 30) > flight_alt:
            high_obstacles += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(st.session_state.points_gcj['A'], 
                                                  st.session_state.points_gcj['B'], coords):
                straight_blocked = True
    
    st.sidebar.info(
        f"🏫 南京科技职业学院\n"
        f"🚧 障碍物: {obs_count} 个\n"
        f"📈 高于飞行高度: {high_obstacles} 个\n"
        f"📌 直线路径: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}\n"
        f"✈️ 飞行高度: {flight_alt} m\n"
        f"🛡️ 安全半径: {safety_radius} 米\n"
        f"✨ 绕行策略: 向右=1个绕行点 | 向左=沿边界绕行（绝不触碰）"
    )
    
    col_refresh1, col_refresh2 = st.sidebar.columns(2)
    with col_refresh1:
        if st.button("🔄 刷新数据", use_container_width=True):
            st.session_state.obstacles_gcj = load_obstacles()
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,
                flight_alt,
                st.session_state.current_direction,
                safety_radius
            )
            st.success("数据已刷新")
            st.rerun()
    
    with col_refresh2:
        if st.button("📊 导出日志", use_container_width=True):
            if st.session_state.heartbeat_sim.flight_log:
                df = st.session_state.heartbeat_sim.export_flight_data()
                csv = df.to_csv(index=False)
                st.download_button("下载飞行日志", csv, f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            else:
                st.warning("暂无飞行数据")
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，已自动规划避障路径")
        else:
            st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
        
        st.info("📝 **绕行说明**：向右绕行→1个绕行点（右侧绕过）| 向左绕行→沿障碍物左侧边界绕行（绝不触碰）")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            with st.expander("📍 起点/终点设置", expanded=True):
                st.markdown("#### 🟢 起点 A")
                col_a1, col_a2 = st.columns(2)
                with col_a1:
                    a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], 
                                           format="%.6f", key="a_lat", step=0.000001)
                with col_a2:
                    a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], 
                                           format="%.6f", key="a_lng", step=0.000001)
                
                if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                    try:
                        validate_coordinates(a_lng, a_lat)
                        st.session_state.points_gcj['A'] = [a_lng, a_lat]
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.success("起点已更新")
                        st.rerun()
                    except ValueError as e:
                        st.error(f"坐标无效: {e}")
                
                st.markdown("#### 🔴 终点 B")
                col_b1, col_b2 = st.columns(2)
                with col_b1:
                    b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], 
                                           format="%.6f", key="b_lat", step=0.000001)
                with col_b2:
                    b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], 
                                           format="%.6f", key="b_lng", step=0.000001)
                
                if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                    try:
                        validate_coordinates(b_lng, b_lat)
                        st.session_state.points_gcj['B'] = [b_lng, b_lat]
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.success("终点已更新")
                        st.rerun()
                    except ValueError as e:
                        st.error(f"坐标无效: {e}")
            
            with st.expander("🤖 路径规划策略", expanded=True):
                col_dir1, col_dir2, col_dir3 = st.columns(3)
                
                with col_dir1:
                    if st.button("🔄 最佳航线", use_container_width=True, 
                                type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
                        st.session_state.current_direction = "最佳航线"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            "最佳航线",
                            safety_radius
                        )
                        st.success("已切换到最佳航线模式")
                        st.rerun()
                
                with col_dir2:
                    if st.button("⬅️ 向左绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
                        st.session_state.current_direction = "向左绕行"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            "向左绕行",
                            safety_radius
                        )
                        st.success("已切换到向左绕行模式（沿边界绕行，绝不触碰）")
                        st.rerun()
                
                with col_dir3:
                    if st.button("➡️ 向右绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
                        st.session_state.current_direction = "向右绕行"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            "向右绕行",
                            safety_radius
                        )
                        st.success("已切换到向右绕行模式（1个绕行点）")
                        st.rerun()
                
                st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
                
                if st.button("🔄 重新规划路径", use_container_width=True):
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        st.session_state.current_direction,
                        safety_radius
                    )
                    if st.session_state.planned_path:
                        waypoint_count = len(st.session_state.planned_path) - 2
                        st.success(f"✅ 已规划避障路径，{waypoint_count}个绕行点")
                    st.rerun()
            
            with st.expander("✈️ 飞行控制", expanded=True):
                col_met1, col_met2, col_met3 = st.columns(3)
                with col_met1:
                    st.metric("当前飞行高度", f"{flight_alt} m")
                with col_met2:
                    st.metric("速度系数", f"{drone_speed}%")
                with col_met3:
                    st.metric("🛡️ 安全半径", f"{safety_radius} 米")
                
                if st.session_state.planned_path:
                    waypoint_count = len(st.session_state.planned_path) - 2
                    st.metric("🎯 绕行点数量", waypoint_count)
                    
                    total_dist = 0
                    for i in range(len(st.session_state.planned_path)-1):
                        total_dist += distance(st.session_state.planned_path[i], 
                                              st.session_state.planned_path[i+1]) * 111000
                    st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
                
                col_btn1, col_btn2 = st.columns(2)
                with col_btn1:
                    if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                        if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                            path = st.session_state.planned_path or [st.session_state.points_gcj['A'], 
                                                                    st.session_state.points_gcj['B']]
                            st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, safety_radius)
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
            
            st.markdown("### 📍 当前坐标")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
            st.caption(f"📏 直线距离: {dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            st.caption("🟣 向左绕行（沿边界绕行，绝不触碰）| 🟠 向右绕行（1个绕行点）| 🟢 最佳航线")
            st.caption("⚪ 白色圆点=绕行点 | 🔴 红色=需避让障碍物 | 🔵 蓝色虚线=左侧安全边界")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    st.session_state.current_direction,
                    safety_radius
                )
            
            drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
            
            m = create_planning_map(center, st.session_state.points_gcj, 
                                   st.session_state.obstacles_gcj, flight_trail, 
                                   st.session_state.planned_path, map_type, 
                                   straight_blocked, flight_alt, drone_pos, 
                                   st.session_state.current_direction, safety_radius)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
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
                            else:
                                st.error("绘制的多边形自相交，请重新绘制")
            
            if st.session_state.pending_obstacle is not None:
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
                
                col_name1, col_name2 = st.columns(2)
                with col_name1:
                    new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
                with col_name2:
                    new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, 
                                                value=30, step=5, key="height_input")
                
                col_ok, col_cancel = st.columns(2)
                with col_ok:
                    if st.button("✅ 确认添加", use_container_width=True, type="primary"):
                        st.session_state.obstacles_gcj.append({
                            "name": new_name,
                            "polygon": st.session_state.pending_obstacle,
                            "height": new_height
                        })
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.session_state.pending_obstacle = None
                        st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
                        st.rerun()
                with col_cancel:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_obstacle = None
                        st.rerun()
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        st.caption(f"✈️ 当前飞行高度: {flight_alt} 米 | 🧭 避障策略: {st.session_state.current_direction} | 🛡️ 安全半径: {safety_radius} 米")
        
        current_time = time.time()
        if st.session_state.simulation_running:
            if current_time - st.session_state.last_hb_time >= 0.2:
                try:
                    new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
                    if new_hb:
                        st.session_state.last_hb_time = current_time
                        st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                        if len(st.session_state.flight_history) > 200:
                            st.session_state.flight_history.pop(0)
                        if not st.session_state.heartbeat_sim.simulating:
                            st.session_state.simulation_running = False
                            st.success("🏁 无人机已安全到达目的地！")
                        st.rerun()
                except Exception as e:
                    st.error(f"更新心跳时出错: {e}")
                    logger.error(f"心跳更新错误: {e}")
        else:
            st.session_state.last_hb_time = current_time
        
        # 修复：确保 history 中的每个元素都是字典类型
        valid_history = []
        for hb in st.session_state.heartbeat_sim.history:
            if isinstance(hb, dict):
                valid_history.append(hb)
            else:
                logger.warning(f"心跳数据格式错误: {type(hb)}")
        
        st.session_state.heartbeat_sim.history = valid_history
        
        if valid_history:
            latest = valid_history[0]
            
            # 确保 latest 的所有字段都存在
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            col1.metric("⏰ 时间", latest.get('timestamp', 'N/A'))
            col2.metric("📍 纬度", f"{latest.get('lat', 0):.6f}")
            col3.metric("📍 经度", f"{latest.get('lng', 0):.6f}")
            col4.metric("📊 高度", f"{latest.get('altitude', 0)} m")
            col5.metric("🔋 电压", f"{latest.get('voltage', 0)} V")
            col6.metric("🛰️ 卫星", latest.get('satellites', 0))
            
            col7, col8, col9 = st.columns(3)
            col7.metric("💨 速度", f"{latest.get('speed', 0)} m/s")
            col8.metric("⚡ 速度系数", f"{drone_speed}%")
            col9.metric("📏 剩余距离", f"{latest.get('remaining_distance', 0):.0f} m")
            
            if latest.get('safety_violation', False):
                st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
            
            progress = latest.get('progress', 0)
            st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            if latest.get('arrived', False):
                st.success("🎉 无人机已到达目的地！飞行任务完成！")
                if st.session_state.auto_backup and st.session_state.heartbeat_sim.flight_log:
                    df = st.session_state.heartbeat_sim.export_flight_data()
                    csv = df.to_csv(index=False)
                    st.download_button("📥 下载本次飞行日志", csv, 
                                     f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest.get('lat', SCHOOL_CENTER_GCJ[1]), latest.get('lng', SCHOOL_CENTER_GCJ[0])], 
                                    zoom_start=17, tiles=tiles, attr="高德地图")
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords and len(coords) >= 3:
                    color = "red" if height > flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords], 
                                  color=color, weight=2, fill=True, fill_opacity=0.3, 
                                  popup=f"🚧 {obs.get('name')}").add_to(monitor_map)
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                if "向左" in st.session_state.current_direction:
                    line_color = "purple"
                elif "向右" in st.session_state.current_direction:
                    line_color = "orange"
                else:
                    line_color = "green"
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], 
                              color=line_color, weight=3, opacity=0.7).add_to(monitor_map)
            
            folium.Circle(
                radius=safety_radius,
                location=[latest.get('lat', SCHOOL_CENTER_GCJ[1]), latest.get('lng', SCHOOL_CENTER_GCJ[0])],
                color="blue",
                weight=2,
                fill=True,
                fill_color="blue",
                fill_opacity=0.2,
                popup=f"🛡️ 安全半径: {safety_radius}米"
            ).add_to(monitor_map)
            
            trail = []
            for hb in st.session_state.heartbeat_sim.history[:30]:
                if isinstance(hb, dict) and hb.get('lat') and hb.get('lng'):
                    trail.append([hb['lat'], hb['lng']])
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.7).add_to(monitor_map)
            
            folium.Marker([latest.get('lat', SCHOOL_CENTER_GCJ[1]), latest.get('lng', SCHOOL_CENTER_GCJ[0])], 
                         popup=f"当前位置\n高度: {latest.get('altitude', 0)}m\n速度: {latest.get('speed', 0)}m/s", 
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
            
            if st.session_state.points_gcj.get('A'):
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                            popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj.get('B'):
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                            popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            st.subheader("📈 实时数据图表")
            col_ch1, col_ch2 = st.columns(2)
            
            with col_ch1:
                if len(valid_history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": list(range(min(30, len(valid_history)))),
                        "高度(m)": [h.get("altitude", 0) for h in valid_history[:30]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
            
            with col_ch2:
                if len(valid_history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": list(range(min(30, len(valid_history)))),
                        "速度(m/s)": [h.get("speed", 0) for h in valid_history[:30]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
            
            st.subheader("📋 历史心跳记录")
            history_df = pd.DataFrame(valid_history[:10])
            display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites']
            display_cols = [col for col in display_cols if col in history_df.columns]
            st.dataframe(history_df[display_cols], use_container_width=True)
            
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                csv = df.to_csv(index=False)
                st.download_button("下载CSV文件", csv, 
                                 f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        col_info1, col_info2 = st.columns(2)
        with col_info1:
            if os.path.exists(CONFIG_FILE):
                file_size = os.path.getsize(CONFIG_FILE)
                mod_time = datetime.fromtimestamp(os.path.getmtime(CONFIG_FILE)).strftime("%Y-%m-%d %H:%M:%S")
                st.info(f"📁 配置文件: {CONFIG_FILE}\n📏 大小: {file_size} bytes\n🕐 修改时间: {mod_time}")
            else:
                st.warning(f"⚠️ 配置文件 {CONFIG_FILE} 不存在，请先保存数据")
        
        with col_info2:
            backup_count = len([f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)])
            st.info(f"💾 备份文件数量: {backup_count} 个\n📂 备份目录: {BACKUP_DIR}")
        
        st.subheader("💾 数据管理")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物 | 🛡️ 安全半径: {safety_radius} 米")
        
        col_save1, col_save2, col_save3, col_save4 = st.columns(4)
        with col_save1:
            if st.button("💾 保存到JSON", use_container_width=True, type="primary"):
                try:
                    save_obstacles(st.session_state.obstacles_gcj)
                    st.success(f"✅ 已保存到 {CONFIG_FILE}")
                    st.balloons()
                except Exception as e:
                    st.error(f"保存失败: {e}")
        
        with col_save2:
            if st.button("📂 加载JSON", use_container_width=True):
                try:
                    if not os.path.exists(CONFIG_FILE):
                        st.warning(f"配置文件 {CONFIG_FILE} 不存在，请先保存数据")
                    else:
                        loaded_obstacles = load_obstacles()
                        
                        if loaded_obstacles:
                            st.session_state.obstacles_gcj = loaded_obstacles
                            
                            st.session_state.planned_path = create_avoidance_path(
                                st.session_state.points_gcj['A'],
                                st.session_state.points_gcj['B'],
                                st.session_state.obstacles_gcj,
                                flight_alt,
                                st.session_state.current_direction,
                                safety_radius
                            )
                            
                            st.success(f"✅ 成功加载 {len(loaded_obstacles)} 个障碍物")
                            
                            with st.expander("查看加载的障碍物详情"):
                                for i, obs in enumerate(loaded_obstacles):
                                    st.write(f"{i+1}. {obs.get('name', '未知')} - 高度: {obs.get('height', 30)}m - 顶点数: {len(obs.get('polygon', []))}")
                            
                            st.rerun()
                        else:
                            st.warning("⚠️ 配置文件为空或格式错误，未加载任何障碍物")
                            
                except json.JSONDecodeError as e:
                    st.error(f"配置文件格式错误: {e}")
                except Exception as e:
                    st.error(f"加载失败: {e}")
                    logger.error(f"加载障碍物错误: {e}")
        
        with col_save3:
            if st.session_state.obstacles_gcj:
                config_data = {
                    'obstacles': st.session_state.obstacles_gcj, 
                    'count': len(st.session_state.obstacles_gcj), 
                    'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 
                    'version': 'v27.5'
                }
                st.download_button(
                    label="📥 下载配置", 
                    data=json.dumps(config_data, ensure_ascii=False, indent=2), 
                    file_name=CONFIG_FILE, 
                    mime="application/json", 
                    use_container_width=True
                )
            else:
                st.button("📥 下载配置", use_container_width=True, disabled=True, help="暂无障碍物可导出")
        
        with col_save4:
            if st.button("🔄 恢复上次备份", use_container_width=True):
                try:
                    backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)]
                    if backup_files:
                        backup_files.sort(key=lambda x: os.path.getctime(os.path.join(BACKUP_DIR, x)), reverse=True)
                        latest_backup = backup_files[0]
                        
                        with open(f"{BACKUP_DIR}/{latest_backup}", 'r', encoding='utf-8') as f:
                            data = json.load(f)
                            restored_obstacles = data.get('obstacles', [])
                            
                            if restored_obstacles:
                                st.session_state.obstacles_gcj = restored_obstacles
                                
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    st.session_state.current_direction,
                                    safety_radius
                                )
                                
                                st.success(f"✅ 已从备份恢复: {latest_backup} (共 {len(restored_obstacles)} 个障碍物)")
                                st.rerun()
                            else:
                                st.warning("备份文件中没有障碍物数据")
                    else:
                        st.info("没有找到备份文件")
                except Exception as e:
                    st.error(f"恢复备份失败: {e}")
        
        st.markdown("---")
        st.subheader("📝 障碍物列表")
        
        col_list, col_map = st.columns([1, 1.5])
        
        with col_list:
            if st.session_state.obstacles_gcj:
                for i, obs in enumerate(st.session_state.obstacles_gcj):
                    height = obs.get('height', 30)
                    color = "🔴" if height > flight_alt else "🟠"
                    with st.container():
                        st.markdown(f"**{color} {obs.get('name', f'障碍物{i+1}')}**")
                        col_h, col_edit, col_del = st.columns([1, 1.5, 0.8])
                        with col_h:
                            st.write(f"高度: {height}m")
                        with col_edit:
                            new_h = st.number_input("", value=height, min_value=1, max_value=200, 
                                                   step=5, key=f"edit_{i}", label_visibility="collapsed")
                            if new_h != height:
                                obs['height'] = new_h
                                if st.session_state.auto_backup:
                                    save_obstacles(st.session_state.obstacles_gcj)
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    st.session_state.current_direction,
                                    safety_radius
                                )
                                st.rerun()
                        with col_del:
                            if st.button("🗑️", key=f"del_{i}"):
                                st.session_state.obstacles_gcj.pop(i)
                                if st.session_state.auto_backup:
                                    save_obstacles(st.session_state.obstacles_gcj)
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    st.session_state.current_direction,
                                    safety_radius
                                )
                                st.rerun()
                        st.markdown("---")
            else:
                st.info("暂无任何障碍物，可以在地图上绘制添加")
            
            if st.button("🗑️ 清除所有障碍物", use_container_width=True):
                if st.session_state.auto_backup:
                    backup_config()
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    [],
                    flight_alt,
                    st.session_state.current_direction,
                    safety_radius
                )
                st.success("已清除所有障碍物")
                st.rerun()
        
        with col_map:
            st.subheader("🗺️ 障碍物分布图")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], 
                                zoom_start=16, tiles=tiles, attr="高德地图")
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                color = "red" if height > flight_alt else "orange"
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], 
                                  color=color, weight=3, fill=True, fill_opacity=0.5, 
                                  popup=f"{obs.get('name')}\n高度: {height}m").add_to(obs_map)
            
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], 
                         popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], 
                         popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
            
            folium_static(obs_map, width=700, height=500)
            
            st.caption("🔴 红色: 需要避让（高于飞行高度）| 🟠 橙色: 安全（低于飞行高度）")

if __name__ == "__main__":
    main()
