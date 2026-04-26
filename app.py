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

# 高德地图瓦片地址
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# 创建备份目录
os.makedirs(BACKUP_DIR, exist_ok=True)

# ==================== 坐标验证 ====================
def validate_coordinates(lng: float, lat: float) -> bool:
    """验证坐标有效性"""
    if not (-180 <= lng <= 180):
        raise ValueError(f"经度超出有效范围: {lng}")
    if not (-90 <= lat <= 90):
        raise ValueError(f"纬度超出有效范围: {lat}")
    return True

# ==================== 几何函数 ====================
def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
    """射线法判断点是否在多边形内"""
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
    """检查点q是否在线段pr上"""
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

def orientation(p: List[float], q: List[float], r: List[float]) -> int:
    """计算三点方向"""
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-10:
        return 0
    return 1 if val > 0 else 2

def segments_intersect(p1: List[float], p2: List[float], 
                       p3: List[float], p4: List[float]) -> bool:
    """检查两线段是否相交"""
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
    """检查线段是否与多边形相交"""
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def distance_deg(p1: List[float], p2: List[float]) -> float:
    """计算两点间距离（度）"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def distance_meters(p1: List[float], p2: List[float]) -> float:
    """计算两点间距离（米）"""
    lat_mid = (p1[1] + p2[1]) / 2
    lng_deg = p2[0] - p1[0]
    lat_deg = p2[1] - p1[1]
    lng_m = lng_deg * 111000 * math.cos(math.radians(lat_mid))
    lat_m = lat_deg * 111000
    return math.sqrt(lng_m**2 + lat_m**2)

def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
    """获取多边形边界"""
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
    """计算点到线段距离（米）"""
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    
    if len_sq == 0:
        dist_deg = math.sqrt((px-x1)**2 + (py-y1)**2)
    else:
        t = ((px - x1) * dx + (py - y1) * dy) / len_sq
        t = max(0, min(1, t))
        
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        dist_deg = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    
    # 转换为米（考虑纬度影响）
    lat_mid = (point[1] + seg_start[1] + seg_end[1]) / 3
    return dist_deg * (111000 * math.cos(math.radians(lat_mid)))

def check_safety_radius(drone_pos: List[float], 
                       obstacles_gcj: List[Dict], 
                       flight_altitude: float, 
                       safety_radius: float) -> Tuple[bool, Optional[float], Optional[str]]:
    """检查无人机是否在安全半径内"""
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
    """将米转换为度"""
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

# ==================== 避障算法 ====================
def get_blocking_obstacles(start: List[float], end: List[float], 
                          obstacles_gcj: List[Dict], flight_altitude: float) -> List[Dict]:
    """获取阻挡航线的障碍物"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                if line_intersects_polygon(start, end, coords):
                    blocking_obs.append(obs)
    return blocking_obs

def get_obstacle_bounds(obstacles: List[Dict]) -> Optional[Dict]:
    """获取障碍物群体的边界"""
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
    """检查两点之间的路径是否与障碍物相交"""
    for obs in obstacles:
        coords = obs.get('polygon', [])
        if coords and line_intersects_polygon(p1, p2, coords):
            return False
    return True

def find_right_path(start: List[float], end: List[float], 
                   obstacles_gcj: List[Dict], flight_altitude: float, 
                   safety_radius: float = 5) -> List[List[float]]:
    """
    向右绕行 - 1个绕行点
    路径：起点 → 绕行点 → 终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    bounds = get_obstacle_bounds(blocking_obs)
    if not bounds:
        return [start, end]
    
    # 计算偏移距离
    offset_lng, offset_lat = meters_to_deg(safety_radius * 2.5, start[1])
    
    # 候选绕行点：障碍物右侧
    right_x = bounds['max_lng'] + offset_lng
    
    candidates = []
    
    # 候选1：从右侧平飞绕过
    waypoint1 = [right_x, start[1]]
    if is_path_clear(start, waypoint1, blocking_obs) and is_path_clear(waypoint1, end, blocking_obs):
        candidates.append(waypoint1)
    
    # 候选2：从右侧偏上绕过
    waypoint2 = [right_x, (start[1] + end[1]) / 2 + offset_lat]
    if is_path_clear(start, waypoint2, blocking_obs) and is_path_clear(waypoint2, end, blocking_obs):
        candidates.append(waypoint2)
    
    # 候选3：从右侧偏下绕过
    waypoint3 = [right_x, (start[1] + end[1]) / 2 - offset_lat]
    if is_path_clear(start, waypoint3, blocking_obs) and is_path_clear(waypoint3, end, blocking_obs):
        candidates.append(waypoint3)
    
    if candidates:
        # 选择距离最短的绕行点
        best = min(candidates, key=lambda wp: distance_deg(start, wp) + distance_deg(wp, end))
        return [start, best, end]
    
    # 默认：右侧偏上
    return [start, [right_x, (start[1] + end[1]) / 2], end]

def find_left_path(start: List[float], end: List[float], 
                  obstacles_gcj: List[Dict], flight_altitude: float, 
                  safety_radius: float = 5) -> List[List[float]]:
    """
    向左绕行 - 3个绕行点
    路径：起点 → 向上到障碍物顶部左侧 → 向右到障碍物右侧顶部 → 到终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    bounds = get_obstacle_bounds(blocking_obs)
    if not bounds:
        return [start, end]
    
    # 计算偏移距离
    offset_lng, offset_lat = meters_to_deg(safety_radius * 2, start[1])
    
    # 障碍物顶部Y坐标
    top_y = bounds['min_lat'] - offset_lat * 1.5
    
    # 障碍物左侧X坐标
    left_x = bounds['min_lng'] - offset_lng
    
    # 障碍物右侧X坐标
    right_x = bounds['max_lng'] + offset_lng
    
    # 三个绕行点
    waypoint1 = [left_x, top_y]                # 第1个绕行点：向上到障碍物顶部左侧附近
    waypoint2 = [right_x, top_y]               # 第2个绕行点：向右到障碍物右侧顶部
    waypoint3 = [end[0], end[1]]               # 第3个绕行点：到终点
    
    # 验证路径是否安全
    if (is_path_clear(start, waypoint1, blocking_obs) and
        is_path_clear(waypoint1, waypoint2, blocking_obs) and
        is_path_clear(waypoint2, waypoint3, blocking_obs)):
        return [start, waypoint1, waypoint2, waypoint3]
    
    # 如果被阻挡，尝试更大的偏移
    top_y = bounds['min_lat'] - offset_lat * 2.5
    left_x = bounds['min_lng'] - offset_lng * 1.5
    right_x = bounds['max_lng'] + offset_lng * 1.5
    waypoint1 = [left_x, top_y]
    waypoint2 = [right_x, top_y]
    
    return [start, waypoint1, waypoint2, end]

def find_best_path(start: List[float], end: List[float], 
                  obstacles_gcj: List[Dict], flight_altitude: float, 
                  safety_radius: float = 5) -> List[List[float]]:
    """最佳航线 - 自动选择较短路径"""
    right_path = find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    left_path = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    right_len = sum(distance_deg(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))
    left_len = sum(distance_deg(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
    
    return right_path if right_len <= left_len else left_path

def create_avoidance_path(start: List[float], end: List[float], 
                         obstacles_gcj: List[Dict], flight_altitude: float, 
                         direction: str, safety_radius: float = 5) -> List[List[float]]:
    """创建避障路径（主函数）"""
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 障碍物管理 ====================
def backup_config():
    """备份配置文件"""
    if os.path.exists(CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{BACKUP_DIR}/{CONFIG_FILE}.{timestamp}.bak"
        shutil.copy(CONFIG_FILE, backup_name)
        logger.info(f"配置文件已备份到: {backup_name}")
        return backup_name
    return None

def load_obstacles() -> List[Dict]:
    """加载障碍物配置"""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                # 验证每个障碍物的数据完整性
                for obs in obstacles:
                    if 'height' not in obs:
                        obs['height'] = 30
                    if 'name' not in obs:
                        obs['name'] = '未命名障碍物'
                    if 'polygon' not in obs or len(obs['polygon']) < 3:
                        logger.warning(f"障碍物 {obs.get('name')} 多边形无效")
                logger.info(f"成功加载 {len(obstacles)} 个障碍物")
                return obstacles
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return []
    return []

def save_obstacles(obstacles: List[Dict]):
    """保存障碍物配置"""
    try:
        # 自动备份
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
        self.speed_percent = 50
        self.progress = 0.0
        self.total_distance_meters = 0.0
        self.distance_traveled_meters = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None
        self.base_speed_ms = 5.0  # 恢复正常速度 5米/秒
        
    def set_path(self, path: List[List[float]], altitude: float = 50, 
                 speed_percent: float = 50, safety_radius: float = 5):
        """设置飞行路径"""
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed_percent = speed_percent
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled_meters = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = time.time()
        self.history = []
        
        # 计算总距离（米）
        self.total_distance_meters = 0.0
        for i in range(len(path) - 1):
            self.total_distance_meters += distance_meters(path[i], path[i + 1])
        
        actual_speed = self.get_actual_speed()
        estimated_time = self.total_distance_meters / actual_speed if actual_speed > 0 else 0
        
        logger.info(f"飞行任务开始: 总距离={self.total_distance_meters:.1f}m, 高度={altitude}m, 速度={actual_speed:.2f}m/s, 预计时间={estimated_time:.0f}秒")
        
        # 记录初始心跳
        initial_hb = self._generate_heartbeat(False)
        self.history.append(initial_hb)
    
    def get_actual_speed(self) -> float:
        """获取实际速度（米/秒）"""
        return self.base_speed_ms * (self.speed_percent / 100)
    
    def update_and_generate(self, obstacles_gcj: List[Dict]) -> Optional[Dict]:
        """更新并生成新的心跳包"""
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
                logger.info("飞行任务完成")
            return None
        
        # 控制更新频率（每秒更新5次，即0.2秒一次）
        current_time = time.time()
        if self.last_update_time and (current_time - self.last_update_time) < 0.2:
            return None
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        
        # 获取当前段距离（米）
        segment_distance_meters = distance_meters(start, end)
        
        # 计算移动距离（米）
        actual_speed = self.get_actual_speed()
        move_distance_meters = actual_speed * 0.2  # 0.2秒移动距离（米）
        
        self.distance_traveled_meters += move_distance_meters
        
        # 更新进度（基于米）
        if self.total_distance_meters > 0:
            self.progress = min(1.0, self.distance_traveled_meters / self.total_distance_meters)
        
        # 检查是否到达当前路径段终点
        if self.distance_traveled_meters >= segment_distance_meters and segment_distance_meters > 0:
            # 到达当前航点
            self.path_index += 1
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
                # 重置已行驶距离（用于当前段）
                self.distance_traveled_meters = 0
                logger.info(f"到达航点 {self.path_index}/{len(self.path)-1}, 进度: {self.progress*100:.1f}%")
            else:
                # 到达终点
                self.simulating = False
                logger.info("到达目的地")
                return self._generate_heartbeat(True)
        else:
            # 计算当前位置（线性插值）
            if segment_distance_meters > 0:
                t = self.distance_traveled_meters / segment_distance_meters
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        # 安全检查
        safe, min_dist, danger = check_safety_radius(
            self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius
        )
        if not safe:
            self.safety_violation = True
            logger.warning(f"安全半径违规！距离{danger}: {min_dist:.1f}米")
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> Dict:
        """生成心跳包数据"""
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        # 计算剩余距离（米）
        remaining_distance_meters = max(0, self.total_distance_meters - self.distance_traveled_meters)
        
        actual_speed = self.get_actual_speed()
        
        heartbeat = {
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'flight_time': flight_time,
            'lat': self.current_pos[1],
            'lng': self.current_pos[0],
            'altitude': self.flight_altitude,
            'voltage': round(22.2 + random.uniform(-0.5, 0.5), 1),
            'satellites': random.randint(8, 14),
            'speed': round(actual_speed, 2),
            'speed_percent': self.speed_percent,
            'progress': round(self.progress * 100, 1),
            'arrived': arrived,
            'safety_violation': self.safety_violation,
            'remaining_distance': round(remaining_distance_meters, 0),
            'current_waypoint': self.path_index,
            'total_waypoints': len(self.path) - 1,
            'traveled_distance': round(self.distance_traveled_meters, 0),
            'total_distance': round(self.total_distance_meters, 0)
        }
        
        # 保留最近100条记录
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        
        # 记录飞行日志
        self.flight_log.append(heartbeat)
        
        return heartbeat
    
    def get_flight_status(self) -> Dict:
        """获取当前飞行状态"""
        if not self.simulating:
            return {'status': 'stopped', 'message': '无人机已停止'}
        
        if self.path_index >= len(self.path) - 1:
            return {'status': 'completed', 'message': '飞行已完成'}
        
        actual_speed = self.get_actual_speed()
        remaining_time = 0
        if actual_speed > 0:
            remaining_distance_meters = max(0, self.total_distance_meters - self.distance_traveled_meters)
            remaining_time = remaining_distance_meters / actual_speed
        
        return {
            'status': 'flying',
            'progress': self.progress,
            'progress_percent': self.progress * 100,
            'current_waypoint': self.path_index,
            'total_waypoints': len(self.path) - 1,
            'speed': actual_speed,
            'speed_percent': self.speed_percent,
            'remaining_time': remaining_time,
            'remaining_distance': max(0, self.total_distance_meters - self.distance_traveled_meters),
            'total_distance': self.total_distance_meters,
            'traveled_distance': self.distance_traveled_meters
        }
    
    def export_flight_data(self) -> pd.DataFrame:
        """导出飞行数据为DataFrame"""
        return pd.DataFrame(self.flight_log)

# ==================== 创建地图 ====================
def create_planning_map(center_gcj: List[float], points_gcj: Dict, 
                       obstacles_gcj: List[Dict], flight_history: List = None, 
                       planned_path: List = None, map_type: str = "satellite", 
                       straight_blocked: bool = True, flight_altitude: float = 50, 
                       drone_pos: List = None, direction: str = "最佳航线", 
                       safety_radius: float = 5) -> folium.Map:
    """创建规划地图"""
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
    
    # 绘制障碍物
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
    
    # 绘制规划的路径
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        
        if "向左" in direction:
            line_color = "purple"
            waypoint_count = len(planned_path) - 2
            line_label = f"向左绕行（{waypoint_count}个绕行点）"
        elif "向右" in direction:
            line_color = "orange"
            waypoint_count = len(planned_path) - 2
            line_label = f"向右绕行（{waypoint_count}个绕行点）"
        else:
            line_color = "green"
            line_label = "最佳航线"
        
        folium.PolyLine(path_locations, color=line_color, weight=4, opacity=0.9, 
                       popup=f"✈️ {line_label}").add_to(m)
        
        # 标记绕行点
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=7, color=line_color, 
                              fill=True, fill_color="white", fill_opacity=0.9, 
                              popup=f"绕行点 {i+1}").add_to(m)
    
    # 绘制直线航线
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
    # 自定义CSS样式
    st.markdown("""
    <style>
    .stButton button {
        width: 100%;
        transition: all 0.3s ease;
    }
    .stButton button:hover {
        transform: translateY(-2px);
        box-shadow: 0 2px 5px rgba(0,0,0,0.2);
    }
    .flight-status-flying {
        background-color: #4CAF50;
        color: white;
        padding: 10px;
        border-radius: 5px;
        text-align: center;
        font-weight: bold;
    }
    .flight-status-stopped {
        background-color: #f44336;
        color: white;
        padding: 10px;
        border-radius: 5px;
        text-align: center;
        font-weight: bold;
    }
    .flight-status-completed {
        background-color: #2196F3;
        color: white;
        padding: 10px;
        border-radius: 5px;
        text-align: center;
        font-weight: bold;
    }
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
    if "flight_started" not in st.session_state:
        st.session_state.flight_started = False
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    
    # 速度说明
    st.sidebar.caption("💡 基础速度: 5 m/s（正常速度）")
    drone_speed_percent = st.sidebar.slider(
        "速度系数", 
        min_value=10, 
        max_value=100, 
        value=50, 
        step=10,
        help="速度系数越高飞行越快"
    )
    actual_speed = 5.0 * (drone_speed_percent / 100)
    st.sidebar.info(f"📊 实际速度: **{actual_speed:.1f} m/s**")
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=MIN_FLIGHT_ALTITUDE, 
                                   max_value=MAX_FLIGHT_ALTITUDE, value=DEFAULT_FLIGHT_ALTITUDE, step=5,
                                   help="高于障碍物时可直接飞越")
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, 
                                     value=st.session_state.safety_radius, step=1,
                                     help="无人机与障碍物的最小安全距离")
    
    if safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = safety_radius
    
    # 自动保存和备份设置
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    st.session_state.auto_backup = auto_save
    
    # 重新规划路径（当飞行高度改变时）
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None and not st.session_state.simulation_running:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],
                st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,
                flight_alt,
                st.session_state.current_direction,
                safety_radius
            )
            st.rerun()
    
    # 统计信息
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
        f"✨ 绕行点: 向右=1个 | 向左=3个\n"
        f"⚡ 飞行速度: {actual_speed:.1f} m/s"
    )
    
    # 刷新按钮
    col_refresh1, col_refresh2 = st.sidebar.columns(2)
    with col_refresh1:
        if st.button("🔄 刷新数据", use_container_width=True):
            st.session_state.obstacles_gcj = load_obstacles()
            if not st.session_state.simulation_running:
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
        
        st.info("📝 **绕行说明**：向右绕行→1个绕行点（右侧绕过）| 向左绕行→3个绕行点（从顶部绕过）")
        
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
                        if not st.session_state.simulation_running:
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
                        if not st.session_state.simulation_running:
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
                        if not st.session_state.simulation_running:
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
                        else:
                            st.warning("请先停止飞行再切换策略")
                
                with col_dir2:
                    if st.button("⬅️ 向左绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
                        if not st.session_state.simulation_running:
                            st.session_state.current_direction = "向左绕行"
                            st.session_state.planned_path = create_avoidance_path(
                                st.session_state.points_gcj['A'],
                                st.session_state.points_gcj['B'],
                                st.session_state.obstacles_gcj,
                                flight_alt,
                                "向左绕行",
                                safety_radius
                            )
                            st.success("已切换到向左绕行模式（3个绕行点）")
                            st.rerun()
                        else:
                            st.warning("请先停止飞行再切换策略")
                
                with col_dir3:
                    if st.button("➡️ 向右绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
                        if not st.session_state.simulation_running:
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
                        else:
                            st.warning("请先停止飞行再切换策略")
                
                st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
                
                if st.button("🔄 重新规划路径", use_container_width=True):
                    if not st.session_state.simulation_running:
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
                    else:
                        st.warning("请先停止飞行再重新规划")
            
            with st.expander("✈️ 飞行控制", expanded=True):
                # 显示飞行状态
                if st.session_state.simulation_running:
                    status = st.session_state.heartbeat_sim.get_flight_status()
                    if status['status'] == 'flying':
                        st.markdown('<div class="flight-status-flying">🚁 飞行状态: 飞行中</div>', unsafe_allow_html=True)
                        st.progress(status['progress'], text=f"✈️ 飞行进度: {status['progress_percent']:.1f}%")
                        st.write(f"📍 当前航点: {status['current_waypoint']}/{status['total_waypoints']}")
                        st.write(f"💨 当前速度: {status['speed']:.1f} m/s")
                        st.write(f"📏 已飞距离: {status['traveled_distance']:.0f} / {status['total_distance']:.0f} 米")
                        if status['remaining_time'] > 0:
                            st.write(f"⏱️ 预计剩余时间: {status['remaining_time']:.0f} 秒")
                else:
                    st.markdown('<div class="flight-status-stopped">⏹️ 飞行状态: 已停止</div>', unsafe_allow_html=True)
                
                st.markdown("---")
                
                # 显示参数
                col_met1, col_met2, col_met3 = st.columns(3)
                with col_met1:
                    st.metric("当前飞行高度", f"{flight_alt} m")
                with col_met2:
                    st.metric("速度系数", f"{drone_speed_percent}%")
                with col_met3:
                    st.metric("🛡️ 安全半径", f"{safety_radius} 米")
                    st.caption(f"实际速度: {actual_speed:.1f} m/s")
                
                if st.session_state.planned_path:
                    waypoint_count = len(st.session_state.planned_path) - 2
                    st.metric("🎯 绕行点数量", waypoint_count)
                    
                    # 计算路径信息（米）
                    total_dist_meters = 0
                    for i in range(len(st.session_state.planned_path)-1):
                        total_dist_meters += distance_meters(st.session_state.planned_path[i], 
                                                            st.session_state.planned_path[i+1])
                    estimated_time = total_dist_meters / actual_speed if actual_speed > 0 else 0
                    st.caption(f"📏 规划路径总长: {total_dist_meters:.0f} 米")
                    st.caption(f"⏱️ 预计飞行时间: {estimated_time:.0f} 秒 ({estimated_time/60:.1f} 分钟)")
                
                st.markdown("---")
                
                col_btn1, col_btn2 = st.columns(2)
                with col_btn1:
                    if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                        if not st.session_state.simulation_running:
                            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                                path = st.session_state.planned_path or [st.session_state.points_gcj['A'], 
                                                                        st.session_state.points_gcj['B']]
                                st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed_percent, safety_radius)
                                st.session_state.simulation_running = True
                                st.session_state.flight_history = []
                                st.session_state.flight_started = True
                                waypoint_count = len(path) - 2
                                
                                # 计算总距离和预计时间
                                total_dist_meters = 0
                                for i in range(len(path)-1):
                                    total_dist_meters += distance_meters(path[i], path[i+1])
                                est_time = total_dist_meters / actual_speed
                                
                                # 显示起飞确认信息
                                st.success(f"""
                                🚁 **无人机已起飞！**
                                - 起点: ({path[0][0]:.6f}, {path[0][1]:.6f})
                                - 终点: ({path[-1][0]:.6f}, {path[-1][1]:.6f})
                                - 路径类型: {'直线飞行' if waypoint_count == 0 else f'包含{waypoint_count}个绕行点'}
                                - 飞行高度: {flight_alt}米
                                - 安全半径: {safety_radius}米
                                - 飞行速度: {actual_speed:.1f} m/s
                                - 总距离: {total_dist_meters:.0f}米
                                - 预计时间: {est_time:.0f}秒 ({est_time/60:.1f}分钟)
                                """)
                                st.rerun()
                            else:
                                st.error("❌ 请先设置起点和终点！")
                        else:
                            st.warning("⚠️ 无人机正在飞行中，请先停止再开始新任务")
                
                with col_btn2:
                    if st.button("⏹️ 停止飞行", use_container_width=True):
                        if st.session_state.simulation_running:
                            st.session_state.simulation_running = False
                            st.session_state.heartbeat_sim.simulating = False
                            st.success("🛑 飞行已停止")
                            st.rerun()
                        else:
                            st.info("无人机未在飞行中")
            
            # 坐标信息
            st.markdown("### 📍 当前坐标")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist_meters = distance_meters(a, b)
            st.caption(f"📏 直线距离: {dist_meters:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            st.caption("🟣 向左绕行（3个绕行点）| 🟠 向右绕行（1个绕行点）| 🟢 最佳航线")
            st.caption("⚪ 白色圆点=绕行点 | 🔴 红色=需避让障碍物")
            st.caption(f"⚡ 当前速度设置: {actual_speed:.1f} m/s")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20] 
                          if 'lng' in hb and 'lat' in hb]
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
            
            drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.simulation_running else None
            
            m = create_planning_map(center, st.session_state.points_gcj, 
                                   st.session_state.obstacles_gcj, flight_trail, 
                                   st.session_state.planned_path, map_type, 
                                   straight_blocked, flight_alt, drone_pos, 
                                   st.session_state.current_direction, safety_radius)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
            # 处理新绘制的多边形
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                            st.session_state.pending_obstacle = poly
                            st.rerun()
            
            # 添加新障碍物对话框
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
                        save_obstacles(st.session_state.obstacles_gcj)
                        if not st.session_state.simulation_running:
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
        st.caption(f"⚡ 飞行速度: {actual_speed:.1f} m/s | ⏱️ 更新频率: 5次/秒")
        
        # 实时更新心跳包
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
                            st.balloons()
                        st.rerun()
                except Exception as e:
                    st.error(f"更新心跳时出错: {e}")
                    logger.error(f"心跳更新错误: {e}")
        else:
            st.session_state.last_hb_time = current_time
        
        # 显示最新心跳数据
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            # 主要数据显示
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            col1.metric("⏰ 时间", latest['timestamp'])
            col2.metric("📍 纬度", f"{latest['lat']:.6f}")
            col3.metric("📍 经度", f"{latest['lng']:.6f}")
            col4.metric("📊 高度", f"{latest['altitude']} m")
            col5.metric("🔋 电压", f"{latest['voltage']} V")
            col6.metric("🛰️ 卫星", latest['satellites'])
            
            col7, col8, col9 = st.columns(3)
            col7.metric("💨 速度", f"{latest.get('speed', 0):.1f} m/s")
            col8.metric("⚡ 速度系数", f"{latest.get('speed_percent', 50)}%")
            col9.metric("📏 剩余距离", f"{latest.get('remaining_distance', 0):.0f} m")
            
            # 航点信息
            if 'current_waypoint' in latest and 'total_waypoints' in latest:
                st.info(f"📍 航点进度: {latest['current_waypoint']}/{latest['total_waypoints']}")
            
            # 安全警告
            if latest.get('safety_violation', False):
                st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
            
            # 飞行进度
            progress = latest.get('progress', 0)
            st.progress(progress / 100, text=f"✈️ 飞行进度: {progress}%")
            
            if latest.get('arrived', False):
                st.success("🎉 无人机已到达目的地！飞行任务完成！")
                # 如果自动保存开启，保存飞行日志
                if st.session_state.auto_backup and st.session_state.heartbeat_sim.flight_log:
                    df = st.session_state.heartbeat_sim.export_flight_data()
                    csv = df.to_csv(index=False)
                    st.download_button("📥 下载本次飞行日志", csv, 
                                     f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            
            # 实时地图
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
            
            # 绘制障碍物
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords and len(coords) >= 3:
                    color = "red" if height > flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords], 
                                  color=color, weight=2, fill=True, fill_opacity=0.3, 
                                  popup=f"🚧 {obs.get('name')}").add_to(monitor_map)
            
            # 绘制规划路径
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                if "向左" in st.session_state.current_direction:
                    line_color = "purple"
                elif "向右" in st.session_state.current_direction:
                    line_color = "orange"
                else:
                    line_color = "green"
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], 
                              color=line_color, weight=3, opacity=0.7).add_to(monitor_map)
            
            # 安全半径圆
            folium.Circle(
                radius=safety_radius,
                location=[latest['lat'], latest['lng']],
                color="blue",
                weight=2,
                fill=True,
                fill_color="blue",
                fill_opacity=0.2,
                popup=f"🛡️ 安全半径: {safety_radius}米"
            ).add_to(monitor_map)
            
            # 历史轨迹
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] 
                    if hb.get('lat') and hb.get('lng')]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.7).add_to(monitor_map)
            
            # 当前位置标记
            folium.Marker([latest['lat'], latest['lng']], 
                         popup=f"当前位置\n高度: {latest['altitude']}m\n速度: {latest['speed']:.1f}m/s", 
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
            
            # 起点终点标记
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                            popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                            popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            # 数据显示
            st.subheader("📈 实时数据图表")
            col_ch1, col_ch2 = st.columns(2)
            
            with col_ch1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": list(range(min(30, len(st.session_state.heartbeat_sim.history)))),
                        "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:30]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
            
            with col_ch2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": list(range(min(30, len(st.session_state.heartbeat_sim.history)))),
                        "速度(m/s)": [h.get("speed", 0) for h in st.session_state.heartbeat_sim.history[:30]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
            
            # 进度数据图表
            if len(st.session_state.heartbeat_sim.history) > 1:
                progress_df = pd.DataFrame({
                    "序号": list(range(min(30, len(st.session_state.heartbeat_sim.history)))),
                    "进度(%)": [h.get("progress", 0) for h in st.session_state.heartbeat_sim.history[:30]]
                })
                st.line_chart(progress_df, x="序号", y="进度(%)")
            
            # 历史记录表格
            st.subheader("📋 历史心跳记录")
            history_df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            # 选择要显示的列
            display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'progress', 'remaining_distance', 'current_waypoint', 'total_waypoints', 'traveled_distance', 'total_distance']
            display_cols = [col for col in display_cols if col in history_df.columns]
            st.dataframe(history_df[display_cols], use_container_width=True)
            
            # 导出按钮
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                csv = df.to_csv(index=False)
                st.download_button("下载CSV文件", csv, 
                                 f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
            
            # 显示提示信息
            st.markdown("""
            ### 🚁 如何开始飞行？
            
            1. 在左侧边栏设置飞行参数（高度、速度等）
            2. 切换到「航线规划」页面
            3. 确认起点(A点)和终点(B点)已设置
            4. 选择绕行策略（最佳航线/向左绕行/向右绕行）
            5. 点击「开始飞行」按钮
            
            💡 **速度设置**: 
            - 基础速度: 5 m/s（正常速度）
            - 可通过速度系数调节（10%-100%）
            
            📊 **飞行特点**:
            - 进度基于实际飞行距离计算
            - 每0.2秒更新一次位置
            - 实时显示剩余距离和预计时间
            """)
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        st.subheader("💾 数据管理")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物 | 🛡️ 安全半径: {safety_radius} 米")
        
        col_save1, col_save2, col_save3, col_save4 = st.columns(4)
        with col_save1:
            if st.button("💾 保存到JSON", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success(f"已保存到 {CONFIG_FILE}")
        with col_save2:
            if st.button("📂 加载JSON", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
                    if not st.session_state.simulation_running:
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                    st.success(f"已加载 {len(loaded)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("无配置文件或文件为空")
        with col_save3:
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
        with col_save4:
            if st.button("🔄 恢复上次备份", use_container_width=True):
                # 查找最新的备份文件
                backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)]
                if backup_files:
                    latest_backup = max(backup_files)
                    with open(f"{BACKUP_DIR}/{latest_backup}", 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        st.session_state.obstacles_gcj = data.get('obstacles', [])
                        if not st.session_state.simulation_running:
                            st.session_state.planned_path = create_avoidance_path(
                                st.session_state.points_gcj['A'],
                                st.session_state.points_gcj['B'],
                                st.session_state.obstacles_gcj,
                                flight_alt,
                                st.session_state.current_direction,
                                safety_radius
                            )
                        st.success(f"已从备份恢复: {latest_backup}")
                        st.rerun()
                else:
                    st.warning("没有找到备份文件")
        
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
                                if not st.session_state.simulation_running:
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
                                if not st.session_state.simulation_running:
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
                if not st.session_state.simulation_running:
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
