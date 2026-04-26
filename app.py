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

# ==================== JavaScript自动刷新 ====================
AUTO_REFRESH_JS = """
<script>
    // 自动刷新页面每0.5秒
    setTimeout(function() {
        window.location.reload();
    }, 500);
</script>
"""

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
        backup_config()
        
        data = {
            'obstacles': obstacles,
            'count': len(obstacles),
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'version': 'v27.9'
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
        self.speed_percent = 100
        self.progress = 0.0
        self.total_distance_meters = 0.0
        self.distance_traveled_meters = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None
        # 速度 20 m/s，600米约30秒完成（进度明显）
        self.base_speed_ms = 20.0
        
    def set_path(self, path: List[List[float]], altitude: float = 50, 
                 speed_percent: float = 100, safety_radius: float = 5):
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
        
        logger.info(f"飞行任务开始: 总距离={self.total_distance_meters:.1f}m, 速度={actual_speed:.1f}m/s, 预计时间={estimated_time:.0f}秒")
        
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
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        
        # 获取当前段距离（米）
        segment_distance_meters = distance_meters(start, end)
        
        # 计算移动距离（米）
        actual_speed = self.get_actual_speed()
        move_distance_meters = actual_speed * 0.3  # 0.3秒移动距离
        
        self.distance_traveled_meters += move_distance_meters
        
        # 更新进度
        if self.total_distance_meters > 0:
            self.progress = min(1.0, self.distance_traveled_meters / self.total_distance_meters)
        
        # 检查是否到达当前路径段终点
        if self.distance_traveled_meters >= segment_distance_meters and segment_distance_meters > 0:
            self.path_index += 1
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
                self.distance_traveled_meters = 0
                logger.info(f"到达航点 {self.path_index}/{len(self.path)-1}, 进度: {self.progress*100:.1f}%")
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
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
            'speed': round(actual_speed, 1),
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
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        
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
    # 自动刷新注入（每0.5秒刷新）
    auto_refresh_enabled = st.query_params.get("auto_refresh", "false") == "true"
    
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
    .auto-refresh-badge {
        background-color: #ff9800;
        color: white;
        padding: 5px 10px;
        border-radius: 5px;
        font-size: 14px;
        font-weight: bold;
        text-align: center;
        display: inline-block;
    }
    </style>
    """, unsafe_allow_html=True)
    
    # 自动刷新逻辑
    if auto_refresh_enabled and st.session_state.get("simulation_running", False):
        st.markdown('<div class="auto-refresh-badge">🔄 自动刷新中 (0.5秒/次)</div>', unsafe_allow_html=True)
        st.components.v1.html(AUTO_REFRESH_JS, height=0)
    
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
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    
    st.sidebar.caption("💡 基础速度: 20 m/s (72 km/h)")
    st.sidebar.caption("⏱️ 600米约30秒完成")
    
    drone_speed_percent = st.sidebar.slider(
        "速度系数", 
        min_value=30, 
        max_value=100, 
        value=100,
        step=10,
    )
    actual_speed = 20.0 * (drone_speed_percent / 100)
    st.sidebar.info(f"📊 实际速度: **{actual_speed:.1f} m/s** ({actual_speed*3.6:.0f} km/h)")
    
    default_distance = distance_meters(DEFAULT_A_GCJ, DEFAULT_B_GCJ)
    est_time = default_distance / actual_speed if actual_speed > 0 else 0
    st.sidebar.info(f"⏱️ 预计飞行时间: {est_time:.0f} 秒")
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🔄 自动刷新")
    
    # 自动刷新开关
    if st.sidebar.button("🔘 开启自动刷新", use_container_width=True, type="primary"):
        st.query_params["auto_refresh"] = "true"
        st.rerun()
    
    if st.sidebar.button("⏹️ 关闭自动刷新", use_container_width=True):
        st.query_params["auto_refresh"] = "false"
        st.rerun()
    
    if auto_refresh_enabled:
        st.sidebar.success("✅ 自动刷新已开启 (0.5秒/次)")
    else:
        st.sidebar.info("⏸️ 自动刷新已关闭")
    
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
    
    # 重新规划路径
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
    
    # ==================== 实时更新心跳 ====================
    current_time = time.time()
    if st.session_state.simulation_running:
        # 每0.3秒更新一次位置（不使用自动刷新时也更新）
        if current_time - st.session_state.get("last_update_time", 0) >= 0.3:
            try:
                new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
                if new_hb:
                    st.session_state.last_update_time = current_time
                    st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                    if len(st.session_state.flight_history) > 200:
                        st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
                    st.rerun()
            except Exception as e:
                logger.error(f"心跳更新错误: {e}")
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，已自动规划避障路径")
        else:
            st.success("✅ 直线航线畅通无阻")
        
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
                            st.warning("请先停止飞行")
                
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
                            st.success("已切换到向左绕行模式")
                            st.rerun()
                        else:
                            st.warning("请先停止飞行")
                
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
                            st.success("已切换到向右绕行模式")
                            st.rerun()
                        else:
                            st.warning("请先停止飞行")
                
                st.info(f"📌 当前策略: **{st.session_state.current_direction}**")
            
            with st.expander("✈️ 飞行控制", expanded=True):
                if st.session_state.simulation_running:
                    status = st.session_state.heartbeat_sim.get_flight_status()
                    if status['status'] == 'flying':
                        st.markdown('<div class="flight-status-flying">🚁 飞行中</div>', unsafe_allow_html=True)
                        st.progress(status['progress'], text=f"✈️ 进度: {status['progress_percent']:.1f}%")
                        st.write(f"📍 航点: {status['current_waypoint']}/{status['total_waypoints']}")
                        st.write(f"💨 速度: {status['speed']:.1f} m/s ({status['speed']*3.6:.0f} km/h)")
                        st.write(f"📏 已飞: {status['traveled_distance']:.0f}/{status['total_distance']:.0f}米")
                else:
                    st.markdown('<div class="flight-status-stopped">⏹️ 已停止</div>', unsafe_allow_html=True)
                
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
                                
                                total_dist_meters = 0
                                for i in range(len(path)-1):
                                    total_dist_meters += distance_meters(path[i], path[i+1])
                                est_time = total_dist_meters / actual_speed
                                
                                st.success(f"""
                                🚁 **无人机已起飞！**
                                - 速度: {actual_speed:.1f} m/s
                                - 距离: {total_dist_meters:.0f}米
                                - 预计时间: {est_time:.0f}秒
                                """)
                                st.rerun()
                            else:
                                st.error("❌ 请先设置起点和终点！")
                        else:
                            st.warning("⚠️ 正在飞行中")
                
                with col_btn2:
                    if st.button("⏹️ 停止飞行", use_container_width=True):
                        if st.session_state.simulation_running:
                            st.session_state.simulation_running = False
                            st.session_state.heartbeat_sim.simulating = False
                            st.success("🛑 飞行已停止")
                            st.rerun()
        
        with col2:
            st.subheader("🗺️ 规划地图")
            
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
            
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                            st.session_state.pending_obstacle = poly
                            st.rerun()
            
            if st.session_state.pending_obstacle is not None:
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                
                col_name1, col_name2 = st.columns(2)
                with col_name1:
                    new_name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
                with col_name2:
                    new_height = st.number_input("高度(米)", min_value=1, max_value=200, value=30, step=5)
                
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
                        st.success(f"✅ 已添加 {new_name}")
                        st.rerun()
                with col_cancel:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_obstacle = None
                        st.rerun()
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            col1, col2, col3, col4 = st.columns(4)
            col1.metric("📍 纬度", f"{latest['lat']:.6f}")
            col2.metric("📍 经度", f"{latest['lng']:.6f}")
            col3.metric("📊 高度", f"{latest['altitude']} m")
            col4.metric("💨 速度", f"{latest.get('speed', 0):.1f} m/s")
            
            col5, col6, col7, col8 = st.columns(4)
            col5.metric("⚡ 速度系数", f"{latest.get('speed_percent', 100)}%")
            col6.metric("📏 剩余距离", f"{latest.get('remaining_distance', 0):.0f} m")
            col7.metric("📍 航点", f"{latest.get('current_waypoint', 0)}/{latest.get('total_waypoints', 0)}")
            col8.metric("📏 已飞距离", f"{latest.get('traveled_distance', 0):.0f} m")
            
            if latest.get('safety_violation', False):
                st.error("⚠️ 安全半径违规！")
            
            progress = latest.get('progress', 0)
            st.progress(progress / 100, text=f"✈️ 飞行进度: {progress}%")
            
            if latest.get('arrived', False):
                st.success("🎉 到达目的地！")
                st.balloons()
            
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles)
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=2, fill_opacity=0.3).add_to(monitor_map)
            
            if st.session_state.planned_path:
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3).add_to(monitor_map)
            
            folium.Circle(radius=safety_radius, location=[latest['lat'], latest['lng']], color="blue", fill=True, fill_opacity=0.2).add_to(monitor_map)
            
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat')]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2).add_to(monitor_map)
            
            folium.Marker([latest['lat'], latest['lng']], popup=f"当前位置\n进度:{progress}%", icon=folium.Icon(color='red')).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="起点", icon=folium.Icon(color='green')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="终点", icon=folium.Icon(color='red')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            st.subheader("📋 历史心跳")
            history_df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            display_cols = ['timestamp', 'lat', 'lng', 'altitude', 'speed', 'progress', 'remaining_distance']
            display_cols = [col for col in display_cols if col in history_df.columns]
            st.dataframe(history_df[display_cols], use_container_width=True)
        
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面开始飞行")
    
    # ==================== 障碍物管理页面 ====================
    else:
        st.header("🚧 障碍物管理")
        
        st.subheader("💾 数据管理")
        col_save1, col_save2, col_save3 = st.columns(3)
        with col_save1:
            if st.button("💾 保存", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success("已保存")
        with col_save2:
            if st.button("📂 加载", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
                    st.rerun()
                else:
                    st.warning("无配置")
        with col_save3:
            config_data = {'obstacles': st.session_state.obstacles_gcj, 'count': len(st.session_state.obstacles_gcj)}
            st.download_button("📥 下载", data=json.dumps(config_data), file_name=CONFIG_FILE)
        
        st.markdown("---")
        st.subheader("📝 障碍物列表")
        
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            col_name, col_height, col_del = st.columns([2, 1, 0.5])
            col_name.write(f"🏢 {obs.get('name')}")
            col_height.write(f"{obs.get('height')}m")
            if col_del.button("🗑️", key=f"del_{i}"):
                st.session_state.obstacles_gcj.pop(i)
                save_obstacles(st.session_state.obstacles_gcj)
                st.rerun()
        
        if st.button("🗑️ 清除所有", use_container_width=True):
            st.session_state.obstacles_gcj = []
            save_obstacles([])
            st.rerun()
        
        st.subheader("🗺️ 障碍物分布")
        obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16)
        for obs in st.session_state.obstacles_gcj:
            coords = obs.get('polygon', [])
            if coords:
                folium.Polygon([[c[1], c[0]] for c in coords], color="red", fill=True).add_to(obs_map)
        folium_static(obs_map, width=700, height=400)

if __name__ == "__main__":
    main()
