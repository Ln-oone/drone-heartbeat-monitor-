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
from typing import List, Dict, Tuple, Optional, Any
import pandas as pd
import numpy as np

# ==================== 日志配置 ====================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==================== 页面配置 ====================
st.set_page_config(page_title="南京科技职业学院 - 无人机地面站系统", layout="wide")

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

# ==================== 几何函数 ====================
def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

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

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            return True
    return False

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_polygon_bounds(polygon):
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

def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def validate_polygon(polygon):
    return len(polygon) >= 3

# ==================== 绕行算法 ====================
def get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude):
    """获取阻挡直线路径的障碍物"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking_obs.append(obs)
    return blocking_obs

def find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    """
    向左绕行：沿着障碍物左侧边界绕行
    路径：起点 -> 向左平移至障碍物左侧 -> 沿左侧垂直移动 -> 向右平移至终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    # 收集所有阻挡障碍物的边界点
    all_points = []
    for obs in blocking_obs:
        all_points.extend(obs.get('polygon', []))
    
    if not all_points:
        return [start, end]
    
    # 计算障碍物的边界
    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)
    
    # 安全偏移量（紧贴边界）
    offset_x, offset_y = meters_to_deg(safety_radius * 0.5, start[1])
    
    # 左侧边界X坐标
    left_x = min_x - offset_x * 2
    
    # 构建绕行路径
    path = [start.copy()]
    
    # 1. 从起点水平向左移动到左侧边界
    path.append([left_x, start[1]])
    
    # 2. 判断从上方还是下方绕过
    # 计算起点和终点相对于障碍物的位置
    start_above = start[1] > max_y
    start_below = start[1] < min_y
    end_above = end[1] > max_y
    end_below = end[1] < min_y
    
    if start_above and end_above:
        # 都在上方，直接从上方水平绕过
        path.append([left_x, max_y + offset_y])
        path.append([end[0], max_y + offset_y])
    elif start_below and end_below:
        # 都在下方，直接从下方水平绕过
        path.append([left_x, min_y - offset_y])
        path.append([end[0], min_y - offset_y])
    else:
        # 一个在上方一个在下方，需要绕过整个障碍物
        if start[1] > end[1]:
            # 起点在上方，终点在下方：先向上再向下
            path.append([left_x, max_y + offset_y])
            path.append([left_x, min_y - offset_y])
        else:
            # 起点在下方，终点在上方：先向下再向上
            path.append([left_x, min_y - offset_y])
            path.append([left_x, max_y + offset_y])
        path.append([end[0], path[-1][1]])
    
    # 4. 垂直移动到终点
    path.append(end.copy())
    
    # 去重
    unique_path = []
    for p in path:
        if not unique_path:
            unique_path.append(p)
        elif abs(unique_path[-1][0] - p[0]) > 1e-10 or abs(unique_path[-1][1] - p[1]) > 1e-10:
            unique_path.append(p)
    
    logger.info(f"向左绕行路径: {len(unique_path)}个航点")
    return unique_path

def find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    """
    向右绕行：沿着障碍物右侧边界绕行
    路径：起点 -> 向右平移至障碍物右侧 -> 沿右侧垂直移动 -> 向左平移至终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    # 收集所有阻挡障碍物的边界点
    all_points = []
    for obs in blocking_obs:
        all_points.extend(obs.get('polygon', []))
    
    if not all_points:
        return [start, end]
    
    # 计算障碍物的边界
    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)
    
    # 安全偏移量（紧贴边界）
    offset_x, offset_y = meters_to_deg(safety_radius * 0.5, start[1])
    
    # 右侧边界X坐标
    right_x = max_x + offset_x * 2
    
    # 构建绕行路径
    path = [start.copy()]
    
    # 1. 从起点水平向右移动到右侧边界
    path.append([right_x, start[1]])
    
    # 2. 判断从上方还是下方绕过
    start_above = start[1] > max_y
    start_below = start[1] < min_y
    end_above = end[1] > max_y
    end_below = end[1] < min_y
    
    if start_above and end_above:
        # 都在上方，直接从上方水平绕过
        path.append([right_x, max_y + offset_y])
        path.append([end[0], max_y + offset_y])
    elif start_below and end_below:
        # 都在下方，直接从下方水平绕过
        path.append([right_x, min_y - offset_y])
        path.append([end[0], min_y - offset_y])
    else:
        # 一个在上方一个在下方，需要绕过整个障碍物
        if start[1] > end[1]:
            # 起点在上方，终点在下方：先向上再向下
            path.append([right_x, max_y + offset_y])
            path.append([right_x, min_y - offset_y])
        else:
            # 起点在下方，终点在上方：先向下再向上
            path.append([right_x, min_y - offset_y])
            path.append([right_x, max_y + offset_y])
        path.append([end[0], path[-1][1]])
    
    # 4. 垂直移动到终点
    path.append(end.copy())
    
    # 去重
    unique_path = []
    for p in path:
        if not unique_path:
            unique_path.append(p)
        elif abs(unique_path[-1][0] - p[0]) > 1e-10 or abs(unique_path[-1][1] - p[1]) > 1e-10:
            unique_path.append(p)
    
    logger.info(f"向右绕行路径: {len(unique_path)}个航点")
    return unique_path

def find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    """最佳航线：选择向左和向右中较短的一条"""
    left_path = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right_path = find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    left_len = sum(distance(left_path[i], left_path[i+1]) for i in range(len(left_path)-1))
    right_len = sum(distance(right_path[i], right_path[i+1]) for i in range(len(right_path)-1))
    
    return left_path if left_len <= right_len else right_path

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction, safety_radius=5):
    """创建避障路径"""
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 障碍物管理 ====================
def backup_config():
    if os.path.exists(CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{BACKUP_DIR}/{CONFIG_FILE}.{timestamp}.bak"
        try:
            shutil.copy(CONFIG_FILE, backup_name)
            return backup_name
        except:
            pass
    return None

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles = data.get('obstacles', [])
                for obs in obstacles:
                    if 'selected' not in obs:
                        obs['selected'] = False
                return obstacles
        except:
            return []
    return []

def save_obstacles(obstacles):
    backup_config()
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v13.0'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = []
        self.last_update_time = None
        
    def set_path(self, path, altitude=50, speed=50, safety_radius=5):
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
    
    def update_and_generate(self, obstacles_gcj):
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
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
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = self.distance_traveled / segment_distance
                t = min(1.0, max(0.0, t))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived=False):
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
    
    def export_flight_data(self):
        return pd.DataFrame(self.flight_log)

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, 
                        map_type="satellite", straight_blocked=True, flight_altitude=50, 
                        drone_pos=None, direction="最佳航线", safety_radius=5):
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
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, 
                          fill=True, fill_color=color, fill_opacity=0.4,
                          popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    # 绘制起终点
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
        
        # 标记绕行点
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=6, color=line_color, 
                              fill=True, fill_color="white", fill_opacity=0.9, 
                              popup=f"绕行点 {i+1}").add_to(m)
    
    # 绘制直线航线
    if points_gcj.get('A') and points_gcj.get('B'):
        color = "gray" if straight_blocked else "blue"
        folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], 
                       [points_gcj['B'][1], points_gcj['B'][0]]], 
                      color=color, weight=2, opacity=0.5, dash_array='5, 5').add_to(m)
    
    # 绘制安全半径
    if drone_pos:
        folium.Circle(radius=safety_radius, location=[drone_pos[1], drone_pos[0]],
                     color="blue", weight=2, fill=True, fill_color="blue", fill_opacity=0.2).add_to(m)
    
    # 绘制历史轨迹
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6).add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    # 初始化session state
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()}
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = load_obstacles()
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
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", 10, 100, 50, 5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", MIN_FLIGHT_ALTITUDE, MAX_FLIGHT_ALTITUDE, 
                                   DEFAULT_FLIGHT_ALTITUDE, 5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", 1, 20, st.session_state.safety_radius, 1)
    st.session_state.safety_radius = safety_radius
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    st.session_state.auto_backup = st.sidebar.checkbox("自动保存障碍物", st.session_state.auto_backup)
    
    # 高度变化时重新规划
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        if st.session_state.planned_path is not None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
            st.rerun()
    
    # 统计信息
    obs_count = len(st.session_state.obstacles_gcj)
    high_obstacles = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
    straight_blocked = any(obs.get('height', 30) > flight_alt and 
                          line_intersects_polygon(st.session_state.points_gcj['A'], 
                                                  st.session_state.points_gcj['B'], 
                                                  obs.get('polygon', []))
                          for obs in st.session_state.obstacles_gcj)
    
    st.sidebar.info(f"障碍物: {obs_count}个 | 高于飞行高度: {high_obstacles}个\n"
                   f"直线路径: {'被阻挡' if straight_blocked else '畅通'}\n"
                   f"飞行高度: {flight_alt}m | 安全半径: {safety_radius}m")
    
    col_refresh1, col_refresh2 = st.sidebar.columns(2)
    with col_refresh1:
        if st.button("🔄 刷新数据", use_container_width=True):
            st.session_state.obstacles_gcj = load_obstacles()
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
            st.rerun()
    
    with col_refresh2:
        if st.button("📊 导出日志", use_container_width=True) and st.session_state.heartbeat_sim.flight_log:
            df = st.session_state.heartbeat_sim.export_flight_data()
            st.download_button("下载", df.to_csv(index=False), f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
        else:
            st.success("✅ 直线航线畅通无阻")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            with st.expander("📍 起点/终点设置", expanded=True):
                st.markdown("#### 🟢 起点 A")
                col_a1, col_a2 = st.columns(2)
                with col_a1:
                    a_lat = st.number_input("纬度", st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
                with col_a2:
                    a_lng = st.number_input("经度", st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
                if st.button("设置 A 点", use_container_width=True):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
                    st.rerun()
                
                st.markdown("#### 🔴 终点 B")
                col_b1, col_b2 = st.columns(2)
                with col_b1:
                    b_lat = st.number_input("纬度", st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
                with col_b2:
                    b_lng = st.number_input("经度", st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
                if st.button("设置 B 点", use_container_width=True):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
                    st.rerun()
            
            with st.expander("🤖 路径规划策略", expanded=True):
                col_dir1, col_dir2, col_dir3 = st.columns(3)
                with col_dir1:
                    if st.button("🔄 最佳航线", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
                        st.session_state.current_direction = "最佳航线"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj, flight_alt, "最佳航线", safety_radius)
                        st.rerun()
                with col_dir2:
                    if st.button("⬅️ 向左绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
                        st.session_state.current_direction = "向左绕行"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj, flight_alt, "向左绕行", safety_radius)
                        st.rerun()
                with col_dir3:
                    if st.button("➡️ 向右绕行", use_container_width=True,
                                type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
                        st.session_state.current_direction = "向右绕行"
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj, flight_alt, "向右绕行", safety_radius)
                        st.rerun()
                
                st.info(f"当前策略: {st.session_state.current_direction}")
                if st.button("重新规划", use_container_width=True):
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
                    st.success(f"已规划 {len(st.session_state.planned_path)-2} 个绕行点")
                    st.rerun()
            
            with st.expander("✈️ 飞行控制", expanded=True):
                col_met1, col_met2, col_met3 = st.columns(3)
                col_met1.metric("飞行高度", f"{flight_alt}m")
                col_met2.metric("速度系数", f"{drone_speed}%")
                col_met3.metric("安全半径", f"{safety_radius}m")
                
                col_btn1, col_btn2 = st.columns(2)
                with col_btn1:
                    if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                        path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                        st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, safety_radius)
                        st.session_state.simulation_running = True
                        st.session_state.flight_history = []
                        st.success("飞行已开始")
                        st.rerun()
                with col_btn2:
                    if st.button("⏹️ 停止飞行", use_container_width=True):
                        st.session_state.simulation_running = False
                        st.session_state.heartbeat_sim.simulating = False
                        st.info("飞行已停止")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
            
            drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            
            m = create_planning_map(SCHOOL_CENTER_GCJ, st.session_state.points_gcj, st.session_state.obstacles_gcj,
                                   flight_trail, st.session_state.planned_path, map_type, straight_blocked,
                                   flight_alt, drone_pos, st.session_state.current_direction, safety_radius)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
            # 处理新绘制的障碍物
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                            st.session_state.pending_obstacle = poly
                            st.rerun()
            
            if st.session_state.pending_obstacle:
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                col_name1, col_name2 = st.columns(2)
                with col_name1:
                    new_name = st.text_input("名称", f"建筑物{len(st.session_state.obstacles_gcj)+1}")
                with col_name2:
                    new_height = st.number_input("高度(m)", 1, 200, 30, 5)
                
                col_ok, col_cancel = st.columns(2)
                with col_ok:
                    if st.button("确认添加", use_container_width=True, type="primary"):
                        st.session_state.obstacles_gcj.append({
                            "name": new_name, "polygon": st.session_state.pending_obstacle,
                            "height": new_height, "selected": False
                        })
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj, flight_alt, st.session_state.current_direction, safety_radius)
                        st.session_state.pending_obstacle = None
                        st.rerun()
                with col_cancel:
                    if st.button("取消", use_container_width=True):
                        st.session_state.pending_obstacle = None
                        st.rerun()
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控")
        
        if st.session_state.simulation_running and time.time() - st.session_state.last_hb_time >= 0.2:
            new_hb = st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
            if new_hb:
                st.session_state.last_hb_time = time.time()
                st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                if not st.session_state.heartbeat_sim.simulating:
                    st.session_state.simulation_running = False
                    st.success("到达目的地")
                st.rerun()
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            cols = st.columns(6)
            cols[0].metric("时间", latest['timestamp'])
            cols[1].metric("纬度", f"{latest['lat']:.6f}")
            cols[2].metric("经度", f"{latest['lng']:.6f}")
            cols[3].metric("高度", f"{latest['altitude']}m")
            cols[4].metric("速度", f"{latest.get('speed',0)}m/s")
            cols[5].metric("进度", f"{latest.get('progress',0)*100:.1f}%")
            
            st.progress(latest.get('progress',0))
            st.subheader("实时地图")
            
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17,
                                    tiles=GAODE_SATELLITE_URL if map_type=="satellite" else GAODE_VECTOR_URL)
            folium.Marker([latest['lat'], latest['lng']], icon=folium.Icon(color='red', icon='plane')).add_to(monitor_map)
            folium_static(monitor_map, width=800, height=400)
        else:
            st.info("等待飞行开始...")
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        col_save1, col_save2, col_save3 = st.columns(3)
        with col_save1:
            if st.button("💾 保存到文件", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success("已保存")
        with col_save2:
            if st.button("📂 加载文件", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
                    st.rerun()
        with col_save3:
            if st.button("🗑️ 清除所有", use_container_width=True):
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                st.rerun()
        
        st.markdown("---")
        st.subheader("障碍物列表")
        
        # 批量操作
        col_batch1, col_batch2, col_batch3 = st.columns(3)
        with col_batch1:
            select_all = st.checkbox("全选")
        with col_batch2:
            batch_height = st.number_input("批量高度(m)", 1, 200, 30, 5, key="batch_h")
        with col_batch3:
            if st.button("批量设置高度", use_container_width=True):
                for obs in st.session_state.obstacles_gcj:
                    if obs.get('selected', False):
                        obs['height'] = batch_height
                save_obstacles(st.session_state.obstacles_gcj)
                st.rerun()
        
        # 显示列表
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            col_c, col_n, col_h, col_d = st.columns([0.3, 2, 1, 0.5])
            selected = col_c.checkbox("", key=f"sel_{i}", value=obs.get('selected', False))
            obs['selected'] = selected or select_all
            col_n.write(f"{'🔴' if obs['height']>flight_alt else '🟠'} {obs.get('name', f'障碍物{i+1}')}")
            new_h = col_h.number_input("高度", obs['height'], 1, 200, 5, key=f"h_{i}", label_visibility="collapsed")
            if new_h != obs['height']:
                obs['height'] = new_h
                save_obstacles(st.session_state.obstacles_gcj)
                st.rerun()
            if col_d.button("删除", key=f"del_{i}"):
                st.session_state.obstacles_gcj.pop(i)
                save_obstacles(st.session_state.obstacles_gcj)
                st.rerun()

if __name__ == "__main__":
    main()
