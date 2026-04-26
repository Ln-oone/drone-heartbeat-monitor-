import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="南京科技职业学院 - 无人机地面站系统", layout="wide")

# ==================== 坐标 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

CONFIG_FILE = "obstacle_config.json"
BACKUP_DIR = "backups"
DEFAULT_SAFETY_RADIUS_METERS = 5
MAX_BACKUP_FILES = 10

# 高德地图瓦片地址
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

def validate_polygon(polygon):
    if len(polygon) < 3:
        return False
    # 简单检查：多边形顶点数足够
    return True

# ==================== 绕行算法 ====================
def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    """向左绕行：从障碍物左侧绕过"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking_obs.append(obs)
    
    if not blocking_obs:
        return [start, end]
    
    # 获取所有阻挡障碍物的边界
    min_lng_all = float('inf')
    max_lng_all = -float('inf')
    min_lat_all = float('inf')
    max_lat_all = -float('inf')
    
    for obs in blocking_obs:
        coords = obs.get('polygon', [])
        if coords:
            bounds = get_polygon_bounds(coords)
            if bounds:
                min_lng_all = min(min_lng_all, bounds['min_lng'])
                max_lng_all = max(max_lng_all, bounds['max_lng'])
                min_lat_all = min(min_lat_all, bounds['min_lat'])
                max_lat_all = max(max_lat_all, bounds['max_lat'])
    
    if min_lng_all == float('inf'):
        return [start, end]
    
    offset_lng, offset_lat = meters_to_deg(safety_radius * 3)
    
    # 向左绕行：经度取最左侧向左偏移
    left_x = min_lng_all - offset_lng * 3
    
    # 判断从上方还是下方绕过
    avg_lat = (start[1] + end[1]) / 2
    center_lat = (min_lat_all + max_lat_all) / 2
    
    if avg_lat < center_lat:
        # 路径偏下，从上方绕过
        waypoint = [left_x, max_lat_all + offset_lat]
    else:
        # 路径偏上，从下方绕过
        waypoint = [left_x, min_lat_all - offset_lat]
    
    return [start, waypoint, end]

def find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    """向右绕行：从障碍物右侧绕过"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking_obs.append(obs)
    
    if not blocking_obs:
        return [start, end]
    
    # 计算中点
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.sqrt(dx*dx + dy*dy)
    
    if length == 0:
        return [start, end]
    
    # 垂直向量（向右）
    perp_x = dy / length
    perp_y = -dx / length
    
    offset_dist = safety_radius * 10
    lat_rad = math.radians(mid_y)
    lng_scale = 111000 * math.cos(lat_rad)
    lat_scale = 111000
    
    offset_x = perp_x * offset_dist / lng_scale
    offset_y = perp_y * offset_dist / lat_scale
    
    waypoint = [mid_x + offset_x, mid_y + offset_y]
    
    return [start, waypoint, end]

def find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius=5):
    left_path = find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    right_path = find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    
    left_len = 0
    for i in range(len(left_path) - 1):
        left_len += distance(left_path[i], left_path[i + 1])
    
    right_len = 0
    for i in range(len(right_path) - 1):
        right_len += distance(right_path[i], right_path[i + 1])
    
    return left_path if left_len < right_len else right_path

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction, safety_radius=5):
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude, safety_radius)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude, safety_radius)

# ==================== 障碍物管理 ====================
def cleanup_old_backups():
    try:
        backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)]
        if len(backup_files) > MAX_BACKUP_FILES:
            backup_files.sort()
            for old_file in backup_files[:-MAX_BACKUP_FILES]:
                os.remove(os.path.join(BACKUP_DIR, old_file))
    except Exception as e:
        pass

def backup_config():
    if os.path.exists(CONFIG_FILE):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_name = f"{BACKUP_DIR}/{CONFIG_FILE}.{timestamp}.bak"
        try:
            import shutil
            shutil.copy(CONFIG_FILE, backup_name)
            cleanup_old_backups()
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
                # 为每个障碍物添加selected字段（用于批量操作）
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
        
        safe, min_dist, danger = self._check_safety(obstacles_gcj)
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _check_safety(self, obstacles_gcj):
        if not self.simulating:
            return True, None, None
        
        min_distance = float('inf')
        danger_name = None
        
        for obs in obstacles_gcj:
            coords = obs.get('polygon', [])
            obs_height = obs.get('height', 30)
            
            if obs_height <= self.flight_altitude:
                continue
            
            if coords and len(coords) >= 3:
                for i in range(len(coords)):
                    p1 = coords[i]
                    p2 = coords[(i + 1) % len(coords)]
                    dist = point_to_segment_distance_deg(self.current_pos, p1, p2) * 111000
                    if dist < min_distance:
                        min_distance = dist
                        danger_name = obs.get('name', '障碍物')
        
        if min_distance < self.safety_radius:
            return False, min_distance, danger_name
        return True, min_distance if min_distance != float('inf') else None, None
    
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
        if len(self.history) > 100:
            self.history.pop()
        
        self.flight_log.append(heartbeat)
        if len(self.flight_log) > 1000:
            self.flight_log.pop(0)
        
        return heartbeat
    
    def export_flight_data(self):
        return pd.DataFrame(self.flight_log)

def point_to_segment_distance_deg(point, seg_start, seg_end):
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    
    if len_sq == 0:
        return math.sqrt((px-x1)**2 + (py-y1)**2)
    
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

def point_to_segment_distance_meters(point, seg_start, seg_end):
    return point_to_segment_distance_deg(point, seg_start, seg_end) * 111000

def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude, safety_radius):
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

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, map_type="satellite", straight_blocked=True, flight_altitude=50, drone_pos=None, direction="最佳航线", safety_radius=5):
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        if "向左" in direction:
            line_color = "purple"
        elif "向右" in direction:
            line_color = "orange"
        else:
            line_color = "green"
        folium.PolyLine(path_locations, color=line_color, weight=5, opacity=0.9, popup=f"✈️ {direction}").add_to(m)
        
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color=line_color, fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
    
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
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    # 初始化
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
        st.session_state.last_flight_altitude = 50
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
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("✈️ 无人机飞行高度")
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=10, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, value=st.session_state.safety_radius, step=1)
    
    if safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = safety_radius
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("💾 自动保存")
    auto_save = st.sidebar.checkbox("自动保存障碍物", value=st.session_state.auto_backup)
    st.session_state.auto_backup = auto_save
    
    if flight_alt != st.session_state.last_flight_altitude:
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
            st.rerun()
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    
    straight_blocked = False
    high_obstacles = 0
    for obs in st.session_state.obstacles_gcj:
        if obs.get('height', 30) > flight_alt:
            high_obstacles += 1
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], coords):
                straight_blocked = True
    
    st.sidebar.info(
        f"🏫 南京科技职业学院\n"
        f"🚧 障碍物: {obs_count} 个\n"
        f"📈 高于飞行高度: {high_obstacles} 个\n"
        f"📌 直线路径: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}\n"
        f"✈️ 飞行高度: {flight_alt} m\n"
        f"🛡️ 安全半径: {safety_radius} 米"
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
            st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，需要绕行")
        else:
            st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
        
        st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 输入高度并保存")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            with st.expander("📍 起点/终点设置", expanded=True):
                st.markdown("#### 🟢 起点 A")
                col_a1, col_a2 = st.columns(2)
                with col_a1:
                    a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat", step=0.000001)
                with col_a2:
                    a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng", step=0.000001)
                
                if st.button("📍 设置 A 点", use_container_width=True):
                    st.session_state.points_gcj['A'] = [a_lng, a_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        st.session_state.current_direction,
                        safety_radius
                    )
                    st.rerun()
                
                st.markdown("#### 🔴 终点 B")
                col_b1, col_b2 = st.columns(2)
                with col_b1:
                    b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat", step=0.000001)
                with col_b2:
                    b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng", step=0.000001)
                
                if st.button("📍 设置 B 点", use_container_width=True):
                    st.session_state.points_gcj['B'] = [b_lng, b_lat]
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        st.session_state.current_direction,
                        safety_radius
                    )
                    st.rerun()
            
            with st.expander("🤖 路径规划策略", expanded=True):
                st.markdown("**选择绕行方向：**")
                col_dir1, col_dir2, col_dir3 = st.columns(3)
                
                with col_dir1:
                    if st.button("🔄 最佳航线", use_container_width=True, type="primary" if st.session_state.current_direction == "最佳航线" else "secondary"):
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
                    if st.button("⬅️ 向左绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向左绕行" else "secondary"):
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
                
                with col_dir3:
                    if st.button("➡️ 向右绕行", use_container_width=True, type="primary" if st.session_state.current_direction == "向右绕行" else "secondary"):
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
                        st.success(f"已按照「{st.session_state.current_direction}」规划路径，{waypoint_count}个绕行点")
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
                        total_dist += distance(st.session_state.planned_path[i], st.session_state.planned_path[i+1]) * 111000
                    st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
                
                col_btn1, col_btn2 = st.columns(2)
                with col_btn1:
                    if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                        if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                            path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
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
            if straight_blocked:
                st.caption(f"当前避障策略: {st.session_state.current_direction}")
            st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色圆圈=安全半径")
            
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
            
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail, st.session_state.planned_path, map_type, straight_blocked, flight_alt, drone_pos, st.session_state.current_direction, safety_radius)
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
                                st.error("绘制的多边形无效，请重新绘制")
            
            if st.session_state.pending_obstacle is not None:
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
            
            st.caption("📌 **图例**：🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔴 红色=需绕行 | 🔵 蓝色圆圈=安全半径")
    
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
        else:
            st.session_state.last_hb_time = current_time
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            col1.metric("⏰ 时间", latest['timestamp'])
            col2.metric("📍 纬度", f"{latest['lat']:.6f}")
            col3.metric("📍 经度", f"{latest['lng']:.6f}")
            col4.metric("📊 高度", f"{latest['altitude']} m")
            col5.metric("🔋 电压", f"{latest['voltage']} V")
            col6.metric("🛰️ 卫星", latest['satellites'])
            
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
            
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords and len(coords) >= 3:
                    color = "red" if height > flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3, popup=f"🚧 {obs.get('name')}").add_to(monitor_map)
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                if "向左" in st.session_state.current_direction:
                    line_color = "purple"
                elif "向右" in st.session_state.current_direction:
                    line_color = "orange"
                else:
                    line_color = "green"
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color=line_color, weight=3, opacity=0.7).add_to(monitor_map)
            
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
            
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.7).add_to(monitor_map)
            
            folium.Marker([latest['lat'], latest['lng']], popup=f"当前位置", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
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
            
            st.subheader("📋 历史心跳记录")
            history_df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            display_cols = ['timestamp', 'flight_time', 'lat', 'lng', 'altitude', 'speed', 'voltage', 'satellites']
            display_cols = [col for col in display_cols if col in history_df.columns]
            st.dataframe(history_df[display_cols], use_container_width=True)
            
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                csv = df.to_csv(index=False)
                st.download_button("下载CSV文件", csv, f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
    
    # ==================== 障碍物管理页面（增强版-支持圈选和批量操作） ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        # 统计信息
        col_stats1, col_stats2, col_stats3, col_stats4 = st.columns(4)
        with col_stats1:
            st.metric("📊 障碍物总数", len(st.session_state.obstacles_gcj))
        with col_stats2:
            high_obs = sum(1 for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt)
            st.metric("🔴 需避让障碍物", high_obs)
        with col_stats3:
            safe_obs = len(st.session_state.obstacles_gcj) - high_obs
            st.metric("🟠 安全障碍物", safe_obs)
        with col_stats4:
            total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
            st.metric("📍 总顶点数", total_vertices)
        
        st.markdown("---")
        
        # 批量操作栏
        st.subheader("🎯 批量操作")
        col_batch1, col_batch2, col_batch3, col_batch4, col_batch5 = st.columns(5)
        
        # 初始化选中状态
        for i, obs in enumerate(st.session_state.obstacles_gcj):
            if 'selected' not in obs:
                obs['selected'] = False
        
        with col_batch1:
            select_all = st.checkbox("☑️ 全选所有")
            if select_all:
                for obs in st.session_state.obstacles_gcj:
                    obs['selected'] = True
        
        with col_batch2:
            if st.button("🗑️ 批量删除", use_container_width=True, type="primary"):
                selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                if selected_indices:
                    for i in reversed(selected_indices):
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
                    st.success(f"已删除 {len(selected_indices)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("请先选择要删除的障碍物")
        
        with col_batch3:
            batch_height = st.number_input("批量高度(m)", min_value=1, max_value=200, value=30, step=5, key="batch_height", label_visibility="collapsed")
        
        with col_batch4:
            if st.button("📏 批量设高度", use_container_width=True):
                selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                if selected_indices:
                    for i in selected_indices:
                        st.session_state.obstacles_gcj[i]['height'] = batch_height
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
                    st.success(f"已为 {len(selected_indices)} 个障碍物设置高度为 {batch_height}m")
                    st.rerun()
                else:
                    st.warning("请先选择要修改的障碍物")
        
        with col_batch5:
            if st.button("🏷️ 批量重命名", use_container_width=True):
                selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                if selected_indices:
                    st.session_state.show_rename_dialog = True
                else:
                    st.warning("请先选择要重命名的障碍物")
        
        # 批量重命名对话框
        if st.session_state.get('show_rename_dialog', False):
            with st.expander("🏷️ 批量重命名", expanded=True):
                col_n1, col_n2 = st.columns(2)
                with col_n1:
                    name_prefix = st.text_input("名称前缀", value="建筑物")
                    start_number = st.number_input("起始编号", min_value=1, value=1, step=1)
                with col_n2:
                    name_suffix = st.text_input("名称后缀", value="")
                col_confirm, col_cancel_r = st.columns(2)
                with col_confirm:
                    if st.button("确认重命名", use_container_width=True, type="primary"):
                        selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) if obs.get('selected', False)]
                        for idx, i in enumerate(selected_indices):
                            new_name = f"{name_prefix}{start_number + idx}{name_suffix}"
                            st.session_state.obstacles_gcj[i]['name'] = new_name
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.show_rename_dialog = False
                        st.success(f"已重命名 {len(selected_indices)} 个障碍物")
                        st.rerun()
                with col_cancel_r:
                    if st.button("取消"):
                        st.session_state.show_rename_dialog = False
                        st.rerun()
        
        st.markdown("---")
        
        # 创建标签页
        tab_list, tab_map = st.tabs(["📋 列表视图", "🗺️ 地图视图"])
        
        with tab_list:
            st.subheader("📝 障碍物列表（可多选）")
            st.caption("💡 提示：勾选复选框后可使用批量操作功能")
            
            if st.session_state.obstacles_gcj:
                # 使用网格布局显示障碍物
                items_per_row = 2
                rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row
                
                for row in range(rows):
                    cols = st.columns(items_per_row)
                    for col_idx in range(items_per_row):
                        idx = row * items_per_row + col_idx
                        if idx < len(st.session_state.obstacles_gcj):
                            obs = st.session_state.obstacles_gcj[idx]
                            with cols[col_idx]:
                                with st.container(border=True):
                                    height = obs.get('height', 30)
                                    color = "🔴" if height > flight_alt else "🟠"
                                    name = obs.get('name', f'障碍物{idx+1}')
                                    
                                    # 复选框
                                    checked = st.checkbox("选择", key=f"select_card_{idx}", value=obs.get('selected', False))
                                    st.session_state.obstacles_gcj[idx]['selected'] = checked
                                    
                                    st.markdown(f"**{color} {name}**")
                                    st.caption(f"📏 高度: {height}m")
                                    st.caption(f"📍 顶点: {len(obs.get('polygon', []))}个")
                                    
                                    # 快速编辑高度
                                    new_h = st.number_input("高度", value=height, min_value=1, max_value=200,
                                                           step=5, key=f"quick_edit_{idx}", label_visibility="collapsed")
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
                                    
                                    if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
                                        st.session_state.obstacles_gcj.pop(idx)
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
            else:
                st.info("暂无任何障碍物，可以在「地图视图」中绘制添加")
        
        with tab_map:
            st.subheader("🗺️ 障碍物地图视图")
            st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")
            
            # 地图类型选择
            col_map_type1, col_map_type2 = st.columns(2)
            with col_map_type1:
                map_view_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
                map_type_view = "satellite" if map_view_type == "卫星影像" else "vector"
            
            # 创建带绘制工具的地图
            tiles = GAODE_SATELLITE_URL if map_type_view == "satellite" else GAODE_VECTOR_URL
            obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
            
            # 添加绘图工具
            draw = plugins.Draw(
                export=True, position='topleft',
                draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 
                                         'fillColor': '#ff0000', 'fillOpacity': 0.4},
                              'polyline': False, 'rectangle': False, 'circle': False, 
                              'marker': False, 'circlemarker': False},
                edit_options={'edit': True, 'remove': True}
            )
            obs_map.add_child(draw)
            
            # 绘制障碍物
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
            
            # 添加起点和终点标记
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="起点", 
                         icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="终点", 
                         icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
            
            map_output = st_folium(obs_map, width=800, height=550, 
                                   key="obstacle_map_view",
                                   returned_objects=["last_active_drawing"])
            
            # 处理新绘制的障碍物
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
            
            # 添加障碍物对话框
            if st.session_state.pending_obstacle is not None:
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
                
                col_name1, col_name2 = st.columns(2)
                with col_name1:
                    new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
                with col_name2:
                    new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, 
                                                value=flight_alt + 10 if flight_alt < 100 else flight_alt,
                                                step=5, key="map_height_input")
                
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
        
        # 保存按钮
        st.markdown("---")
        col_save1, col_save2, col_save3 = st.columns(3)
        with col_save1:
            if st.button("💾 保存到文件", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success(f"已保存到 {CONFIG_FILE}")
                st.balloons()
        with col_save2:
            if st.button("📂 加载文件", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
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
                    st.warning("无配置文件")
        with col_save3:
            if st.button("🗑️ 清除所有", use_container_width=True):
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

if __name__ == "__main__":
    main()
