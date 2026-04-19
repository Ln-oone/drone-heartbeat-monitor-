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
DEFAULT_SAFETY_RADIUS_METERS = 5

# 高德地图瓦片地址
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

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

# ==================== 修复后的三种避障算法（真正绕过）====================
def find_left_path(start, end, obstacles_gcj, flight_altitude):
    """向左绕行：从障碍物左侧绕过，绕行点取上方或下方"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking_obs.append(obs)
    
    if not blocking_obs:
        return [start, end]
    
    path = [start]
    
    for obs in blocking_obs:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            bounds = get_polygon_bounds(coords)
            if bounds:
                offset_x = 0.0006  # 向左偏移约66米
                offset_y = 0.0004  # 向上或向下偏移约44米
                
                # 比较起点和终点的纬度，决定从上还是从下绕
                # 如果起点和终点纬度都高于障碍物，从上方绕过；否则从下方绕过
                start_above = start[1] > bounds['max_lat']
                end_above = end[1] > bounds['max_lat']
                start_below = start[1] < bounds['min_lat']
                end_below = end[1] < bounds['min_lat']
                
                if start_above and end_above:
                    # 都在上方，从上方绕过
                    waypoint = [bounds['min_lng'] - offset_x, bounds['max_lat'] + offset_y]
                elif start_below and end_below:
                    # 都在下方，从下方绕过
                    waypoint = [bounds['min_lng'] - offset_x, bounds['min_lat'] - offset_y]
                else:
                    # 一个上一个下，选择离起点更近的一侧
                    dist_to_top = abs(start[1] - bounds['max_lat'])
                    dist_to_bottom = abs(start[1] - bounds['min_lat'])
                    if dist_to_top < dist_to_bottom:
                        waypoint = [bounds['min_lng'] - offset_x, bounds['max_lat'] + offset_y]
                    else:
                        waypoint = [bounds['min_lng'] - offset_x, bounds['min_lat'] - offset_y]
                
                path.append(waypoint)
    
    path.append(end)
    return path

def find_right_path(start, end, obstacles_gcj, flight_altitude):
    """向右绕行：从障碍物右侧绕过，绕行点取上方或下方"""
    blocking_obs = []
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(start, end, coords):
                blocking_obs.append(obs)
    
    if not blocking_obs:
        return [start, end]
    
    path = [start]
    
    for obs in blocking_obs:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            bounds = get_polygon_bounds(coords)
            if bounds:
                offset_x = 0.0006
                offset_y = 0.0004
                
                start_above = start[1] > bounds['max_lat']
                end_above = end[1] > bounds['max_lat']
                start_below = start[1] < bounds['min_lat']
                end_below = end[1] < bounds['min_lat']
                
                if start_above and end_above:
                    waypoint = [bounds['max_lng'] + offset_x, bounds['max_lat'] + offset_y]
                elif start_below and end_below:
                    waypoint = [bounds['max_lng'] + offset_x, bounds['min_lat'] - offset_y]
                else:
                    dist_to_top = abs(start[1] - bounds['max_lat'])
                    dist_to_bottom = abs(start[1] - bounds['min_lat'])
                    if dist_to_top < dist_to_bottom:
                        waypoint = [bounds['max_lng'] + offset_x, bounds['max_lat'] + offset_y]
                    else:
                        waypoint = [bounds['max_lng'] + offset_x, bounds['min_lat'] - offset_y]
                
                path.append(waypoint)
    
    path.append(end)
    return path

def find_best_path(start, end, obstacles_gcj, flight_altitude):
    """最佳航线：分别计算左右路径，选择较短的那条"""
    left_path = find_left_path(start, end, obstacles_gcj, flight_altitude)
    right_path = find_right_path(start, end, obstacles_gcj, flight_altitude)
    
    left_len = 0
    for i in range(len(left_path) - 1):
        left_len += distance(left_path[i], left_path[i + 1])
    
    right_len = 0
    for i in range(len(right_path) - 1):
        right_len += distance(right_path[i], right_path[i + 1])
    
    return left_path if left_len < right_len else right_path

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction):
    if direction == "向左绕行":
        return find_left_path(start, end, obstacles_gcj, flight_altitude)
    elif direction == "向右绕行":
        return find_right_path(start, end, obstacles_gcj, flight_altitude)
    else:
        return find_best_path(start, end, obstacles_gcj, flight_altitude)

# ==================== 障碍物管理 ====================
def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return data.get('obstacles', [])
        except:
            return []
    return []

def save_obstacles(obstacles):
    data = {
        'obstacles': obstacles,
        'count': len(obstacles),
        'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        'version': 'v12.2'
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
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += distance(path[i], path[i + 1])
    
    def check_safety(self, obstacles_gcj):
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
                    # 简化计算：使用度数距离近似
                    dist = point_to_segment_distance_deg(self.current_pos, p1, p2) * 111000
                    if dist < min_distance:
                        min_distance = dist
                        danger_name = obs.get('name', '障碍物')
        
        if min_distance < self.safety_radius:
            return False, min_distance, danger_name
        return True, min_distance if min_distance != float('inf') else None, None

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
    
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            st.session_state.current_direction
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
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            st.session_state.current_direction
        )
        st.rerun()
    
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
            
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    st.session_state.current_direction
                )
                st.rerun()
            
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    st.session_state.current_direction
                )
                st.rerun()
            
            st.markdown("#### 🤖 路径规划")
            
            st.markdown("**选择绕行方向：**")
            col_dir1, col_dir2, col_dir3 = st.columns(3)
            
            with col_dir1:
                if st.button("🔄 最佳航线", use_container_width=True):
                    st.session_state.current_direction = "最佳航线"
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        "最佳航线"
                    )
                    st.rerun()
            
            with col_dir2:
                if st.button("⬅️ 向左绕行", use_container_width=True):
                    st.session_state.current_direction = "向左绕行"
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        "向左绕行"
                    )
                    st.rerun()
            
            with col_dir3:
                if st.button("➡️ 向右绕行", use_container_width=True):
                    st.session_state.current_direction = "向右绕行"
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        "向右绕行"
                    )
                    st.rerun()
            
            st.info(f"📌 当前绕行策略: **{st.session_state.current_direction}**")
            
            if st.button("🔄 重新规划路径", use_container_width=True):
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    st.session_state.current_direction
                )
                st.success(f"已按照「{st.session_state.current_direction}」规划路径，共 {len(st.session_state.planned_path)} 个航点")
                st.rerun()
            
            st.markdown("#### ✈️ 飞行参数")
            st.metric("当前飞行高度", f"{flight_alt} m")
            st.metric("速度系数", f"{drone_speed}%")
            st.metric("当前策略", st.session_state.current_direction)
            st.metric("🛡️ 安全半径", f"{safety_radius} 米")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed, safety_radius)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    st.success(f"🚁 飞行已开始！按照「{st.session_state.current_direction}」飞行")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.session_state.heartbeat_sim.simulating = False
                    st.info("飞行已停止")
            
            st.markdown("### 📍 当前航点")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
            st.caption(f"📏 直线距离: {dist:.0f} 米")
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
                path_dist = 0
                for i in range(len(st.session_state.planned_path)-1):
                    p1, p2 = st.session_state.planned_path[i], st.session_state.planned_path[i+1]
                    path_dist += math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) * 111000
                st.caption(f"🔄 避障路径: {path_dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            if straight_blocked:
                st.caption(f"当前避障策略: {st.session_state.current_direction}")
                st.caption("🟢 绿色=最佳航线 | 🟣 紫色=向左绕行 | 🟠 橙色=向右绕行 | 🔵 蓝色圆圈=安全半径")
            else:
                st.caption("✅ 直线畅通 | 🔵 蓝色圆圈=安全半径")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    st.session_state.current_direction
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
                            st.session_state.pending_obstacle = poly
                            st.rerun()
            
            if st.session_state.pending_obstacle is not None:
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
                
                new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, value=30, step=5, key="height_input")
                
                col_ok, col_cancel = st.columns(2)
                with col_ok:
                    if st.button("✅ 确认添加", use_container_width=True, type="primary"):
                        new_name = f"建筑物{len(st.session_state.obstacles_gcj) + 1}"
                        st.session_state.obstacles_gcj.append({
                            "name": new_name,
                            "polygon": st.session_state.pending_obstacle,
                            "height": new_height
                        })
                        save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction
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
                    st.session_state.last_hb_time = current_time
                    st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                    if len(st.session_state.flight_history) > 200:
                        st.session_state.flight_history.pop(0)
                    if not st.session_state.heartbeat_sim.simulating:
                        st.session_state.simulation_running = False
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
            
            col7, col8 = st.columns(2)
            col7.metric("💨 速度", f"{latest.get('speed', 0)} m/s")
            col8.metric("⚡ 速度系数", f"{drone_speed}%")
            
            progress = latest.get('progress', 0)
            st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
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
            
            st.subheader("📈 数据图表")
            if len(st.session_state.heartbeat_sim.history) > 1:
                alt_df = pd.DataFrame({
                    "序号": range(min(20, len(st.session_state.heartbeat_sim.history))),
                    "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]
                })
                st.line_chart(alt_df, x="序号", y="高度(m)")
            
            st.subheader("📋 历史心跳记录")
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_sim.history[:10]), use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        st.subheader("💾 一键保存/加载")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物 | 🛡️ 安全半径: {safety_radius} 米")
        
        col_save_load1, col_save_load2, col_save_load3 = st.columns(3)
        with col_save_load1:
            if st.button("💾 一键保存到JSON", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success(f"已保存到 {CONFIG_FILE}")
        with col_save_load2:
            if st.button("📂 一键加载JSON", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        st.session_state.current_direction
                    )
                    st.success(f"已加载 {len(loaded)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("无配置文件")
        with col_save_load3:
            config_data = {'obstacles': st.session_state.obstacles_gcj, 'count': len(st.session_state.obstacles_gcj), 'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'version': 'v12.2'}
            st.download_button(label="📥 下载配置", data=json.dumps(config_data, ensure_ascii=False, indent=2), file_name=CONFIG_FILE, mime="application/json", use_container_width=True)
        
        st.markdown("---")
        st.subheader("📝 障碍物列表")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            if st.session_state.obstacles_gcj:
                for i, obs in enumerate(st.session_state.obstacles_gcj):
                    height = obs.get('height', 30)
                    color = "🔴" if height > flight_alt else "🟠"
                    with st.container():
                        col_name, col_height, col_edit, col_del = st.columns([2, 1, 1, 1])
                        col_name.write(f"{color} {obs.get('name', f'障碍物{i+1}')}")
                        col_height.write(f"{height}m")
                        with col_edit:
                            new_h = st.number_input("", value=height, min_value=1, max_value=200, key=f"edit_{i}", label_visibility="collapsed")
                            if new_h != height:
                                obs['height'] = new_h
                                save_obstacles(st.session_state.obstacles_gcj)
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    st.session_state.current_direction
                                )
                                st.rerun()
                        with col_del:
                            if st.button("🗑️", key=f"del_{i}"):
                                st.session_state.obstacles_gcj.pop(i)
                                save_obstacles(st.session_state.obstacles_gcj)
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    st.session_state.current_direction
                                )
                                st.rerun()
            else:
                st.write("暂无障碍物")
            
            if st.button("🗑️ 清除所有障碍物", use_container_width=True):
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    [],
                    flight_alt,
                    st.session_state.current_direction
                )
                st.rerun()
        
        with col2:
            st.subheader("🗺️ 障碍物分布图")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                color = "red" if height > flight_alt else "orange"
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_opacity=0.5, popup=f"{obs.get('name')}\n高度: {height}m").add_to(obs_map)
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
            folium_static(obs_map, width=700, height=500)

if __name__ == "__main__":
    main()
