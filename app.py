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

# 安全半径（米）
SAFETY_RADIUS_METERS = 5

# 绕行方向选项
AVOID_DIRECTIONS = ["🔄 最佳航线（自动计算最短路径）", "⬅️ 向左绕行", "➡️ 向右绕行"]

# 高德地图瓦片地址
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标系转换函数 ====================
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def wgs84_to_gcj02(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

# ==================== 距离计算 ====================
def point_to_segment_distance(point, seg_start, seg_end):
    px, py = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    
    if len_sq == 0:
        dist_deg = math.sqrt((px-x1)**2 + (py-y1)**2)
        return dist_deg * 111000
    
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0, min(1, t))
    
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    
    dist_deg = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    return dist_deg * 111000

# ==================== 避障路径规划算法（支持绕行方向）====================
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

def segments_intersect(p1, p2, p3, p4):
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            if not (p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4):
                return True
    return False

def is_path_blocked_by_obstacle(p1, p2, obstacle, flight_altitude):
    coords = obstacle.get('polygon', [])
    obstacle_height = obstacle.get('height', 30)
    
    if not coords or len(coords) < 3:
        return False
    
    if obstacle_height <= flight_altitude:
        return False
    
    return line_intersects_polygon(p1, p2, coords)

def is_path_blocked(p1, p2, obstacles_gcj, flight_altitude):
    for obs in obstacles_gcj:
        if is_path_blocked_by_obstacle(p1, p2, obs, flight_altitude):
            return True
    return False

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_polygon_center(polygon):
    """获取多边形中心点"""
    if not polygon:
        return None
    sum_x = sum(p[0] for p in polygon)
    sum_y = sum(p[1] for p in polygon)
    return [sum_x / len(polygon), sum_y / len(polygon)]

def get_side_offset_points(polygon, direction="left", offset_deg=0.0002):
    """
    获取多边形左右两侧的绕行点
    direction: "left" 或 "right"
    """
    if not polygon or len(polygon) < 3:
        return []
    
    center = get_polygon_center(polygon)
    if not center:
        return []
    
    offset_points = []
    
    for pt in polygon:
        # 计算从中心到顶点的向量
        dx = pt[0] - center[0]
        dy = pt[1] - center[1]
        
        # 计算垂直向量（绕行方向）
        if direction == "left":
            # 向左绕行：逆时针垂直
            perp_x = -dy
            perp_y = dx
        else:
            # 向右绕行：顺时针垂直
            perp_x = dy
            perp_y = -dx
        
        # 归一化并应用偏移
        length = math.sqrt(perp_x*perp_x + perp_y*perp_y)
        if length > 0:
            perp_x /= length
            perp_y /= length
        
        # 偏移点
        offset_x = pt[0] + perp_x * offset_deg
        offset_y = pt[1] + perp_y * offset_deg
        offset_points.append([offset_x, offset_y])
    
    return offset_points

def find_path_with_direction(start, end, obstacles_gcj, flight_altitude, direction="best"):
    """
    根据指定方向规划路径
    direction: "best", "left", "right"
    """
    # 先检查是否有需要绕行的障碍物
    need_avoid = False
    blocking_obstacles = []
    for obs in obstacles_gcj:
        obs_height = obs.get('height', 30)
        if obs_height > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                if line_intersects_polygon(start, end, coords):
                    need_avoid = True
                    blocking_obstacles.append(obs)
    
    if not need_avoid:
        return [start, end]
    
    if direction == "best":
        # 使用原有 A* 算法找最短路径
        return find_shortest_path_astar(start, end, obstacles_gcj, flight_altitude)
    
    # 向左或向右绕行
    # 收集所有绕行点
    waypoints = [start, end]
    
    for obs in blocking_obstacles:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            # 获取指定方向的偏移点
            offset_points = get_side_offset_points(coords, direction, 0.00025)
            waypoints.extend(offset_points)
    
    # 去重并连接路径
    unique_waypoints = []
    for wp in waypoints:
        is_unique = True
        for u in unique_waypoints:
            if abs(wp[0]-u[0]) < 0.00001 and abs(wp[1]-u[1]) < 0.00001:
                is_unique = False
                break
        if is_unique:
            unique_waypoints.append(wp)
    
    # 按距离起点排序
    unique_waypoints.sort(key=lambda p: distance(start, p))
    
    # 构建路径
    path = [start]
    current = start
    
    # 贪心算法连接路径点
    remaining = [p for p in unique_waypoints if p != start and p != end]
    while remaining:
        # 找到最近的可见点
        nearest = None
        min_dist = float('inf')
        for p in remaining:
            if not is_path_blocked(current, p, obstacles_gcj, flight_altitude):
                dist = distance(current, p)
                if dist < min_dist:
                    min_dist = dist
                    nearest = p
        
        if nearest:
            path.append(nearest)
            current = nearest
            remaining.remove(nearest)
        else:
            break
    
    path.append(end)
    
    # 简化路径
    simplified = [path[0]]
    for i in range(1, len(path) - 1):
        prev = simplified[-1]
        curr = path[i]
        nxt = path[i + 1]
        if not is_path_blocked(prev, nxt, obstacles_gcj, flight_altitude):
            # 可以跳过中间点
            continue
        simplified.append(curr)
    simplified.append(path[-1])
    
    return simplified

def find_shortest_path_astar(start, end, obstacles_gcj, flight_altitude):
    """A* 算法找最短路径"""
    # 收集路径点
    waypoints = [start, end]
    
    for obs in obstacles_gcj:
        obs_height = obs.get('height', 30)
        if obs_height > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and len(coords) >= 3:
                for pt in coords:
                    waypoints.append([pt[0], pt[1]])
                    offset = 0.00015
                    waypoints.append([pt[0] + offset, pt[1]])
                    waypoints.append([pt[0] - offset, pt[1]])
                    waypoints.append([pt[0], pt[1] + offset])
                    waypoints.append([pt[0], pt[1] - offset])
    
    # 去重
    unique_waypoints = []
    for wp in waypoints:
        is_unique = True
        for u in unique_waypoints:
            if abs(wp[0]-u[0]) < 0.00001 and abs(wp[1]-u[1]) < 0.00001:
                is_unique = False
                break
        if is_unique:
            unique_waypoints.append(wp)
    
    n = len(unique_waypoints)
    graph = {}
    for i in range(n):
        graph[i] = []
        for j in range(n):
            if i == j:
                continue
            if not is_path_blocked(unique_waypoints[i], unique_waypoints[j], obstacles_gcj, flight_altitude):
                dist = distance(unique_waypoints[i], unique_waypoints[j])
                graph[i].append((j, dist))
    
    start_idx = None
    end_idx = None
    for i, wp in enumerate(unique_waypoints):
        if abs(wp[0]-start[0]) < 0.00001 and abs(wp[1]-start[1]) < 0.00001:
            start_idx = i
        if abs(wp[0]-end[0]) < 0.00001 and abs(wp[1]-end[1]) < 0.00001:
            end_idx = i
    
    if start_idx is None or end_idx is None:
        return [start, end]
    
    import heapq
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}
    g_score = {i: float('inf') for i in range(n)}
    g_score[start_idx] = 0
    f_score = {i: float('inf') for i in range(n)}
    
    def heuristic(a, b):
        return distance(unique_waypoints[a], unique_waypoints[b])
    
    f_score[start_idx] = heuristic(start_idx, end_idx)
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == end_idx:
            path = []
            while current in came_from:
                path.append(unique_waypoints[current])
                current = came_from[current]
            path.append(unique_waypoints[start_idx])
            path.reverse()
            simplified = [path[0]]
            for i in range(1, len(path) - 1):
                prev = simplified[-1]
                curr = path[i]
                nxt = path[i + 1]
                angle1 = math.atan2(curr[1]-prev[1], curr[0]-prev[0])
                angle2 = math.atan2(nxt[1]-curr[1], nxt[0]-curr[0])
                if abs(angle1 - angle2) > 0.01:
                    simplified.append(curr)
            simplified.append(path[-1])
            return simplified
        
        for neighbor, dist in graph.get(current, []):
            tentative_g = g_score[current] + dist
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, end_idx)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return [start, end]

def create_avoidance_path(start, end, obstacles_gcj, flight_altitude, direction="best"):
    """创建避开障碍物的路径"""
    return find_path_with_direction(start, end, obstacles_gcj, flight_altitude, direction)

# ==================== 安全半径检查 ====================
def check_safety_radius(drone_pos, obstacles_gcj, flight_altitude):
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
                
                dist_m = point_to_segment_distance(drone_pos, p1, p2)
                
                if dist_m < min_distance:
                    min_distance = dist_m
                    danger_name = obs.get('name', '障碍物')
    
    if min_distance < SAFETY_RADIUS_METERS:
        return False, min_distance, danger_name
    return True, min_distance if min_distance != float('inf') else None, None

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
        self.safety_violation = False
        
    def set_path(self, path, altitude=50, speed=50):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += distance(path[i], path[i + 1])
    
    def check_safety(self, obstacles_gcj):
        is_safe, dist, name = check_safety_radius(self.current_pos, obstacles_gcj, self.flight_altitude)
        self.safety_violation = not is_safe
        return is_safe, dist, name
        
    def update_and_generate(self, obstacles_gcj):
        if self.simulating and self.path_index < len(self.path) - 1:
            target = self.path[self.path_index + 1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist_to_target = math.sqrt(dx*dx + dy*dy)
            
            step = 0.00015 + (self.speed / 100) * 0.0005
            
            if dist_to_target < step:
                self.distance_traveled += dist_to_target
                self.current_pos = target.copy()
                self.path_index += 1
            else:
                ratio = step / dist_to_target
                self.current_pos[0] += dx * ratio
                self.current_pos[1] += dy * ratio
                self.distance_traveled += step
            
            if self.total_distance > 0:
                self.progress = min(1.0, self.distance_traveled / self.total_distance)
            
            if self.path_index >= len(self.path) - 1:
                self.simulating = False
                self.progress = 1.0
        elif self.path_index >= len(self.path) - 1:
            self.simulating = False
            self.progress = 1.0
        
        is_safe, danger_dist, danger_name = self.check_safety(obstacles_gcj)
        
        altitude = self.flight_altitude + random.randint(-5, 5) if self.simulating else random.randint(0, 10)
        speed_display = round(self.speed * 0.1, 1) if self.simulating else 0
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": speed_display,
            "progress": self.progress,
            "distance_traveled": self.distance_traveled,
            "total_distance": self.total_distance,
            "simulating": self.simulating,
            "safety_status": "✅ 安全" if is_safe else "⚠️ 危险",
            "safety_distance": danger_dist,
            "safety_danger": danger_name,
            "safety_violation": not is_safe
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        
        return heartbeat

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, map_type="satellite", straight_blocked=True, flight_altitude=50, drone_pos=None):
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
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}\n高度: {height}m").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        folium.PolyLine(path_locations, color="green", weight=5, opacity=0.9, popup="✈️ 智能避障航线").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=4, color="green", fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线（畅通）").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被高层建筑物阻挡").add_to(m)
    
    if drone_pos:
        folium.Circle(
            radius=SAFETY_RADIUS_METERS,
            location=[drone_pos[1], drone_pos[0]],
            color="blue",
            weight=2,
            fill=True,
            fill_color="blue",
            fill_opacity=0.2,
            popup=f"🛡️ 安全半径: {SAFETY_RADIUS_METERS}米"
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
    if "avoid_direction" not in st.session_state:
        st.session_state.avoid_direction = "🔄 最佳航线（自动计算最短路径）"
    
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
    flight_alt = st.sidebar.slider("飞行高度 (m)", min_value=10, max_value=200, value=50, step=5)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🛡️ 安全防护")
    st.sidebar.info(f"**安全半径: {SAFETY_RADIUS_METERS} 米**\n\n超出半径 = ✅ 安全\n小于半径 = ⚠️ 危险")
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🧭 避障策略")
    avoid_direction = st.sidebar.radio(
        "选择绕行方向",
        AVOID_DIRECTIONS,
        index=0,
        help="当障碍物高于飞行高度时，选择如何绕过"
    )
    
    # 检测方向是否改变
    if avoid_direction != st.session_state.avoid_direction:
        st.session_state.avoid_direction = avoid_direction
        # 方向改变时重新规划路径
        direction_key = avoid_direction.split(" ")[1] if "⬅️" in avoid_direction else ("right" if "➡️" in avoid_direction else "best")
        if direction_key == "向左绕行":
            dir_value = "left"
        elif direction_key == "向右绕行":
            dir_value = "right"
        else:
            dir_value = "best"
        
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            dir_value
        )
        st.rerun()
    
    # 当飞行高度改变时，自动重新规划路径
    if flight_alt != st.session_state.last_flight_altitude:
        st.session_state.last_flight_altitude = flight_alt
        direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
        if direction_key == "向左绕行":
            dir_value = "left"
        elif direction_key == "向右绕行":
            dir_value = "right"
        else:
            dir_value = "best"
        
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            dir_value
        )
        st.rerun()
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    high_obstacles = [obs for obs in st.session_state.obstacles_gcj if obs.get('height', 30) > flight_alt]
    straight_blocked = is_path_blocked(
        st.session_state.points_gcj['A'],
        st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj,
        flight_alt
    )
    
    st.sidebar.info(
        f"🏫 南京科技职业学院\n"
        f"🚧 障碍物: {obs_count} 个\n"
        f"📈 高于飞行高度: {len(high_obstacles)} 个\n"
        f"📌 直线路径: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}\n"
        f"✈️ 飞行高度: {flight_alt} m\n"
        f"🛡️ 安全半径: {SAFETY_RADIUS_METERS} m\n"
        f"🧭 当前策略: {st.session_state.avoid_direction}"
    )
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
        if direction_key == "向左绕行":
            dir_value = "left"
        elif direction_key == "向右绕行":
            dir_value = "right"
        else:
            dir_value = "best"
        
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            dir_value
        )
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        st.info(f"✈️ **当前飞行高度: {flight_alt} 米** | 🔴 红色障碍物需要绕行 | 🟠 橙色障碍物可直接飞越 | 🛡️ 蓝色圆圈为无人机安全半径({SAFETY_RADIUS_METERS}米) | 🧭 **当前避障策略: {st.session_state.avoid_direction}**")
        
        straight_blocked = is_path_blocked(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt
        )
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {len(high_obstacles)} 个障碍物高于飞行高度({flight_alt}m)，已按照「{st.session_state.avoid_direction}」规划航线")
        else:
            st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度，可直接飞越）")
        
        st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成 → 在弹出的对话框中输入高度并保存")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    dir_value
                )
                st.rerun()
            
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    dir_value
                )
                st.rerun()
            
            st.markdown("#### 🤖 路径规划")
            if st.button("🔄 重新规划路径", use_container_width=True):
                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    dir_value
                )
                if st.session_state.planned_path:
                    st.success(f"已按照「{st.session_state.avoid_direction}」规划 {len(st.session_state.planned_path)} 个航点")
                st.rerun()
            
            st.markdown("#### ✈️ 飞行参数")
            st.metric("当前飞行高度", f"{flight_alt} m")
            st.metric("速度系数", f"{drone_speed}%")
            st.metric("🛡️ 安全半径", f"{SAFETY_RADIUS_METERS} m")
            st.metric("🧭 避障策略", st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("向右绕行" if "➡️" in st.session_state.avoid_direction else "最佳航线"))
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    if straight_blocked:
                        st.success(f"🚁 飞行已开始！无人机将按照「{st.session_state.avoid_direction}」飞行")
                    else:
                        st.success("🚁 飞行已开始！无人机将沿直线飞越低矮建筑物")
            
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
                st.caption(f"🟢 **绿色线 = {st.session_state.avoid_direction}** | ⚪ 灰色虚线 = 直线（被阻挡） | 🛡️ 蓝色圆圈 = 安全半径")
            else:
                st.caption("✅ 直线畅通 | 🛡️ 蓝色圆圈 = 安全半径")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_path is None:
                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    dir_value
                )
            
            drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
            
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail, st.session_state.planned_path, map_type, straight_blocked, flight_alt, drone_pos)
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
            
            # 显示高度输入对话框
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
                        # 重新规划路径
                        direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                        dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            dir_value
                        )
                        st.session_state.pending_obstacle = None
                        st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
                        st.rerun()
                with col_cancel:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_obstacle = None
                        st.rerun()
            
            st.caption("📌 **图例**：🟢 绿色=避障航线 | 🔴 红色=需绕行 | 🟠 橙色=可飞越 | 🛡️ 蓝色圆圈=安全半径")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        st.caption(f"✈️ 当前飞行高度: {flight_alt} 米 | 🛡️ 安全半径: {SAFETY_RADIUS_METERS} 米 | 🧭 避障策略: {st.session_state.avoid_direction}")
        
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
            
            if latest.get('safety_violation', False):
                st.error(f"⚠️ **安全警告！** 无人机距离 {latest.get('safety_danger', '障碍物')} 仅 {latest.get('safety_distance', 0):.1f} 米，已进入安全半径({SAFETY_RADIUS_METERS}米)内！")
            else:
                if latest.get('safety_distance'):
                    st.success(f"✅ **安全状态**：距离最近障碍物 {latest.get('safety_distance', 0):.1f} 米")
                else:
                    st.success("✅ **安全状态**：周围无障碍物")
            
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
            
            dist_traveled = latest.get('distance_traveled', 0) * 111000
            total_dist = latest.get('total_distance', 0) * 111000
            st.caption(f"📏 已飞距离: {dist_traveled:.0f} 米 / 总距离: {total_dist:.0f} 米")
            
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords and len(coords) >= 3:
                    color = "red" if height > flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(monitor_map)
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3, opacity=0.7, popup="避障航线").add_to(monitor_map)
            
            folium.Circle(
                radius=SAFETY_RADIUS_METERS,
                location=[latest['lat'], latest['lng']],
                color="blue",
                weight=2,
                fill=True,
                fill_color="blue",
                fill_opacity=0.2,
                popup=f"🛡️ 安全半径: {SAFETY_RADIUS_METERS}米"
            ).add_to(monitor_map)
            
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.7, popup="历史轨迹").add_to(monitor_map)
            
            folium.Marker([latest['lat'], latest['lng']], popup=f"📍 当前位置\n高度: {latest['altitude']}m\n安全状态: {latest.get('safety_status', '未知')}", 
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                             popup="🟢 起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                             popup="🔴 终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            st.subheader("📈 数据图表")
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": range(min(20, len(st.session_state.heartbeat_sim.history))),
                        "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
                    st.caption("📊 高度变化趋势")
            with chart_col2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": range(min(20, len(st.session_state.heartbeat_sim.history))),
                        "速度(m/s)": [h.get("speed", 0) for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
                    st.caption("📊 速度变化趋势")
            
            st.subheader("📋 历史心跳记录")
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_sim.history[:10]), use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        col_refresh, col_clear = st.columns(2)
        with col_refresh:
            if st.button("🔄 立即刷新", use_container_width=True):
                if st.session_state.simulation_running:
                    st.session_state.heartbeat_sim.update_and_generate(st.session_state.obstacles_gcj)
                    st.rerun()
        with col_clear:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_sim.history = []
                st.rerun()
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        st.subheader("💾 一键保存/加载")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物")
        
        col_save_load1, col_save_load2, col_save_load3 = st.columns(3)
        with col_save_load1:
            if st.button("💾 一键保存到JSON", use_container_width=True, type="primary"):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success(f"已保存 {len(st.session_state.obstacles_gcj)} 个障碍物到 {CONFIG_FILE}")
        with col_save_load2:
            if st.button("📂 一键加载JSON", use_container_width=True):
                loaded = load_obstacles()
                if loaded:
                    st.session_state.obstacles_gcj = loaded
                    direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                    dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        dir_value
                    )
                    st.success(f"已加载 {len(loaded)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("无配置文件或文件为空")
        with col_save_load3:
            config_data = {
                'obstacles': st.session_state.obstacles_gcj,
                'count': len(st.session_state.obstacles_gcj),
                'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'version': 'v12.2'
            }
            st.download_button(
                label="📥 下载配置文件",
                data=json.dumps(config_data, ensure_ascii=False, indent=2),
                file_name=CONFIG_FILE,
                mime="application/json",
                use_container_width=True
            )
        
        st.markdown("---")
        st.subheader("📝 障碍物列表")
        st.caption("**🔴 红色** = 高度 > 飞行高度（需绕行） | **🟠 橙色** = 高度 ≤ 飞行高度（可飞越）")
        
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
                                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    dir_value
                                )
                                st.rerun()
                        with col_del:
                            if st.button("🗑️", key=f"del_{i}"):
                                st.session_state.obstacles_gcj.pop(i)
                                save_obstacles(st.session_state.obstacles_gcj)
                                direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                                dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    dir_value
                                )
                                st.rerun()
            else:
                st.write("暂无障碍物，请在「航线规划」页面圈选添加")
            
            st.markdown("---")
            col_clear_all, _ = st.columns(2)
            with col_clear_all:
                if st.button("🗑️ 清除所有障碍物", use_container_width=True):
                    st.session_state.obstacles_gcj = []
                    save_obstacles([])
                    direction_key = st.session_state.avoid_direction.split(" ")[1] if "⬅️" in st.session_state.avoid_direction else ("right" if "➡️" in st.session_state.avoid_direction else "best")
                    dir_value = "left" if direction_key == "向左绕行" else ("right" if direction_key == "向右绕行" else "best")
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        [],
                        flight_alt,
                        dir_value
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
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="🟢 起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="🔴 终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
            folium_static(obs_map, width=700, height=500)

if __name__ == "__main__":
    main()
