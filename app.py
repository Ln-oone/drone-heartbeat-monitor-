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
import numpy as np

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

def meters_to_deg(meters, lat=32.23):
    lat_deg = meters / 111000
    lng_deg = meters / (111000 * math.cos(math.radians(lat)))
    return lng_deg, lat_deg

def is_path_clear(p1, p2, obstacles_gcj, flight_altitude):
    """检查两点之间的路径是否畅通"""
    for obs in obstacles_gcj:
        if obs.get('height', 30) > flight_altitude:
            coords = obs.get('polygon', [])
            if coords and line_intersects_polygon(p1, p2, coords):
                return False
    return True

# ==================== 改进的避障算法 - 不穿越障碍物 ====================
def find_clear_path_around_obstacle(start, end, obstacle_polygon, safety_distance_meters=20, side="auto"):
    """
    绕行单个障碍物，返回绕行点
    side: "left", "right", "top", "bottom", "auto"
    """
    bounds = get_polygon_bounds(obstacle_polygon)
    if not bounds:
        return [start, end]
    
    offset_lng, offset_lat = meters_to_deg(safety_distance_meters, start[1])
    
    # 计算四个方向的绕行点
    left_point = [bounds['min_lng'] - offset_lng, (start[1] + end[1]) / 2]
    right_point = [bounds['max_lng'] + offset_lng, (start[1] + end[1]) / 2]
    top_point = [(start[0] + end[0]) / 2, bounds['max_lat'] + offset_lat]
    bottom_point = [(start[0] + end[0]) / 2, bounds['min_lat'] - offset_lat]
    
    # 根据起点和终点位置智能选择最佳绕行方向
    if side == "left":
        candidates = [("left", left_point)]
    elif side == "right":
        candidates = [("right", right_point)]
    elif side == "top":
        candidates = [("top", top_point)]
    elif side == "bottom":
        candidates = [("bottom", bottom_point)]
    else:  # auto - 选择最短路径
        candidates = [
            ("left", left_point),
            ("right", right_point),
            ("top", top_point),
            ("bottom", bottom_point)
        ]
    
    valid_paths = []
    for name, waypoint in candidates:
        # 检查两段路径是否都不穿过障碍物
        if not line_intersects_polygon(start, waypoint, obstacle_polygon) and \
           not line_intersects_polygon(waypoint, end, obstacle_polygon):
            # 检查绕行点是否在障碍物内部
            if not point_in_polygon(waypoint, obstacle_polygon):
                total_dist = distance(start, waypoint) + distance(waypoint, end)
                valid_paths.append((total_dist, [start, waypoint, end], name))
    
    if valid_paths:
        valid_paths.sort(key=lambda x: x[0])
        return valid_paths[0][1]
    
    # 如果单点绕行失败，使用两点绕行
    return find_two_point_bypass(start, end, obstacle_polygon, safety_distance_meters)

def find_two_point_bypass(start, end, obstacle_polygon, safety_distance_meters=20):
    """使用两个绕行点绕过障碍物（矩形绕行）"""
    bounds = get_polygon_bounds(obstacle_polygon)
    if not bounds:
        return [start, end]
    
    offset_lng, offset_lat = meters_to_deg(safety_distance_meters, start[1])
    
    # 矩形绕行方案
    bypass_paths = [
        # 左上绕行
        ([start, 
          [bounds['min_lng'] - offset_lng, start[1]],
          [bounds['min_lng'] - offset_lng, end[1]],
          end], "左上"),
        # 右上绕行
        ([start,
          [bounds['max_lng'] + offset_lng, start[1]],
          [bounds['max_lng'] + offset_lng, end[1]],
          end], "右上"),
        # 左下绕行
        ([start,
          [start[0], bounds['min_lat'] - offset_lat],
          [end[0], bounds['min_lat'] - offset_lat],
          end], "左下"),
        # 右下绕行
        ([start,
          [start[0], bounds['max_lat'] + offset_lat],
          [end[0], bounds['max_lat'] + offset_lat],
          end], "右下"),
    ]
    
    valid_paths = []
    for path, name in bypass_paths:
        valid = True
        for i in range(len(path) - 1):
            if line_intersects_polygon(path[i], path[i+1], obstacle_polygon):
                valid = False
                break
        if valid:
            total_dist = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
            valid_paths.append((total_dist, path, name))
    
    if valid_paths:
        valid_paths.sort(key=lambda x: x[0])
        return valid_paths[0][1]
    
    return [start, end]

def find_avoidance_path_multiple_obstacles(start, end, obstacles_gcj, flight_altitude, safety_distance_meters=20, direction="auto"):
    """
    绕过多个障碍物，返回完整的路径点序列
    direction: "left", "right", "top", "bottom", "auto"
    """
    # 只考虑高于飞行高度的障碍物
    relevant_obstacles = [obs for obs in obstacles_gcj if obs.get('height', 30) > flight_altitude]
    
    if not relevant_obstacles:
        return [start, end]
    
    # 检查直线是否畅通
    if is_path_clear(start, end, relevant_obstacles, flight_altitude):
        return [start, end]
    
    # 找出所有阻挡航线的障碍物
    blocking_obstacles = []
    for obs in relevant_obstacles:
        coords = obs.get('polygon', [])
        if coords and line_intersects_polygon(start, end, coords):
            blocking_obstacles.append(obs)
    
    if not blocking_obstacles:
        return [start, end]
    
    # 合并所有阻挡障碍物的边界
    all_bounds = []
    for obs in blocking_obstacles:
        coords = obs.get('polygon', [])
        if coords:
            bounds = get_polygon_bounds(coords)
            if bounds:
                all_bounds.append(bounds)
    
    if not all_bounds:
        return [start, end]
    
    # 计算整体边界
    min_lng = min(b['min_lng'] for b in all_bounds)
    max_lng = max(b['max_lng'] for b in all_bounds)
    min_lat = min(b['min_lat'] for b in all_bounds)
    max_lat = max(b['max_lat'] for b in all_bounds)
    
    offset_lng, offset_lat = meters_to_deg(safety_distance_meters, start[1])
    
    # 根据方向选择绕行策略
    if direction == "left":
        # 左侧绕行：向左绕过所有障碍物
        bypass_x = min_lng - offset_lng
        path = [
            start,
            [bypass_x, start[1]],
            [bypass_x, end[1]],
            end
        ]
    elif direction == "right":
        # 右侧绕行：向右绕过所有障碍物
        bypass_x = max_lng + offset_lng
        path = [
            start,
            [bypass_x, start[1]],
            [bypass_x, end[1]],
            end
        ]
    elif direction == "top":
        # 上侧绕行：向上绕过所有障碍物
        bypass_y = max_lat + offset_lat
        path = [
            start,
            [start[0], bypass_y],
            [end[0], bypass_y],
            end
        ]
    elif direction == "bottom":
        # 下侧绕行：向下绕过所有障碍物
        bypass_y = min_lat - offset_lat
        path = [
            start,
            [start[0], bypass_y],
            [end[0], bypass_y],
            end
        ]
    else:  # auto - 选择最短路径
        # 尝试四个方向，选择最短的有效路径
        candidates = [
            ("left", min_lng - offset_lng, "x"),
            ("right", max_lng + offset_lng, "x"),
            ("top", max_lat + offset_lat, "y"),
            ("bottom", min_lat - offset_lat, "y")
        ]
        
        valid_paths = []
        for name, value, axis in candidates:
            if axis == "x":
                test_path = [start, [value, start[1]], [value, end[1]], end]
            else:
                test_path = [start, [start[0], value], [end[0], value], end]
            
            # 验证路径是否畅通
            path_clear = True
            for i in range(len(test_path) - 1):
                if not is_path_clear(test_path[i], test_path[i+1], relevant_obstacles, flight_altitude):
                    path_clear = False
                    break
            
            if path_clear:
                total_dist = sum(distance(test_path[i], test_path[i+1]) for i in range(len(test_path)-1))
                valid_paths.append((total_dist, test_path, name))
        
        if valid_paths:
            valid_paths.sort(key=lambda x: x[0])
            path = valid_paths[0][1]
        else:
            # 如果简单绕行失败，使用更复杂的路径
            path = [
                start,
                [min_lng - offset_lng, start[1]],
                [min_lng - offset_lng, max_lat + offset_lat],
                [max_lng + offset_lng, max_lat + offset_lat],
                [max_lng + offset_lng, end[1]],
                end
            ]
    
    # 去重：移除连续的重复点
    unique_path = []
    for p in path:
        if not unique_path or distance(unique_path[-1], p) > 1e-8:
            unique_path.append(p)
    
    return unique_path

# ==================== 平滑曲线生成 ====================
def catmull_rom_spline(points, num_segments=15):
    """Catmull-Rom样条曲线平滑"""
    if len(points) < 2:
        return points
    if len(points) == 2:
        # 两点之间直接线性插值
        result = []
        for t in np.linspace(0, 1, num_segments):
            x = points[0][0] + (points[1][0] - points[0][0]) * t
            y = points[0][1] + (points[1][1] - points[0][1]) * t
            result.append([x, y])
        return result
    
    smooth_path = []
    
    for i in range(len(points) - 1):
        p0 = points[max(0, i-1)]
        p1 = points[i]
        p2 = points[min(len(points)-1, i+1)]
        p3 = points[min(len(points)-1, i+2)]
        
        for t in np.linspace(0, 1, num_segments):
            t2 = t * t
            t3 = t2 * t
            
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            
            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            
            smooth_path.append([x, y])
    
    # 添加最后一个点
    smooth_path.append(points[-1])
    
    return smooth_path

def generate_smooth_path(waypoints, smoothness=0.6):
    """生成平滑路径"""
    if len(waypoints) < 2:
        return waypoints
    
    # 根据平滑度决定分段数
    num_segments = max(5, int(15 * smoothness))
    smooth_path = catmull_rom_spline(waypoints, num_segments)
    
    return smooth_path

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
        'version': 'v15.0'
    }
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.smooth_path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        
    def set_path(self, waypoints, altitude=50, speed=50, safety_radius=5, smoothness=0.6):
        # 生成平滑飞行路径
        self.smooth_path = generate_smooth_path(waypoints, smoothness)
        self.path = self.smooth_path
        self.path_index = 0
        self.current_pos = self.path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        
        self.total_distance = 0.0
        for i in range(len(self.path) - 1):
            self.total_distance += distance(self.path[i], self.path[i + 1])
    
    def update_and_generate(self, obstacles_gcj):
        """更新无人机位置并生成心跳包"""
        if not self.simulating or self.path_index >= len(self.path) - 1:
            self.simulating = False
            return None
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = distance(start, end)
        
        base_speed = 20
        speed_m_per_s = base_speed * (self.speed / 100)
        move_distance = speed_m_per_s * 0.2
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            self.progress = self.distance_traveled / self.total_distance
        
        if self.distance_traveled >= segment_distance:
            self.path_index += 1
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
                self.distance_traveled = 0
            else:
                self.simulating = False
                return self._generate_heartbeat(True)
        else:
            t = self.distance_traveled / segment_distance if segment_distance > 0 else 0
            lng = start[0] + (end[0] - start[0]) * t
            lat = start[1] + (end[1] - start[1]) * t
            self.current_pos = [lng, lat]
        
        safe, min_dist, danger = check_safety_radius(
            self.current_pos, obstacles_gcj, self.flight_altitude, self.safety_radius
        )
        if not safe:
            self.safety_violation = True
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived=False):
        heartbeat = {
            'timestamp': datetime.now().strftime("%H:%M:%S"),
            'lat': self.current_pos[1],
            'lng': self.current_pos[0],
            'altitude': self.flight_altitude,
            'voltage': round(22.2 + random.uniform(-0.5, 0.5), 1),
            'satellites': random.randint(8, 14),
            'speed': round(20 * (self.speed / 100), 1),
            'progress': self.progress if self.total_distance > 0 else 0,
            'arrived': arrived,
            'safety_violation': self.safety_violation
        }
        self.history.insert(0, heartbeat)
        if len(self.history) > 100:
            self.history.pop()
        return heartbeat

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_waypoints=None, smooth_path=None, map_type="satellite", straight_blocked=True, flight_altitude=50, drone_pos=None, direction="最佳航线", safety_radius=5):
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=17, tiles=tiles, attr=attr)
    
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    # 绘制障碍物
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        height = obs.get('height', 30)
        if coords and len(coords) >= 3:
            color = "red" if height > flight_altitude else "orange"
            folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=3, fill=True, fill_color=color, fill_opacity=0.4, popup=f"🚧 {obs.get('name')}\n高度: {height}m").add_to(m)
    
    # 绘制起点和终点
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    
    # 绘制平滑飞行路径
    if smooth_path and len(smooth_path) > 1:
        path_locations = [[p[1], p[0]] for p in smooth_path]
        
        if direction == "向左绕行":
            line_color = "purple"
        elif direction == "向右绕行":
            line_color = "orange"
        elif direction == "向上绕行":
            line_color = "cyan"
        elif direction == "向下绕行":
            line_color = "pink"
        else:
            line_color = "green"
        
        folium.PolyLine(path_locations, color=line_color, weight=4, opacity=0.9, 
                       popup=f"✈️ {direction} (平滑曲线)").add_to(m)
    
    # 绘制关键航点
    if planned_waypoints and len(planned_waypoints) > 2:
        for i, point in enumerate(planned_waypoints[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=6, color="yellow", 
                               fill=True, fill_color="yellow", fill_opacity=0.8, 
                               popup=f"关键航点 {i+1}").add_to(m)
    
    # 绘制直线（用于对比）
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], 
                           color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], 
                           color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
    
    # 绘制安全半径
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
    
    # 绘制历史轨迹
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
    if "planned_waypoints" not in st.session_state:
        st.session_state.planned_waypoints = None
    if "smooth_path" not in st.session_state:
        st.session_state.smooth_path = None
    if "last_flight_altitude" not in st.session_state:
        st.session_state.last_flight_altitude = 50
    if "pending_obstacle" not in st.session_state:
        st.session_state.pending_obstacle = None
    if "current_direction" not in st.session_state:
        st.session_state.current_direction = "最佳航线"
    if "safety_radius" not in st.session_state:
        st.session_state.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
    if "smoothness" not in st.session_state:
        st.session_state.smoothness = 0.6
    
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
    st.sidebar.subheader("🛡️ 安全半径设置")
    safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, value=st.session_state.safety_radius, step=1)
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🔄 航线平滑度")
    smoothness = st.sidebar.slider("平滑度", min_value=0.0, max_value=1.0, value=st.session_state.smoothness, step=0.1,
                                   help="0: 折线 | 0.5: 中等平滑 | 1.0: 非常平滑")
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("🧭 绕行方向设置")
    direction_option = st.sidebar.selectbox(
        "绕行方向",
        options=["最佳航线（自动）", "向左绕行", "向右绕行", "向上绕行", "向下绕行"],
        index=0,
        help="选择无人机绕过障碍物的方向"
    )
    
    # 映射方向选项
    direction_map = {
        "最佳航线（自动）": "auto",
        "向左绕行": "left",
        "向右绕行": "right",
        "向上绕行": "top",
        "向下绕行": "bottom"
    }
    selected_direction = direction_map[direction_option]
    
    # 更新显示名称
    display_direction = direction_option.replace("（自动）", "")
    
    if safety_radius != st.session_state.safety_radius:
        st.session_state.safety_radius = safety_radius
    
    need_update = (flight_alt != st.session_state.last_flight_altitude or 
                   smoothness != st.session_state.smoothness or
                   selected_direction != st.session_state.get("selected_direction", "auto"))
    
    if need_update:
        st.session_state.last_flight_altitude = flight_alt
        st.session_state.smoothness = smoothness
        st.session_state.selected_direction = selected_direction
        st.session_state.current_direction = display_direction
        
        # 计算避障路径
        waypoints = find_avoidance_path_multiple_obstacles(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            safety_distance_meters=safety_radius * 3,
            direction=selected_direction
        )
        st.session_state.planned_waypoints = waypoints
        
        # 生成平滑路径
        st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
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
        f"🛡️ 安全半径: {safety_radius} 米\n"
        f"🔄 平滑度: {smoothness:.1f}\n"
        f"🧭 绕行方向: {display_direction}"
    )
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        waypoints = find_avoidance_path_multiple_obstacles(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            flight_alt,
            safety_distance_meters=safety_radius * 3,
            direction=selected_direction
        )
        st.session_state.planned_waypoints = waypoints
        st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障与平滑飞行")
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {high_obstacles} 个障碍物高于飞行高度({flight_alt}m)，已按「{display_direction}」规划绕行路径")
            st.info("✅ 航线已自动避开障碍物，使用Catmull-Rom样条曲线生成平滑飞行轨迹")
        else:
            st.success("✅ 直线航线畅通无阻，可直接飞行")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                waypoints = find_avoidance_path_multiple_obstacles(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    safety_distance_meters=safety_radius * 3,
                    direction=selected_direction
                )
                st.session_state.planned_waypoints = waypoints
                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                st.rerun()
            
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                waypoints = find_avoidance_path_multiple_obstacles(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    safety_distance_meters=safety_radius * 3,
                    direction=selected_direction
                )
                st.session_state.planned_waypoints = waypoints
                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                st.rerun()
            
            st.markdown("#### 🤖 路径规划")
            
            if st.button("🔄 重新规划路径", use_container_width=True):
                waypoints = find_avoidance_path_multiple_obstacles(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    safety_distance_meters=safety_radius * 3,
                    direction=selected_direction
                )
                st.session_state.planned_waypoints = waypoints
                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                waypoint_count = len(waypoints) - 2
                st.success(f"已规划避障路径，共 {waypoint_count} 个关键航点，平滑后 {len(st.session_state.smooth_path)} 个点")
                st.rerun()
            
            st.markdown("#### ✈️ 飞行参数")
            col_metric1, col_metric2 = st.columns(2)
            with col_metric1:
                st.metric("当前飞行高度", f"{flight_alt} m")
                st.metric("速度系数", f"{drone_speed}%")
                st.metric("🛡️ 安全半径", f"{safety_radius} 米")
            with col_metric2:
                st.metric("当前策略", display_direction)
                st.metric("🔄 平滑度", f"{smoothness:.1f}")
                if st.session_state.smooth_path:
                    st.metric("🎯 轨迹点数", len(st.session_state.smooth_path))
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                    if st.session_state.planned_waypoints:
                        st.session_state.heartbeat_sim.set_path(
                            st.session_state.planned_waypoints, 
                            flight_alt, 
                            drone_speed, 
                            safety_radius,
                            smoothness
                        )
                        st.session_state.simulation_running = True
                        st.session_state.flight_history = []
                        st.success(f"🚁 飞行已开始！按「{display_direction}」绕行，平滑轨迹共 {len(st.session_state.smooth_path)} 个航点")
                    else:
                        st.error("请先规划航线")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.session_state.heartbeat_sim.simulating = False
                    st.info("飞行已停止")
            
            st.markdown("### 📍 坐标信息")
            st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
            st.caption(f"📏 直线距离: {dist:.0f} 米")
            
            if st.session_state.smooth_path:
                path_dist = 0
                for i in range(len(st.session_state.smooth_path)-1):
                    p1, p2 = st.session_state.smooth_path[i], st.session_state.smooth_path[i+1]
                    path_dist += math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) * 111000
                st.caption(f"🔄 平滑路径: {path_dist:.0f} 米 ({((path_dist/dist-1)*100):.1f}% 增量)")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            st.caption("🟢 绿色=平滑航线 | 🟡 黄色=关键航点 | 🔴 红色=障碍物 | 🔵 蓝色圆圈=安全半径")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_waypoints is None:
                waypoints = find_avoidance_path_multiple_obstacles(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj,
                    flight_alt,
                    safety_distance_meters=safety_radius * 3,
                    direction=selected_direction
                )
                st.session_state.planned_waypoints = waypoints
                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
            
            drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
            
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, 
                                   flight_trail, st.session_state.planned_waypoints, 
                                   st.session_state.smooth_path, map_type, straight_blocked, 
                                   flight_alt, drone_pos, display_direction, safety_radius)
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
                        waypoints = find_avoidance_path_multiple_obstacles(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            safety_distance_meters=safety_radius * 3,
                            direction=selected_direction
                        )
                        st.session_state.planned_waypoints = waypoints
                        st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                        st.session_state.pending_obstacle = None
                        st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
                        st.rerun()
                with col_cancel:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_obstacle = None
                        st.rerun()
            
            st.caption("💡 **说明**：航线自动避开障碍物，使用Catmull-Rom样条曲线实现平滑转弯")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        st.caption(f"✈️ 飞行高度: {flight_alt} 米 | 🧭 策略: {display_direction} | 🛡️ 安全半径: {safety_radius} 米 | 🔄 平滑度: {smoothness:.1f}")
        
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
            
            if latest.get('safety_violation', False):
                st.error("⚠️ 警告：无人机进入安全半径危险区域！")
            
            progress = latest.get('progress', 0)
            st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            if latest.get('arrived', False):
                st.success("🎉 无人机已到达目的地！")
            
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                height = obs.get('height', 30)
                if coords and len(coords) >= 3:
                    color = "red" if height > flight_alt else "orange"
                    folium.Polygon([[c[1], c[0]] for c in coords], color=color, weight=2, fill=True, fill_opacity=0.3, popup=f"🚧 {obs.get('name')}").add_to(monitor_map)
            
            if st.session_state.smooth_path and len(st.session_state.smooth_path) > 1:
                if display_direction == "向左绕行":
                    line_color = "purple"
                elif display_direction == "向右绕行":
                    line_color = "orange"
                else:
                    line_color = "green"
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.smooth_path], 
                               color=line_color, weight=3, opacity=0.7, popup="规划航线").add_to(monitor_map)
            
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
            
            folium.Marker([latest['lat'], latest['lng']], popup=f"当前位置", 
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                            popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                            popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
            
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
                    waypoints = find_avoidance_path_multiple_obstacles(
                        st.session_state.points_gcj['A'],
                        st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj,
                        flight_alt,
                        safety_distance_meters=safety_radius * 3,
                        direction=selected_direction
                    )
                    st.session_state.planned_waypoints = waypoints
                    st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                    st.success(f"已加载 {len(loaded)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("无配置文件")
        with col_save_load3:
            config_data = {'obstacles': st.session_state.obstacles_gcj, 'count': len(st.session_state.obstacles_gcj), 
                          'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"), 'version': 'v15.0'}
            st.download_button(label="📥 下载配置", data=json.dumps(config_data, ensure_ascii=False, indent=2), 
                             file_name=CONFIG_FILE, mime="application/json", use_container_width=True)
        
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
                                waypoints = find_avoidance_path_multiple_obstacles(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    safety_distance_meters=safety_radius * 3,
                                    direction=selected_direction
                                )
                                st.session_state.planned_waypoints = waypoints
                                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                                st.rerun()
                        with col_del:
                            if st.button("🗑️", key=f"del_{i}"):
                                st.session_state.obstacles_gcj.pop(i)
                                save_obstacles(st.session_state.obstacles_gcj)
                                waypoints = find_avoidance_path_multiple_obstacles(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj,
                                    flight_alt,
                                    safety_distance_meters=safety_radius * 3,
                                    direction=selected_direction
                                )
                                st.session_state.planned_waypoints = waypoints
                                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
                                st.rerun()
            else:
                st.write("暂无障碍物")
            
            if st.button("🗑️ 清除所有障碍物", use_container_width=True):
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                waypoints = find_avoidance_path_multiple_obstacles(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    [],
                    flight_alt,
                    safety_distance_meters=safety_radius * 3,
                    direction=selected_direction
                )
                st.session_state.planned_waypoints = waypoints
                st.session_state.smooth_path = generate_smooth_path(waypoints, smoothness)
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
