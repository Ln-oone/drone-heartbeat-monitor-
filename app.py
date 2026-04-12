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
from shapely.geometry import Point, Polygon, LineString
import heapq

# ==================== 页面配置 ====================
st.set_page_config(page_title="南京科技职业学院 - 无人机地面站系统", layout="wide")

# ==================== 坐标 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]  # 图书馆
DEFAULT_B_GCJ = [118.751589, 32.235204]  # 食堂

CONFIG_FILE = "obstacle_config.json"

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

# ==================== 增强版路径规划算法 ====================
def line_intersects_polygon(line_start, line_end, polygon_coords):
    """检查线段是否与多边形相交（不包含相切）"""
    line = LineString([(line_start[0], line_start[1]), (line_end[0], line_end[1])])
    poly = Polygon(polygon_coords)
    
    if line.intersects(poly):
        # 检查是否只是相切
        if line.touches(poly):
            return False
        # 检查线段中点是否在多边形内部
        mid_x = (line_start[0] + line_end[0]) / 2
        mid_y = (line_start[1] + line_end[1]) / 2
        if poly.contains(Point(mid_x, mid_y)):
            return True
        # 检查端点是否在多边形内部
        if poly.contains(Point(line_start[0], line_start[1])) or poly.contains(Point(line_end[0], line_end[1])):
            return True
    return False

def is_straight_path_blocked(start, end, obstacles_gcj):
    """检查直线路径是否被障碍物阻挡"""
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            if line_intersects_polygon(start, end, coords):
                return True
    return False

def get_polygon_vertices_with_offset(obstacles_gcj, offset_meters=15):
    """获取障碍物顶点并向外偏移"""
    vertices = []
    lat_scale = 111000
    lng_scale = 111000 * math.cos(math.radians(32.23))  # 南京纬度
    
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            # 添加原始顶点
            for pt in coords:
                vertices.append([pt[0], pt[1]])
            
            # 添加偏移后的顶点（向外扩展）
            offset_deg = offset_meters / lng_scale
            for pt in coords:
                vertices.append([pt[0] + offset_deg, pt[1]])
                vertices.append([pt[0] - offset_deg, pt[1]])
                vertices.append([pt[0], pt[1] + offset_deg])
                vertices.append([pt[0], pt[1] - offset_deg])
                vertices.append([pt[0] + offset_deg, pt[1] + offset_deg])
                vertices.append([pt[0] - offset_deg, pt[1] - offset_deg])
    
    return vertices

def line_of_sight(p1, p2, obstacles_gcj):
    """检查两点之间是否有视线（不被障碍物阻挡）"""
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            if line_intersects_polygon(p1, p2, coords):
                return False
    return True

def distance(p1, p2):
    """计算两点之间的欧氏距离（度）"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def find_shortest_path_astar(start, end, obstacles_gcj):
    """A* 算法寻找最短避障路径"""
    
    if not obstacles_gcj:
        return [start, end]
    
    # 检查直线是否可用
    if not is_straight_path_blocked(start, end, obstacles_gcj):
        return [start, end]
    
    # 获取所有顶点作为潜在路径点
    vertices = [start, end]
    vertices.extend(get_polygon_vertices_with_offset(obstacles_gcj, 20))
    
    # 去重
    unique_vertices = []
    for v in vertices:
        is_unique = True
        for u in unique_vertices:
            if abs(v[0]-u[0]) < 0.00001 and abs(v[1]-u[1]) < 0.00001:
                is_unique = False
                break
        if is_unique:
            unique_vertices.append(v)
    
    # 构建可见性图
    graph = {}
    n = len(unique_vertices)
    
    for i in range(n):
        graph[i] = []
        for j in range(n):
            if i == j:
                continue
            
            # 检查两点之间是否有视线
            if line_of_sight(unique_vertices[i], unique_vertices[j], obstacles_gcj):
                dist = distance(unique_vertices[i], unique_vertices[j])
                graph[i].append((j, dist))
    
    # 找到起点和终点的索引
    start_idx = None
    end_idx = None
    for i, v in enumerate(unique_vertices):
        if abs(v[0]-start[0]) < 0.00001 and abs(v[1]-start[1]) < 0.00001:
            start_idx = i
        if abs(v[0]-end[0]) < 0.00001 and abs(v[1]-end[1]) < 0.00001:
            end_idx = i
    
    if start_idx is None or end_idx is None:
        return [start, end]
    
    # A* 搜索
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}
    g_score = {i: float('inf') for i in range(n)}
    g_score[start_idx] = 0
    f_score = {i: float('inf') for i in range(n)}
    f_score[start_idx] = distance(unique_vertices[start_idx], unique_vertices[end_idx])
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == end_idx:
            # 重构路径
            path = []
            while current in came_from:
                path.append(unique_vertices[current])
                current = came_from[current]
            path.append(unique_vertices[start_idx])
            path.reverse()
            return path
        
        for neighbor, dist in graph.get(current, []):
            tentative_g = g_score[current] + dist
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + distance(unique_vertices[neighbor], unique_vertices[end_idx])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # 没找到路径，返回直线
    return [start, end]

def create_avoidance_path(start, end, obstacles_gcj):
    """创建避开障碍物的路径"""
    return find_shortest_path_astar(start, end, obstacles_gcj)

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
        
    def set_path(self, path, altitude=50, speed=50):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        
    def generate(self):
        if self.simulating and self.path_index < len(self.path) - 1:
            target = self.path[self.path_index + 1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.000005:
                self.path_index += 1
            else:
                step_scale = self.speed / 20
                step_x = dx / max(20, 100 - step_scale)
                step_y = dy / max(20, 100 - step_scale)
                self.current_pos[0] += step_x
                self.current_pos[1] += step_y
        elif self.path_index >= len(self.path) - 1:
            self.simulating = False
        
        if not self.simulating:
            altitude = random.randint(0, 10)
            speed_display = 0
        else:
            altitude = self.flight_altitude + random.randint(-5, 5)
            speed_display = round(self.speed * 0.3, 1)
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": speed_display,
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, map_type="satellite", straight_path_blocked=True):
    """创建规划地图"""
    
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    
    m = folium.Map(
        location=[center_gcj[1], center_gcj[0]],
        zoom_start=16,
        tiles=tiles,
        attr=attr
    )
    
    # 绘图控件
    draw = plugins.Draw(
        export=True,
        position='topleft',
        draw_options={
            'polygon': {
                'allowIntersection': False,
                'showArea': True,
                'color': '#ff0000',
                'fillColor': '#ff0000',
                'fillOpacity': 0.4
            },
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': False,
            'circlemarker': False
        },
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    
    # 显示障碍物
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            locations = [[c[1], c[0]] for c in coords]
            folium.Polygon(
                locations=locations,
                color="red",
                weight=3,
                fill=True,
                fill_color="red",
                fill_opacity=0.4,
                popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}"
            ).add_to(m)
    
    # 起点
    if points_gcj.get('A'):
        folium.Marker(
            [points_gcj['A'][1], points_gcj['A'][0]],
            popup="📚 图书馆 (起点)",
            icon=folium.Icon(color="green", icon="book", prefix="fa")
        ).add_to(m)
    
    # 终点
    if points_gcj.get('B'):
        folium.Marker(
            [points_gcj['B'][1], points_gcj['B'][0]],
            popup="🍽️ 食堂 (终点)",
            icon=folium.Icon(color="red", icon="utensils", prefix="fa")
        ).add_to(m)
    
    # 智能避障航线（绿色）
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        folium.PolyLine(
            path_locations,
            color="green",
            weight=5,
            opacity=0.9,
            popup="✈️ 智能避障航线（无人机实际航线）",
            tooltip="无人机将沿此路线绕过建筑物"
        ).add_to(m)
        
        # 添加航点标记
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker(
                [point[1], point[0]],
                radius=5,
                color="green",
                fill=True,
                fill_color="white",
                fill_opacity=0.8,
                popup=f"航点 {i+1}"
            ).add_to(m)
    
    # 直线航线（仅当未被阻挡时显示，作为参考）
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_path_blocked:
            # 直线路径可用，显示为虚线
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], 
                 [points_gcj['B'][1], points_gcj['B'][0]]],
                color="blue",
                weight=3,
                opacity=0.6,
                dash_array='5, 5',
                popup="直线航线（无阻碍）"
            ).add_to(m)
        else:
            # 直线路径被阻挡，显示为红色虚线警告
            folium.PolyLine(
                [[points_gcj['A'][1], points_gcj['A'][0]], 
                 [points_gcj['B'][1], points_gcj['B'][0]]],
                color="gray",
                weight=2,
                opacity=0.4,
                dash_array='5, 5',
                popup="⚠️ 直线航线（被障碍物阻挡，不可用）"
            ).add_to(m)
    
    # 飞行轨迹
    if flight_history and len(flight_history) > 1:
        trail_locations = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail_locations) > 1:
            folium.PolyLine(
                trail_locations,
                color="orange",
                weight=2,
                opacity=0.6,
                popup="历史轨迹"
            ).add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.title("🏫 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    # ==================== 初始化 ====================
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {
            'A': DEFAULT_A_GCJ.copy(),
            'B': DEFAULT_B_GCJ.copy()
        }
    
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = load_obstacles()
    
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    if "planned_path" not in st.session_state:
        st.session_state.planned_path = None
    if "straight_blocked" not in st.session_state:
        st.session_state.straight_blocked = False
    
    # ==================== 侧边栏 ====================
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio(
        "选择功能模块",
        ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"]
    )
    
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider(
        "飞行速度系数", 
        min_value=10, 
        max_value=100, 
        value=50,
        step=5
    )
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    straight_blocked = is_straight_path_blocked(
        st.session_state.points_gcj['A'],
        st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj
    )
    st.sidebar.info(
        f"🏫 南京科技职业学院\n"
        f"📚 起点: 图书馆\n"
        f"🍽️ 终点: 食堂\n"
        f"🚧 障碍物: {obs_count}\n"
        f"📌 直线路径: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}\n"
        f"🤖 智能避障: {'已启用' if straight_blocked else '无需绕行'}"
    )
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        
        # 检查直线是否被阻挡
        straight_blocked = is_straight_path_blocked(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj
        )
        
        if straight_blocked:
            st.warning("⚠️ **直线航线被障碍物阻挡！** 已自动规划避障航线（绿色）")
        else:
            st.success("✅ **直线航线畅通无阻** 无需绕行")
        
        st.info("""
        ### 📝 操作说明：
        1. **设置起点/终点**：左侧输入框修改坐标
        2. **圈选障碍物**：点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成
        3. **自动避障**：系统自动计算最短避障路径，**绿色线为实际航线**
        """)
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            st.markdown("#### 📚 起点 A: 图书馆")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj
                )
                st.rerun()
            
            st.markdown("---")
            st.markdown("#### 🍽️ 终点 B: 食堂")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj
                )
                st.rerun()
            
            st.markdown("---")
            st.markdown("#### 🤖 路径规划")
            if st.button("🔄 重新规划路径", use_container_width=True):
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj
                )
                if st.session_state.planned_path:
                    st.success(f"✅ 已规划路径，共 {len(st.session_state.planned_path)} 个航点")
                st.rerun()
            
            st.markdown("---")
            st.markdown("#### ✈️ 飞行参数")
            flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=10, max_value=200)
            st.session_state.flight_altitude = flight_alt
            st.metric("⚡ 速度系数", f"{drone_speed}%")
            
            st.markdown("---")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    if st.session_state.planned_path:
                        st.session_state.heartbeat_sim.set_path(
                            st.session_state.planned_path,
                            st.session_state.flight_altitude,
                            drone_speed
                        )
                    else:
                        straight = [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                        st.session_state.heartbeat_sim.set_path(straight, st.session_state.flight_altitude, drone_speed)
                    st.session_state.simulation_running = True
                    st.success("🚁 飞行已开始！")
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.info("飞行已停止")
            
            st.markdown("---")
            st.markdown("### 📍 当前航点")
            st.write(f"📚 图书馆: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🍽️ 食堂: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
            a, b = st.session_state.points_gcj['A'], st.session_state.points_gcj['B']
            dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
            st.caption(f"📏 直线距离: {dist:.0f} 米")
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 2:
                path_dist = 0
                for i in range(len(st.session_state.planned_path) - 1):
                    p1, p2 = st.session_state.planned_path[i], st.session_state.planned_path[i+1]
                    path_dist += math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) * 111000
                st.caption(f"🔄 避障路径: {path_dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            if straight_blocked:
                st.caption("🟢 **绿色线 = 智能避障航线（无人机实际飞行路径）**")
                st.caption("⚪ 灰色虚线 = 直线航线（被障碍物阻挡，不可用）")
            else:
                st.caption("✅ **直线航线畅通** 无人机将沿蓝色线飞行")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj
                )
            
            m = create_planning_map(
                center, 
                st.session_state.points_gcj, 
                st.session_state.obstacles_gcj, 
                flight_trail,
                st.session_state.planned_path,
                map_type,
                straight_blocked
            )
            
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
            if output and output.get("last_active_drawing"):
                last_draw = output["last_active_drawing"]
                if last_draw and last_draw.get("geometry"):
                    geom = last_draw["geometry"]
                    if geom.get("type") == "Polygon":
                        coords = geom.get("coordinates", [])
                        if coords and len(coords) > 0:
                            polygon_coords = [[point[0], point[1]] for point in coords[0]]
                            if len(polygon_coords) >= 3:
                                new_name = f"建筑物{len(st.session_state.obstacles_gcj) + 1}"
                                st.session_state.obstacles_gcj.append({"name": new_name, "polygon": polygon_coords})
                                save_obstacles(st.session_state.obstacles_gcj)
                                st.session_state.planned_path = create_avoidance_path(
                                    st.session_state.points_gcj['A'],
                                    st.session_state.points_gcj['B'],
                                    st.session_state.obstacles_gcj
                                )
                                st.rerun()
            
            st.caption("📌 **图例**：🟢 绿色=智能避障航线 | 🔴 红色区域=障碍物 | 📚 图书馆 | 🍽️ 食堂")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 1 and st.session_state.simulation_running:
            st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
            st.session_state.flight_history.append([st.session_state.heartbeat_sim.current_pos[0], 
                                                     st.session_state.heartbeat_sim.current_pos[1]])
            if len(st.session_state.flight_history) > 100:
                st.session_state.flight_history.pop(0)
            st.rerun()
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            c1, c2, c3, c4, c5, c6 = st.columns(6)
            c1.metric("时间", latest['timestamp'])
            c2.metric("纬度", f"{latest['lat']:.6f}")
            c3.metric("经度", f"{latest['lng']:.6f}")
            c4.metric("高度", f"{latest['altitude']} m")
            c5.metric("电压", f"{latest['voltage']} V")
            c6.metric("卫星", latest['satellites'])
            
            c7, c8 = st.columns(2)
            c7.metric("速度", f"{latest.get('speed', 0)} m/s")
            c8.metric("速度系数", f"{drone_speed}%")
            
            if st.session_state.heartbeat_sim.path and len(st.session_state.heartbeat_sim.path) > 1:
                idx = st.session_state.heartbeat_sim.path_index
                total = len(st.session_state.heartbeat_sim.path)
                prog = idx / (total - 1) if total > 1 else 0
                st.progress(prog, text=f"航线进度: {prog*100:.1f}% ({idx+1}/{total} 航点)")
            
            st.subheader("📍 实时位置")
            
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            mon_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles)
            
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=2, fill=True, fill_opacity=0.3).add_to(mon_map)
            
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3, opacity=0.7).add_to(mon_map)
            
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30]]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2).add_to(mon_map)
            
            folium.Marker([latest['lat'], latest['lng']], popup=f"当前位置", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(mon_map)
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], popup="图书馆", icon=folium.Icon(color='green', icon='book', prefix='fa')).add_to(mon_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], popup="食堂", icon=folium.Icon(color='blue', icon='utensils', prefix='fa')).add_to(mon_map)
            
            folium_static(mon_map, width=800, height=400)
            
            st.subheader("📈 数据图表")
            if len(st.session_state.heartbeat_sim.history) > 1:
                df_alt = pd.DataFrame({"序号": range(20), "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]})
                st.line_chart(df_alt, x="序号", y="高度(m)")
            
            st.subheader("📋 历史记录")
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_sim.history[:10]), use_container_width=True)
        else:
            st.info("等待心跳数据... 请在「航线规划」页面开始飞行")
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        obs_list = st.session_state.obstacles_gcj
        st.info(f"当前共 **{len(obs_list)}** 个障碍物\n\n在「航线规划」页面圈选添加，或点击下方删除")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            if obs_list:
                for i, obs in enumerate(obs_list):
                    col_n, col_b = st.columns([3, 1])
                    col_n.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                    if col_b.button("删除", key=f"del_{i}"):
                        st.session_state.obstacles_gcj.pop(i)
                        save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj
                        )
                        st.rerun()
            else:
                st.write("暂无障碍物")
            
            st.markdown("---")
            col_s, col_l = st.columns(2)
            if col_s.button("💾 保存", use_container_width=True):
                save_obstacles(st.session_state.obstacles_gcj)
            if col_l.button("📂 加载", use_container_width=True):
                st.session_state.obstacles_gcj = load_obstacles()
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj
                )
                st.rerun()
            
            col_c, col_d = st.columns(2)
            if col_c.button("🗑️ 全部清除", use_container_width=True):
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'],
                    st.session_state.points_gcj['B'],
                    []
                )
                st.rerun()
        
        with col2:
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles)
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_opacity=0.5).add_to(obs_map)
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="图书馆", icon=folium.Icon(color='green', icon='book', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="食堂", icon=folium.Icon(color='red', icon='utensils', prefix='fa')).add_to(obs_map)
            folium_static(obs_map, width=700, height=500)

if __name__ == "__main__":
    main()
