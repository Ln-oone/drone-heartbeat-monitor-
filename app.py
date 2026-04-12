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

# ==================== 纯数学路径规划算法 ====================
def point_in_polygon(point, polygon):
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

def segments_intersect(p1, p2, p3, p4):
    """检查两条线段是否相交"""
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))

def line_intersects_polygon(p1, p2, polygon):
    """检查线段是否与多边形相交"""
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

def is_straight_path_blocked(start, end, obstacles_gcj):
    """检查直线路径是否被障碍物阻挡"""
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            if line_intersects_polygon(start, end, coords):
                return True
    return False

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_polygon_vertices(obstacles_gcj):
    """获取所有障碍物的顶点"""
    vertices = []
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            for pt in coords:
                vertices.append([pt[0], pt[1]])
                offset = 0.00005
                vertices.append([pt[0] + offset, pt[1]])
                vertices.append([pt[0] - offset, pt[1]])
                vertices.append([pt[0], pt[1] + offset])
                vertices.append([pt[0], pt[1] - offset])
    return vertices

def find_shortest_path(start, end, obstacles_gcj):
    """简化版路径规划：沿障碍物边缘绕行"""
    if not obstacles_gcj:
        return [start, end]
    
    if not is_straight_path_blocked(start, end, obstacles_gcj):
        return [start, end]
    
    vertices = [start, end]
    vertices.extend(get_polygon_vertices(obstacles_gcj))
    
    # 去重
    unique = []
    for v in vertices:
        found = False
        for u in unique:
            if abs(v[0]-u[0]) < 0.00001 and abs(v[1]-u[1]) < 0.00001:
                found = True
                break
        if not found:
            unique.append(v)
    
    best_path = [start, end]
    best_dist = distance(start, end)
    
    # 尝试单顶点绕行
    for v in unique:
        if v == start or v == end:
            continue
        if not is_straight_path_blocked(start, v, obstacles_gcj) and not is_straight_path_blocked(v, end, obstacles_gcj):
            path_dist = distance(start, v) + distance(v, end)
            if path_dist < best_dist:
                best_dist = path_dist
                best_path = [start, v, end]
    
    # 尝试双顶点绕行
    for i, v1 in enumerate(unique):
        if v1 == start or v1 == end:
            continue
        for j, v2 in enumerate(unique):
            if v2 == start or v2 == end or i == j:
                continue
            if (not is_straight_path_blocked(start, v1, obstacles_gcj) and
                not is_straight_path_blocked(v1, v2, obstacles_gcj) and
                not is_straight_path_blocked(v2, end, obstacles_gcj)):
                path_dist = distance(start, v1) + distance(v1, v2) + distance(v2, end)
                if path_dist < best_dist:
                    best_dist = path_dist
                    best_path = [start, v1, v2, end]
    
    return best_path

def create_avoidance_path(start, end, obstacles_gcj):
    return find_shortest_path(start, end, obstacles_gcj)

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
        
    def set_path(self, path, altitude=50, speed=50):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        self.progress = 0.0
        
    def generate(self):
        if not self.simulating:
            altitude = random.randint(0, 10)
            speed_display = 0
            self.progress = 0.0
        else:
            if self.path_index < len(self.path) - 1:
                target = self.path[self.path_index + 1]
                dx = target[0] - self.current_pos[0]
                dy = target[1] - self.current_pos[1]
                dist_to_target = math.sqrt(dx*dx + dy*dy)
                
                if dist_to_target < 0.000005:
                    self.path_index += 1
                else:
                    # 根据速度移动
                    step_scale = self.speed / 25
                    step = max(0.00001, min(0.0005, step_scale * 0.0001))
                    if dist_to_target < step:
                        self.current_pos = target.copy()
                        self.path_index += 1
                    else:
                        ratio = step / dist_to_target
                        self.current_pos[0] += dx * ratio
                        self.current_pos[1] += dy * ratio
                
                # 计算进度
                total_waypoints = len(self.path)
                self.progress = self.path_index / (total_waypoints - 1) if total_waypoints > 1 else 0
                altitude = self.flight_altitude + random.randint(-5, 5)
                speed_display = round(self.speed * 0.15, 1)
            else:
                self.simulating = False
                altitude = random.randint(0, 10)
                speed_display = 0
                self.progress = 1.0
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": speed_display,
            "progress": self.progress,
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, map_type="satellite", straight_blocked=True):
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
        if coords and len(coords) >= 3:
            folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.4, popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}").add_to(m)
    
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="📚 图书馆 (起点)", icon=folium.Icon(color="green", icon="book", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🍽️ 食堂 (终点)", icon=folium.Icon(color="red", icon="utensils", prefix="fa")).add_to(m)
    
    if planned_path and len(planned_path) > 1:
        folium.PolyLine([[p[1], p[0]] for p in planned_path], color="green", weight=5, opacity=0.9, popup="✈️ 智能避障航线").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=5, color="green", fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=3, opacity=0.6, dash_array='5, 5', popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
    
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
    
    # 侧边栏
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"
    
    st.sidebar.markdown("---")
    st.sidebar.subheader("⚡ 无人机速度设置")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5, help="数值越大，飞行越快")
    
    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    straight_blocked = is_straight_path_blocked(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
    st.sidebar.info(f"🏫 南京科技职业学院\n📚 图书馆\n🍽️ 食堂\n🚧 障碍物: {obs_count}\n📌 直线: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}")
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        
        straight_blocked = is_straight_path_blocked(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
        if straight_blocked:
            st.warning("⚠️ 直线航线被障碍物阻挡！已自动规划避障航线（绿色）")
        else:
            st.success("✅ 直线航线畅通无阻")
        
        st.info("📝 点击地图左上角📐图标 → 选择多边形 → 围绕建筑物绘制 → 双击完成")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            st.markdown("#### 📚 起点 A: 图书馆")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                st.rerun()
            
            st.markdown("#### 🍽️ 终点 B: 食堂")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                st.rerun()
            
            st.markdown("#### 🤖 路径规划")
            if st.button("🔄 重新规划路径", use_container_width=True):
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                st.success(f"已规划 {len(st.session_state.planned_path)} 个航点")
                st.rerun()
            
            st.markdown("#### ✈️ 飞行参数")
            flight_alt = st.number_input("飞行高度 (m)", value=50, step=5, min_value=10, max_value=200)
            st.session_state.flight_altitude = flight_alt
            st.metric("速度系数", f"{drone_speed}%")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, st.session_state.flight_altitude, drone_speed)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []  # 清空旧轨迹
                    st.success("🚁 飞行已开始！请切换到「飞行监控」页面查看进度")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.info("飞行已停止")
            
            st.markdown("### 📍 当前航点")
            st.write(f"📚 图书馆: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
            st.write(f"🍽️ 食堂: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
            
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
                st.caption("🟢 绿色 = 避障航线 | ⚪ 灰色虚线 = 直线（被阻挡）")
            else:
                st.caption("✅ 直线畅通")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
            
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail, st.session_state.planned_path, map_type, straight_blocked)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                    coords = last["geometry"].get("coordinates", [])
                    if coords and len(coords) > 0:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3:
                            st.session_state.obstacles_gcj.append({"name": f"建筑物{len(st.session_state.obstacles_gcj)+1}", "polygon": poly})
                            save_obstacles(st.session_state.obstacles_gcj)
                            st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                            st.rerun()
            
            st.caption("📌 图例：🟢 绿色=避障航线 | 🔴 红色=障碍物 | 📚 图书馆 | 🍽️ 食堂")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        # 自动更新心跳（每秒一次）
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 0.5:  # 0.5秒更新一次，更流畅
            if st.session_state.simulation_running:
                new_hb = st.session_state.heartbeat_sim.generate()
                st.session_state.last_hb_time = current_time
                # 记录轨迹
                st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
                if len(st.session_state.flight_history) > 200:
                    st.session_state.flight_history.pop(0)
                st.rerun()
            else:
                st.session_state.last_hb_time = current_time
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            # 指标卡片
            c1, c2, c3, c4, c5, c6 = st.columns(6)
            c1.metric("⏰ 时间", latest['timestamp'])
            c2.metric("📍 纬度", f"{latest['lat']:.6f}")
            c3.metric("📍 经度", f"{latest['lng']:.6f}")
            c4.metric("📊 高度", f"{latest['altitude']} m")
            c5.metric("🔋 电压", f"{latest['voltage']} V")
            c6.metric("🛰️ 卫星", latest['satellites'])
            
            c7, c8 = st.columns(2)
            c7.metric("💨 速度", f"{latest.get('speed', 0)} m/s")
            c8.metric("⚡ 速度系数", f"{drone_speed}%")
            
            # 飞行进度条
            progress = latest.get('progress', 0)
            st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            # 显示当前航点信息
            if st.session_state.heartbeat_sim.path and len(st.session_state.heartbeat_sim.path) > 1:
                current_idx = st.session_state.heartbeat_sim.path_index
                total_waypoints = len(st.session_state.heartbeat_sim.path)
                st.caption(f"📍 当前航点: {current_idx + 1} / {total_waypoints}")
            
            # 实时位置地图
            st.subheader("📍 实时位置")
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            mon_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=17, tiles=tiles, attr="高德地图")
            
            # 障碍物
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=2, fill=True, fill_opacity=0.3).add_to(mon_map)
            
            # 规划路径
            if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
                folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], color="green", weight=3, opacity=0.7, popup="规划航线").add_to(mon_map)
            
            # 历史轨迹
            trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.7, popup="历史轨迹").add_to(mon_map)
            
            # 当前位置
            folium.Marker([latest['lat'], latest['lng']], popup=f"📍 当前位置\n高度: {latest['altitude']}m", 
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(mon_map)
            
            # 起点终点
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                             popup="📚 图书馆", icon=folium.Icon(color='green', icon='book', prefix='fa')).add_to(mon_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                             popup="🍽️ 食堂", icon=folium.Icon(color='blue', icon='utensils', prefix='fa')).add_to(mon_map)
            
            folium_static(mon_map, width=800, height=400)
            
            # 数据图表
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
            
            # 历史记录表格
            st.subheader("📋 历史心跳记录")
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_sim.history[:10]), use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        # 控制按钮
        col_r, col_c = st.columns(2)
        with col_r:
            if st.button("🔄 立即刷新", use_container_width=True):
                if st.session_state.simulation_running:
                    new_hb = st.session_state.heartbeat_sim.generate()
                    st.rerun()
        with col_c:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_sim.history = []
                st.rerun()
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        st.info(f"当前共 **{len(st.session_state.obstacles_gcj)}** 个障碍物\n\n在「航线规划」页面圈选添加，或点击下方删除")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            if st.session_state.obstacles_gcj:
                for i, obs in enumerate(st.session_state.obstacles_gcj):
                    col_n, col_b = st.columns([3, 1])
                    col_n.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                    if col_b.button("删除", key=f"del_{i}"):
                        st.session_state.obstacles_gcj.pop(i)
                        save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                        st.rerun()
            else:
                st.write("暂无障碍物")
            
            st.markdown("---")
            col_s, col_l = st.columns(2)
            if col_s.button("💾 保存", use_container_width=True):
                save_obstacles(st.session_state.obstacles_gcj)
                st.success("已保存")
            if col_l.button("📂 加载", use_container_width=True):
                st.session_state.obstacles_gcj = load_obstacles()
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], st.session_state.obstacles_gcj)
                st.rerun()
            
            col_cl, col_d = st.columns(2)
            if col_cl.button("🗑️ 全部清除", use_container_width=True):
                st.session_state.obstacles_gcj = []
                save_obstacles([])
                st.session_state.planned_path = create_avoidance_path(st.session_state.points_gcj['A'], st.session_state.points_gcj['B'], [])
                st.rerun()
        
        with col2:
            tiles = GAODE_SATELLITE_URL if map_type == "satellite" else GAODE_VECTOR_URL
            obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], zoom_start=16, tiles=tiles, attr="高德地图")
            for obs in st.session_state.obstacles_gcj:
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_opacity=0.5).add_to(obs_map)
            folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], popup="图书馆", icon=folium.Icon(color='green', icon='book', prefix='fa')).add_to(obs_map)
            folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], popup="食堂", icon=folium.Icon(color='red', icon='utensils', prefix='fa')).add_to(obs_map)
            folium_static(obs_map, width=700, height=500)

if __name__ == "__main__":
    main()
