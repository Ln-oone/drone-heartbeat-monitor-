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

# 高德卫星地图瓦片
GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"

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
        self.target_pos = None
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        
    def set_target(self, target_gcj, altitude=50, speed=50):
        self.target_pos = target_gcj
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        
    def generate(self):
        if self.simulating and self.target_pos:
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.00001:
                self.simulating = False
            else:
                step_scale = self.speed / 20
                step_x = dx / max(20, 100 - step_scale)
                step_y = dy / max(20, 100 - step_scale)
                self.current_pos[0] += step_x
                self.current_pos[1] += step_y
        
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
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None):
    m = folium.Map(
        location=[center_gcj[1], center_gcj[0]],
        zoom_start=16,
        tiles=GAODE_SATELLITE_URL,
        attr="高德卫星地图"
    )
    
    # 绘图控件 - 强制红色
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
    
    # 显示已保存的障碍物（红色）
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
    
    # 起点 - 图书馆
    if points_gcj.get('A'):
        folium.Marker(
            [points_gcj['A'][1], points_gcj['A'][0]],
            popup="📚 图书馆 (起点)",
            icon=folium.Icon(color="green", icon="book", prefix="fa")
        ).add_to(m)
    
    # 终点 - 食堂
    if points_gcj.get('B'):
        folium.Marker(
            [points_gcj['B'][1], points_gcj['B'][0]],
            popup="🍽️ 食堂 (终点)",
            icon=folium.Icon(color="red", icon="utensils", prefix="fa")
        ).add_to(m)
    
    # 规划航线
    if points_gcj.get('A') and points_gcj.get('B'):
        folium.PolyLine(
            [[points_gcj['A'][1], points_gcj['A'][0]], 
             [points_gcj['B'][1], points_gcj['B'][0]]],
            color="blue",
            weight=4,
            opacity=0.8,
            popup="图书馆 → 食堂"
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
    
    # ==================== 侧边栏 ====================
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio(
        "选择功能模块",
        ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"]
    )
    
    map_type = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量地图"], index=0)
    
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
    st.sidebar.info(
        f"🏫 南京科技职业学院\n"
        f"📚 起点: 图书馆\n"
        f"🍽️ 终点: 食堂\n"
        f"🚧 **障碍物数量: {obs_count}**"
    )
    
    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.obstacles_gcj = load_obstacles()
        st.rerun()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        st.info("""
        ### 📝 操作说明：
        1. **设置起点/终点**：左侧输入框修改图书馆/食堂坐标，点击按钮保存
        2. **圈选障碍物**：点击地图左上角的 **📐 图标** → 选择 **多边形** → 在地图上依次点击圈出建筑物轮廓 → **双击完成**
        3. **自动保存**：绘制完成后障碍物会自动保存，侧边栏数字会立即更新
        """)
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            st.markdown("#### 📚 起点 A: 图书馆")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点 (图书馆)", use_container_width=True, type="primary"):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.heartbeat_sim.current_pos = [a_lng, a_lat]
                st.success(f"✅ 起点已设置")
            
            st.markdown("---")
            st.markdown("#### 🍽️ 终点 B: 食堂")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点 (食堂)", use_container_width=True, type="primary"):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.success(f"✅ 终点已设置")
            
            st.markdown("---")
            st.markdown("#### ✈️ 飞行参数")
            flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5, min_value=10, max_value=200)
            st.session_state.flight_altitude = flight_alt
            st.metric("⚡ 当前速度系数", f"{drone_speed}%")
            
            st.markdown("---")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    target = st.session_state.points_gcj['B']
                    st.session_state.heartbeat_sim.set_target(
                        [target[0], target[1]], 
                        st.session_state.flight_altitude,
                        drone_speed
                    )
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
            
            a = st.session_state.points_gcj['A']
            b = st.session_state.points_gcj['B']
            dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 111000
            st.caption(f"📏 航线距离: 约 {dist:.0f} 米")
        
        with col2:
            st.subheader("🗺️ 规划地图")
            st.caption("✏️ 点击左上角📐图标 → 选择多边形 → 围绕建筑物依次点击 → 双击完成")
            
            flight_trail = [[hb['lng'], hb['lat']] for hb in st.session_state.heartbeat_sim.history[:20]]
            center = st.session_state.points_gcj['A'] if st.session_state.points_gcj['A'] else SCHOOL_CENTER_GCJ
            
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj, flight_trail)
            
            # 关键：使用 last_active_drawing 获取最新绘制的图形
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            
            # 处理新绘制的多边形 - 自动保存
            if output and output.get("last_active_drawing"):
                last_draw = output["last_active_drawing"]
                if last_draw and last_draw.get("geometry"):
                    geometry = last_draw["geometry"]
                    if geometry.get("type") == "Polygon":
                        coords = geometry.get("coordinates", [])
                        if coords and len(coords) > 0:
                            polygon_coords = []
                            for point in coords[0]:
                                polygon_coords.append([point[0], point[1]])
                            if len(polygon_coords) >= 3:
                                new_name = f"建筑物{len(st.session_state.obstacles_gcj) + 1}"
                                st.session_state.obstacles_gcj.append({
                                    "name": new_name,
                                    "polygon": polygon_coords
                                })
                                save_obstacles(st.session_state.obstacles_gcj)
                                st.success(f"✅ 已添加 {new_name}，当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
                                st.rerun()
            
            st.caption("📌 **图例**：📚 绿色=图书馆 | 🍽️ 红色=食堂 | 🔴 红色区域=障碍物 | 🔵 蓝色线=规划航线")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 1 and st.session_state.simulation_running:
            new_hb = st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
            st.session_state.flight_history.append([new_hb['lng'], new_hb['lat']])
            if len(st.session_state.flight_history) > 100:
                st.session_state.flight_history.pop(0)
            st.rerun()
        
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            with col1:
                st.metric("⏰ 时间", latest['timestamp'])
            with col2:
                st.metric("📍 纬度", f"{latest['lat']:.6f}")
            with col3:
                st.metric("📍 经度", f"{latest['lng']:.6f}")
            with col4:
                st.metric("📊 高度", f"{latest['altitude']} m")
            with col5:
                st.metric("🔋 电压", f"{latest['voltage']} V")
            with col6:
                st.metric("🛰️ 卫星", latest['satellites'])
            
            col7, col8 = st.columns(2)
            with col7:
                st.metric("💨 当前速度", f"{latest.get('speed', 0)} m/s")
            with col8:
                st.metric("⚡ 速度系数", f"{drone_speed}%")
            
            if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                a = st.session_state.points_gcj['A']
                b = st.session_state.points_gcj['B']
                current = [latest['lng'], latest['lat']]
                total_dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
                current_dist = math.sqrt((current[0]-a[0])**2 + (current[1]-a[1])**2)
                if total_dist > 0:
                    progress = min(1.0, current_dist / total_dist)
                    st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            st.subheader("📍 实时位置")
            monitor_map = folium.Map(
                location=[latest['lat'], latest['lng']],
                zoom_start=17,
                tiles=GAODE_SATELLITE_URL,
                attr="高德卫星地图"
            )
            
            for i, obs in enumerate(st.session_state.obstacles_gcj):
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    locations = [[c[1], c[0]] for c in coords]
                    folium.Polygon(
                        locations=locations,
                        color="red",
                        weight=2,
                        fill=True,
                        fill_color="red",
                        fill_opacity=0.3,
                        popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}"
                    ).add_to(monitor_map)
            
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30] if hb.get('lat') and hb.get('lng')]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="orange", weight=3, opacity=0.7).add_to(monitor_map)
            
            folium.Marker(
                [latest['lat'], latest['lng']],
                popup=f"📍 当前位置\n高度: {latest['altitude']}m",
                icon=folium.Icon(color='red', icon='plane', prefix='fa')
            ).add_to(monitor_map)
            
            if st.session_state.points_gcj['A']:
                folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                             popup="📚 图书馆", icon=folium.Icon(color='green', icon='book', prefix='fa')).add_to(monitor_map)
            if st.session_state.points_gcj['B']:
                folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                             popup="🍽️ 食堂", icon=folium.Icon(color='blue', icon='utensils', prefix='fa')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            st.subheader("📈 数据图表")
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
            with chart_col2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "速度(m/s)": [h.get("speed", 0) for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
            
            st.subheader("📋 历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            st.dataframe(df, use_container_width=True)
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("🔄 立即刷新", use_container_width=True):
                new_hb = st.session_state.heartbeat_sim.generate()
                st.rerun()
        with col_btn2:
            if st.button("🗑️ 清空历史", use_container_width=True):
                st.session_state.heartbeat_sim.history = []
                st.rerun()
    
    # ==================== 障碍物管理页面 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        
        obs_list = st.session_state.obstacles_gcj
        st.info(f"💡 当前共 **{len(obs_list)}** 个障碍物\n\n**添加障碍物**：请在「航线规划」页面使用地图圈选功能\n\n**删除障碍物**：点击下方列表中的删除按钮")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("📝 障碍物列表")
            
            if obs_list:
                for i, obs in enumerate(obs_list):
                    col_name, col_btn = st.columns([3, 1])
                    with col_name:
                        st.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                    with col_btn:
                        if st.button("删除", key=f"del_{i}"):
                            st.session_state.obstacles_gcj.pop(i)
                            save_obstacles(st.session_state.obstacles_gcj)
                            st.rerun()
            else:
                st.write("暂无障碍物，请前往「航线规划」页面圈选建筑物添加")
            
            st.markdown("---")
            st.subheader("💾 持久化操作")
            
            col_save, col_load = st.columns(2)
            with col_save:
                if st.button("💾 保存到文件", use_container_width=True):
                    save_obstacles(st.session_state.obstacles_gcj)
                    st.success(f"已保存 {len(st.session_state.obstacles_gcj)} 个障碍物")
            with col_load:
                if st.button("📂 从文件加载", use_container_width=True):
                    st.session_state.obstacles_gcj = load_obstacles()
                    st.success(f"已加载 {len(st.session_state.obstacles_gcj)} 个障碍物")
                    st.rerun()
            
            col_clear, col_download = st.columns(2)
            with col_clear:
                if st.button("🗑️ 清除全部", use_container_width=True):
                    st.session_state.obstacles_gcj = []
                    save_obstacles([])
                    st.rerun()
            with col_download:
                config_data = {
                    'obstacles': st.session_state.obstacles_gcj,
                    'count': len(st.session_state.obstacles_gcj),
                    'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    'version': 'v12.2'
                }
                st.download_button(
                    label="📥 下载配置",
                    data=json.dumps(config_data, ensure_ascii=False, indent=2),
                    file_name=CONFIG_FILE,
                    mime="application/json",
                    use_container_width=True
                )
        
        with col2:
            st.subheader("🗺️ 障碍物分布图")
            
            obs_map = folium.Map(
                location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]],
                zoom_start=16,
                tiles=GAODE_SATELLITE_URL,
                attr="高德卫星地图"
            )
            
            for i, obs in enumerate(st.session_state.obstacles_gcj):
                coords = obs.get('polygon', [])
                if coords and len(coords) >= 3:
                    locations = [[c[1], c[0]] for c in coords]
                    folium.Polygon(
                        locations=locations,
                        color="red",
                        weight=3,
                        fill=True,
                        fill_color="red",
                        fill_opacity=0.5,
                        popup=f"🚧 {obs.get('name', f'障碍物{i+1}')}"
                    ).add_to(obs_map)
            
            folium.Marker(
                [DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]],
                popup="📚 图书馆",
                icon=folium.Icon(color='green', icon='book', prefix='fa')
            ).add_to(obs_map)
            
            folium.Marker(
                [DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]],
                popup="🍽️ 食堂",
                icon=folium.Icon(color='red', icon='utensils', prefix='fa')
            ).add_to(obs_map)
            
            folium_static(obs_map, width=700, height=500)
            st.caption("🔴 红色多边形为已圈选的建筑物障碍物")

if __name__ == "__main__":
    main()
