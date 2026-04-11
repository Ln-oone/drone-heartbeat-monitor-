import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
import random
import time
import math
import json
from datetime import datetime
import pandas as pd

# ==================== 页面配置 ====================
st.set_page_config(page_title="南京科技职业学院 - 无人机地面站", layout="wide")

# ==================== 南京科技职业学院坐标 ====================
# 学校中心坐标 (WGS84)
SCHOOL_CENTER_WGS84 = [118.6965, 32.2015]  # [经度, 纬度]
# A点 (教学楼区域) B点 (操场区域) - 都位于校园内
DEFAULT_A_WGS84 = [118.6940, 32.2000]  # [经度, 纬度]
DEFAULT_B_WGS84 = [118.6990, 32.2030]

# ==================== 坐标系转换函数 (WGS84 ↔ GCJ-02) ====================
def wgs84_to_gcj02(lng, lat):
    """WGS84转GCJ02 (高德/百度坐标系)"""
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

def gcj02_to_wgs84(lng, lat):
    """GCJ02转WGS84 (OpenStreetMap坐标系)"""
    if out_of_china(lng, lat):
        return lng, lat
    
    a = 6378245.0
    ee = 0.00669342162296594323
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

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point):
        self.history = []
        self.current_pos = start_point.copy()  # [lng, lat]
        self.target_pos = None
        self.simulating = False
        
    def set_target(self, target):
        self.target_pos = target
        self.simulating = True
        
    def generate(self):
        if self.simulating and self.target_pos:
            # 向目标点移动
            dx = self.target_pos[0] - self.current_pos[0]
            dy = self.target_pos[1] - self.current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.0001:  # 到达目标
                self.simulating = False
            else:
                # 每次移动1/50的距离
                step_x = dx / 50
                step_y = dy / 50
                self.current_pos[0] += step_x
                self.current_pos[1] += step_y
        
        # 模拟高度变化 (起飞爬升 -> 巡航 -> 降落)
        if not self.simulating:
            altitude = random.randint(0, 10)  # 地面
        else:
            # 根据距离目标的进度模拟高度
            if self.target_pos:
                dx = self.target_pos[0] - self.current_pos[0]
                dy = self.target_pos[1] - self.current_pos[1]
                dist_to_target = math.sqrt(dx*dx + dy*dy)
                max_dist = 0.01
                if dist_to_target > max_dist * 0.8:
                    altitude = random.randint(40, 60)  # 巡航高度
                else:
                    altitude = random.randint(10, 40)  # 降落阶段
            else:
                altitude = random.randint(40, 60)
        
        heartbeat = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
            "speed": round(random.uniform(0, 15), 1) if self.simulating else 0,
        }
        
        self.history.insert(0, heartbeat)
        if len(self.history) > 50:
            self.history.pop()
        return heartbeat

# ==================== 障碍物管理 (带记忆功能) ====================
def load_obstacles():
    """加载障碍物多边形"""
    if "obstacles_geojson" not in st.session_state:
        # 默认障碍物 (校园内的建筑物区域)
        default_geojson = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": "教学楼", "type": "obstacle"},
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [118.6930, 32.1995],
                            [118.6945, 32.1995],
                            [118.6945, 32.2008],
                            [118.6930, 32.2008],
                            [118.6930, 32.1995]
                        ]]
                    }
                },
                {
                    "type": "Feature",
                    "properties": {"name": "实验楼", "type": "obstacle"},
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [118.6965, 32.2010],
                            [118.6978, 32.2010],
                            [118.6978, 32.2020],
                            [118.6965, 32.2020],
                            [118.6965, 32.2010]
                        ]]
                    }
                }
            ]
        }
        st.session_state.obstacles_geojson = default_geojson
    return st.session_state.obstacles_geojson

def save_obstacles(geojson):
    st.session_state.obstacles_geojson = geojson

def add_obstacle_from_draw(draw_data):
    """从地图绘制数据添加障碍物"""
    if draw_data and draw_data.get("geometry"):
        new_feature = {
            "type": "Feature",
            "properties": {"name": f"障碍物{len(st.session_state.obstacles_geojson['features'])+1}", "type": "obstacle"},
            "geometry": draw_data["geometry"]
        }
        st.session_state.obstacles_geojson["features"].append(new_feature)

# ==================== 创建地图 (支持多边形绘制) ====================
def create_planning_map(center_lat, center_lng, points, obstacles_geojson, flight_path=None):
    """创建航线规划地图"""
    m = folium.Map(
        location=[center_lat, center_lng], 
        zoom_start=16, 
        tiles="OpenStreetMap",
        attr="Google Satellite"
    )
    
    # 添加障碍物多边形
    if obstacles_geojson and obstacles_geojson.get("features"):
        for feature in obstacles_geojson["features"]:
            coords = feature["geometry"]["coordinates"]
            # 绘制多边形
            folium.Polygon(
                locations=[[c[1], c[0]] for c in coords[0]],
                color="red",
                weight=2,
                fill=True,
                fill_color="red",
                fill_opacity=0.4,
                popup=feature["properties"]["name"]
            ).add_to(m)
    
    # 添加 A 点
    if points.get('A'):
        folium.Marker(
            points['A'], 
            popup="🟢 起点 A", 
            icon=folium.Icon(color="green", icon="play", prefix="fa")
        ).add_to(m)
    
    # 添加 B 点
    if points.get('B'):
        folium.Marker(
            points['B'], 
            popup="🔴 终点 B", 
            icon=folium.Icon(color="red", icon="stop", prefix="fa")
        ).add_to(m)
    
    # 添加航线
    if points.get('A') and points.get('B'):
        folium.PolyLine(
            locations=[points['A'], points['B']],
            color="blue",
            weight=3,
            opacity=0.8,
            popup="规划航线"
        ).add_to(m)
    
    # 添加飞行轨迹
    if flight_path and len(flight_path) > 1:
        folium.PolyLine(
            flight_path,
            color="orange",
            weight=2,
            opacity=0.6,
            popup="历史轨迹"
        ).add_to(m)
    
    return m

# ==================== 主程序 ====================
def main():
    st.title("✈️ 南京科技职业学院 - 无人机地面站系统")
    st.markdown("---")
    
    # 侧边栏导航
    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio(
        "选择功能模块",
        ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"]
    )
    
    # 初始化 Session State
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(DEFAULT_A_WGS84)
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "points" not in st.session_state:
        st.session_state.points = {
            'A': DEFAULT_A_WGS84.copy(),
            'B': DEFAULT_B_WGS84.copy()
        }
    if "input_coord_sys" not in st.session_state:
        st.session_state.input_coord_sys = "GCJ-02 (高德/百度)"
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    
    obstacles_geojson = load_obstacles()
    
    # ==================== 航线规划页面 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划")
        st.info("💡 操作说明：设置起点(A)和终点(B)，系统将自动规划航线并避开红色障碍物区域")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("🎮 控制面板")
            
            # 坐标系选择
            coord_sys = st.radio(
                "📐 输入坐标系",
                ["WGS-84", "GCJ-02 (高德/百度)"],
                index=1,
                help="选择你输入的坐标所使用的坐标系，系统会自动转换"
            )
            st.session_state.input_coord_sys = coord_sys
            
            st.markdown("---")
            
            # 起点 A 设置
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=DEFAULT_A_WGS84[1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=DEFAULT_A_WGS84[0], format="%.6f", key="a_lng")
            
            if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
                else:
                    wgs_lng, wgs_lat = a_lng, a_lat
                st.session_state.points['A'] = [wgs_lat, wgs_lng]
                st.success(f"✅ A点已设置")
            
            st.markdown("---")
            
            # 终点 B 设置
            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=DEFAULT_B_WGS84[1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=DEFAULT_B_WGS84[0], format="%.6f", key="b_lng")
            
            if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
                else:
                    wgs_lng, wgs_lat = b_lng, b_lat
                st.session_state.points['B'] = [wgs_lat, wgs_lng]
                st.success(f"✅ B点已设置")
            
            st.markdown("---")
            
            # 飞行控制
            st.markdown("#### 🚁 飞行控制")
            
            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    st.session_state.heartbeat_sim.set_target(st.session_state.points['B'])
                    st.session_state.simulation_running = True
                    st.success("飞行已开始！请切换到「飞行监控」页面查看实时数据")
            
            with col_btn2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.info("飞行已停止")
            
            # 显示当前坐标
            st.markdown("---")
            st.markdown("### 📍 当前航点")
            st.write(f"🟢 A点: ({st.session_state.points['A'][0]:.6f}, {st.session_state.points['A'][1]:.6f})")
            st.write(f"🔴 B点: ({st.session_state.points['B'][0]:.6f}, {st.session_state.points['B'][1]:.6f})")
        
        with col2:
            st.subheader("🗺️ 规划地图 (卫星影像)")
            
            # 获取飞行轨迹用于显示
            flight_trail = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:20]]
            
            planning_map = create_planning_map(
                st.session_state.points['A'][0] if st.session_state.points['A'] else SCHOOL_CENTER_WGS84[1],
                st.session_state.points['A'][1] if st.session_state.points['A'] else SCHOOL_CENTER_WGS84[0],
                st.session_state.points,
                obstacles_geojson,
                flight_trail
            )
            folium_static(planning_map, width=700, height=500)
            
            st.caption("📌 **图例**：🟢 A点 | 🔴 B点 | 🔴 红色区域=障碍物 | 🔵 蓝色线=规划航线 | 🟠 橙色线=历史飞行轨迹")
    
    # ==================== 飞行监控页面 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        
        # 自动生成心跳包
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 1 and st.session_state.simulation_running:
            new_hb = st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
            # 记录飞行轨迹
            st.session_state.flight_history.append([new_hb['lat'], new_hb['lng']])
            if len(st.session_state.flight_history) > 100:
                st.session_state.flight_history.pop(0)
            st.rerun()
        
        # 显示最新心跳数据
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            # 指标卡片
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
            
            # 进度条显示飞行进度
            if st.session_state.points['A'] and st.session_state.points['B']:
                a = st.session_state.points['A']
                b = st.session_state.points['B']
                current = [latest['lat'], latest['lng']]
                total_dist = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
                current_dist = math.sqrt((current[0]-a[0])**2 + (current[1]-a[1])**2)
                if total_dist > 0:
                    progress = min(1.0, current_dist / total_dist)
                    st.progress(progress, text=f"飞行进度: {progress*100:.1f}%")
            
            # 实时位置地图
            st.subheader("📍 实时位置")
            monitor_map = folium.Map(
                location=[latest['lat'], latest['lng']],
                zoom_start=17,
                tiles="OpenStreetMap",
                attr="Google Satellite"
            )
            
            # 绘制完整轨迹
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:30]]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="orange", weight=3, opacity=0.7, popup="飞行轨迹").add_to(monitor_map)
            
            # 最新位置
            folium.Marker(
                [latest['lat'], latest['lng']],
                popup=f"当前位置\n高度: {latest['altitude']}m",
                icon=folium.Icon(color='red', icon='plane', prefix='fa')
            ).add_to(monitor_map)
            
            # 添加 A/B 点标记
            if st.session_state.points['A']:
                folium.Marker(st.session_state.points['A'], popup="起点 A", icon=folium.Icon(color='green')).add_to(monitor_map)
            if st.session_state.points['B']:
                folium.Marker(st.session_state.points['B'], popup="终点 B", icon=folium.Icon(color='blue')).add_to(monitor_map)
            
            folium_static(monitor_map, width=800, height=400)
            
            # 历史数据表格
            st.subheader("📋 历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:15])
            st.dataframe(df, use_container_width=True)
            
            # 数据图表
            st.subheader("📈 数据图表")
            chart_col1, chart_col2 = st.columns(2)
            with chart_col1:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    alt_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "高度(m)": [h["altitude"] for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(alt_df, x="序号", y="高度(m)")
                    st.caption("高度变化趋势")
            with chart_col2:
                if len(st.session_state.heartbeat_sim.history) > 1:
                    speed_df = pd.DataFrame({
                        "序号": range(len(st.session_state.heartbeat_sim.history[:20])),
                        "速度(m/s)": [h.get("speed", 0) for h in st.session_state.heartbeat_sim.history[:20]]
                    })
                    st.line_chart(speed_df, x="序号", y="速度(m/s)")
                    st.caption("速度变化趋势")
        else:
            st.info("⏳ 等待心跳数据... 请在「航线规划」页面点击「开始飞行」")
        
        # 控制按钮
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
        st.info("💡 可以添加、删除障碍物多边形，系统会记住你的设置（刷新页面不丢失）")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            st.subheader("📝 障碍物列表")
            
            if obstacles_geojson.get("features"):
                for i, feature in enumerate(obstacles_geojson["features"]):
                    col_name, col_btn = st.columns([3, 1])
                    with col_name:
                        st.write(f"• {feature['properties']['name']}")
                    with col_btn:
                        if st.button("删除", key=f"del_{i}"):
                            obstacles_geojson["features"].pop(i)
                            save_obstacles(obstacles_geojson)
                            st.rerun()
            else:
                st.write("暂无障碍物")
            
            st.markdown("---")
            st.subheader("➕ 添加障碍物")
            obs_name = st.text_input("障碍物名称", value=f"障碍物{len(obstacles_geojson.get('features', [])) + 1}")
            st.warning("⚠️ 当前版本需要通过修改代码添加多边形坐标，或使用下面的示例坐标")
            
            # 添加示例障碍物
            if st.button("添加示例障碍物", use_container_width=True):
                new_feature = {
                    "type": "Feature",
                    "properties": {"name": obs_name, "type": "obstacle"},
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [118.6950, 32.2015],
                            [118.6960, 32.2015],
                            [118.6960, 32.2025],
                            [118.6950, 32.2025],
                            [118.6950, 32.2015]
                        ]]
                    }
                }
                obstacles_geojson["features"].append(new_feature)
                save_obstacles(obstacles_geojson)
                st.success(f"已添加障碍物: {obs_name}")
                st.rerun()
            
            if st.button("🗑️ 清除所有障碍物", use_container_width=True):
                obstacles_geojson["features"] = []
                save_obstacles(obstacles_geojson)
                st.rerun()
        
        with col2:
            st.subheader("🗺️ 障碍物地图")
            
            # 显示当前障碍物位置
            obs_map = folium.Map(
                location=[SCHOOL_CENTER_WGS84[1], SCHOOL_CENTER_WGS84[0]],
                zoom_start=16,
                tiles="OpenStreetMap",
                attr="Google Satellite"
            )
            
            if obstacles_geojson.get("features"):
                for feature in obstacles_geojson["features"]:
                    coords = feature["geometry"]["coordinates"]
                    folium.Polygon(
                        locations=[[c[1], c[0]] for c in coords[0]],
                        color="red",
                        weight=2,
                        fill=True,
                        fill_color="red",
                        fill_opacity=0.5,
                        popup=feature["properties"]["name"]
                    ).add_to(obs_map)
            
            folium_static(obs_map, width=700, height=500)
            st.caption("🔴 红色区域为已设置的障碍物")

if __name__ == "__main__":
    main()
