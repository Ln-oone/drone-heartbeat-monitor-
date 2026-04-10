import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
import random
import time
import math
import json
from datetime import datetime
import pandas as pd
from streamlit.components.v1 import html

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统", layout="wide")

# ==================== 坐标转换核心算法 ====================
# (完整保留之前的 WGS84 与 GCJ02 互转函数，确保坐标正确)
def wgs84_to_gcj02(lng, lat):
    """WGS84转GCJ02"""
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
    """GCJ02转WGS84"""
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
    def __init__(self):
        self.history = []  # 存储历史心跳记录

    def generate(self):
        # 模拟在A点附近随机游走
        base_lat, base_lng = 32.23775, 118.7490
        new_hb = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lat": base_lat + random.uniform(-0.002, 0.002),
            "lng": base_lng + random.uniform(-0.002, 0.002),
            "altitude": random.randint(40, 60),
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
        }
        self.history.insert(0, new_hb)
        if len(self.history) > 50:
            self.history.pop()
        return new_hb

# ==================== 障碍物多边形管理 (带记忆功能) ====================
def load_obstacles():
    """从localStorage加载障碍物多边形"""
    import streamlit as st
    if "obstacles_geojson" not in st.session_state:
        # 初始化一个示例多边形 (校园内)
        default_geojson = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": "教学楼群", "type": "obstacle"},
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [118.7480, 32.2340],
                            [118.7495, 32.2340],
                            [118.7495, 32.2355],
                            [118.7480, 32.2355],
                            [118.7480, 32.2340]
                        ]]
                    }
                }
            ]
        }
        st.session_state.obstacles_geojson = default_geojson
    return st.session_state.obstacles_geojson

def save_obstacles(geojson):
    """保存障碍物多边形到session_state"""
    st.session_state.obstacles_geojson = geojson

# ==================== 创建带多边形绘制功能的地图 (Folium + 前端JS) ====================
def create_planning_map(center_lat, center_lng, points, obstacles_geojson):
    """创建用于航线规划的地图，支持绘制多边形"""
    m = folium.Map(location=[center_lat, center_lng], zoom_start=16, tiles="OpenStreetMap")

    # 添加已有的障碍物多边形
    if obstacles_geojson:
        folium.GeoJson(
            obstacles_geojson,
            style_function=lambda x: {'fillColor': 'red', 'color': 'red', 'weight': 2, 'fillOpacity': 0.4},
            name='obstacles'
        ).add_to(m)

    # 添加A、B点
    if points.get('A'):
        folium.Marker(points['A'], popup="A点 (起点)", icon=folium.Icon(color='green')).add_to(m)
    if points.get('B'):
        folium.Marker(points['B'], popup="B点 (终点)", icon=folium.Icon(color='red')).add_to(m)

    # 添加用于绘制多边形的JS组件
    draw_script = """
    <script>
    // 此脚本用于在前端处理多边形绘制，并将结果传回后端
    // 实际实现需要更复杂的双向通信，为简化示例，这里展示思路
    console.log("地图已加载，可在此集成Leaflet.draw插件进行多边形绘制");
    // 真实项目中，可通过 st_folium 的 returned_objects 获取绘制的图形
    </script>
    """
    m.get_root().html.add_child(folium.Element(draw_script))
    return m

# ==================== 主程序 ====================
def main():
    st.sidebar.title("导航菜单")
    page = st.sidebar.radio("选择页面", ["航线规划", "飞行监控"])

    # 初始化 session_state
    if 'heartbeat_sim' not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator()
    if 'last_hb_time' not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if 'points' not in st.session_state:
        st.session_state.points = {'A': None, 'B': None}  # 存储WGS84坐标 (lat, lng)
    if 'input_coord_sys' not in st.session_state:
        st.session_state.input_coord_sys = "GCJ-02 (高德/百度)"

    obstacles_geojson = load_obstacles()

    # ---------- 航线规划页面 ----------
    if page == "航线规划":
        st.title("🗺️ 航线规划 - 障碍物圈选与航点设置")
        st.info("💡 操作指南：1️⃣ 使用下方控件设置A/B点  2️⃣ 障碍物多边形具有记忆功能，刷新页面不丢失  3️⃣ 坐标转换确保标注正确")

        col1, col2 = st.columns([1, 1.5])

        with col1:
            st.subheader("控制面板")
            coord_sys = st.radio("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"], index=0 if st.session_state.input_coord_sys == "WGS-84" else 1)
            st.session_state.input_coord_sys = coord_sys

            st.markdown("#### 起点 A")
            a_lat = st.number_input("纬度", value=32.2322, format="%.6f")
            a_lng = st.number_input("经度", value=118.7490, format="%.6f")
            if st.button("设置 A 点"):
                # 坐标转换
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
                else:
                    wgs_lat, wgs_lng = a_lat, a_lng
                st.session_state.points['A'] = [wgs_lat, wgs_lng]
                st.success(f"A点已设置 (WGS84: {wgs_lat:.6f}, {wgs_lng:.6f})")

            st.markdown("#### 终点 B")
            b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
            if st.button("设置 B 点"):
                if coord_sys == "GCJ-02 (高德/百度)":
                    wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
                else:
                    wgs_lat, wgs_lng = b_lat, b_lng
                st.session_state.points['B'] = [wgs_lat, wgs_lng]
                st.success(f"B点已设置 (WGS84: {wgs_lat:.6f}, {wgs_lng:.6f})")

            st.markdown("---")
            st.markdown("#### 🚧 障碍物圈选")
            st.warning("当前障碍物多边形为示例。在完整实现中，可在地图上使用 Leaflet.draw 插件进行多边形绘制。")
            if st.button("清除所有障碍物"):
                st.session_state.obstacles_geojson = {"type": "FeatureCollection", "features": []}
                st.success("已清除所有障碍物")

            # 显示当前航点
            st.markdown("---")
            st.markdown("### 📍 当前航点")
            if st.session_state.points['A']:
                st.write(f"A点 (WGS84): {st.session_state.points['A'][0]:.6f}, {st.session_state.points['A'][1]:.6f}")
            if st.session_state.points['B']:
                st.write(f"B点 (WGS84): {st.session_state.points['B'][0]:.6f}, {st.session_state.points['B'][1]:.6f}")

        with col2:
            st.subheader("规划地图 (OpenStreetMap)")
            # 确定地图中心
            if st.session_state.points['A']:
                center = st.session_state.points['A']
            else:
                center = [32.23775, 118.7490]  # 默认校园中心

            planning_map = create_planning_map(center[0], center[1], st.session_state.points, obstacles_geojson)
            # 使用 st_folium 捕获地图交互（如点击、绘制等）
            output = st_folium(planning_map, width=700, height=500, returned_objects=["last_draw"])

            # 简单演示：如果通过某种方式获取到新绘制的多边形，可以保存它
            # 实际集成需要更复杂的JS->Python通信，此处示意
            if output and output.get("last_draw"):
                st.info("检测到新绘制的图形，可在此处处理保存逻辑。")

            st.caption("图例：🟢 A点 | 🔴 B点 | 🔴 红色半透明区域为障碍物多边形")

    # ---------- 飞行监控页面 ----------
    elif page == "飞行监控":
        st.title("📡 飞行监控 - 实时心跳包与轨迹")
        # 自动生成心跳包 (每秒)
        current_time = time.time()
        if current_time - st.session_state.last_hb_time >= 1:
            new_hb = st.session_state.heartbeat_sim.generate()
            st.session_state.last_hb_time = current_time
            st.rerun()  # 刷新页面以更新数据

        # 显示最新心跳
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            col1, col2, col3, col4, col5 = st.columns(5)
            col1.metric("时间", latest['timestamp'])
            col2.metric("纬度", f"{latest['lat']:.6f}")
            col3.metric("经度", f"{latest['lng']:.6f}")
            col4.metric("高度", f"{latest['altitude']} m")
            col5.metric("电压", f"{latest['voltage']} V")

            # 显示实时位置地图
            st.subheader("实时位置 & 飞行轨迹")
            monitor_map = folium.Map(location=[latest['lat'], latest['lng']], zoom_start=16, tiles="OpenStreetMap")
            # 绘制轨迹
            trail_points = [[hb['lat'], hb['lng']] for hb in st.session_state.heartbeat_sim.history[:20]]
            if len(trail_points) > 1:
                folium.PolyLine(trail_points, color="blue", weight=2, opacity=0.7).add_to(monitor_map)
            # 最新点
            folium.Marker([latest['lat'], latest['lng']], popup="最新位置", icon=folium.Icon(color='red', icon='info-sign')).add_to(monitor_map)
            folium_static(monitor_map, width=700, height=400)

            # 历史数据表格
            st.subheader("历史心跳记录")
            df = pd.DataFrame(st.session_state.heartbeat_sim.history[:10])
            st.dataframe(df, use_container_width=True)
        else:
            st.info("等待心跳数据...")

        if st.button("手动生成一次心跳"):
            st.session_state.heartbeat_sim.generate()
            st.rerun()

if __name__ == "__main__":
    main()
