import streamlit as st
import folium
from streamlit_folium import folium_static
import random
import time
import math
from datetime import datetime
import pandas as pd

# ==================== 坐标系转换函数 ====================
def wgs84_to_gcj02(lng, lat):
    """WGS84转GCJ02"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
    
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
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)
    
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

# ==================== 心跳包模拟 ====================
class HeartbeatSimulator:
    def __init__(self):
        self.history = []

    def generate_heartbeat(self):
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lat": 32.23775 + random.uniform(-0.003, 0.003),
            "lon": 118.7490 + random.uniform(-0.003, 0.003),
            "altitude": random.randint(40, 60),
            "voltage": round(random.uniform(11.5, 12.8), 1),
            "satellites": random.randint(8, 14),
        }

    def update(self):
        hb = self.generate_heartbeat()
        self.history.insert(0, hb)
        if len(self.history) > 20:
            self.history.pop()
        return hb

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站", layout="wide")

# 初始化 session_state
if "heartbeat" not in st.session_state:
    st.session_state.heartbeat = HeartbeatSimulator()
if "point_a_wgs84" not in st.session_state:
    st.session_state.point_a_wgs84 = None
if "point_b_wgs84" not in st.session_state:
    st.session_state.point_b_wgs84 = None
if "last_update" not in st.session_state:
    st.session_state.last_update = time.time()

# 障碍物（WGS84坐标，[lng, lat]）
obstacles = [
    [118.7485, 32.2345],
    [118.7495, 32.2360],
    [118.7480, 32.2385],
    [118.7498, 32.2405],
    [118.7482, 32.2420],
]

# 地图中心
CENTER = [32.23775, 118.7490]  # [lat, lng]

# 侧边栏导航
st.sidebar.title("导航")
page = st.sidebar.radio("选择页面", ["航线规划", "飞行监控"])

# ==================== 航线规划页面 ====================
if page == "航线规划":
    st.title("🗺️ 航线规划")
    
    # 两列布局
    col_left, col_right = st.columns([1, 2])
    
    with col_left:
        st.subheader("控制面板")
        
        # 坐标系选择
        coord_sys = st.radio("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"], index=1)
        st.markdown("---")
        
        # A点设置
        st.markdown("**起点 A**")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
        
        if st.button("设置 A 点", use_container_width=True):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a_wgs84 = [wgs_lng, wgs_lat]
            st.success(f"✓ A点已设置")
        
        st.markdown("---")
        
        # B点设置
        st.markdown("**终点 B**")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
        
        if st.button("设置 B 点", use_container_width=True):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b_wgs84 = [wgs_lng, wgs_lat]
            st.success(f"✓ B点已设置")
        
        st.markdown("---")
        
        # 飞行参数
        flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5)
        
        st.markdown("---")
        
        if st.button("清除 A/B 点", use_container_width=True):
            st.session_state.point_a_wgs84 = None
            st.session_state.point_b_wgs84 = None
            st.info("已清除所有航点")
        
        # 显示当前坐标
        st.markdown("---")
        st.subheader("当前航点")
        st.write(f"A点: {st.session_state.point_a_wgs84 if st.session_state.point_a_wgs84 else '未设置'}")
        st.write(f"B点: {st.session_state.point_b_wgs84 if st.session_state.point_b_wgs84 else '未设置'}")
    
    with col_right:
        st.subheader("地图 (高德卫星图)")
        
        # 使用高德地图瓦片（国内稳定，带中文地名）
        tiles = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
        attr = "高德地图"
        
        m = folium.Map(location=CENTER, zoom_start=16, tiles=tiles, attr=attr)
        
        # 添加障碍物（红色方块）
        for obs in obstacles:
            folium.CircleMarker(
                location=[obs[1], obs[0]],
                radius=8,
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.8,
                popup="🚧 障碍物"
            ).add_to(m)
            # 添加方形区域
            folium.Rectangle(
                bounds=[[obs[1]-0.0003, obs[0]-0.0003], [obs[1]+0.0003, obs[0]+0.0003]],
                color="red",
                fill=True,
                fill_opacity=0.3,
                weight=2
            ).add_to(m)
        
        # 添加 A 点
        if st.session_state.point_a_wgs84:
            lng_a, lat_a = st.session_state.point_a_wgs84
            folium.Marker(
                location=[lat_a, lng_a],
                popup="🚩 起点 A",
                icon=folium.Icon(color="green", icon="play", prefix="fa")
            ).add_to(m)
            folium.CircleMarker(
                [lat_a, lng_a], 
                radius=6, 
                color="green", 
                fill=True, 
                fill_opacity=0.7,
                popup="A"
            ).add_to(m)
        
        # 添加 B 点
        if st.session_state.point_b_wgs84:
            lng_b, lat_b = st.session_state.point_b_wgs84
            folium.Marker(
                location=[lat_b, lng_b],
                popup="🏁 终点 B",
                icon=folium.Icon(color="red", icon="stop", prefix="fa")
            ).add_to(m)
            folium.CircleMarker(
                [lat_b, lng_b], 
                radius=6, 
                color="red", 
                fill=True, 
                fill_opacity=0.7,
                popup="B"
            ).add_to(m)
        
        # 连线
        if st.session_state.point_a_wgs84 and st.session_state.point_b_wgs84:
            points = [
                [st.session_state.point_a_wgs84[1], st.session_state.point_a_wgs84[0]],
                [st.session_state.point_b_wgs84[1], st.session_state.point_b_wgs84[0]]
            ]
            folium.PolyLine(
                points, 
                color="blue", 
                weight=3, 
                opacity=0.7, 
                popup="✈️ 规划航线"
            ).add_to(m)
            
            # 计算距离（粗略）
            from math import radians, sin, cos, sqrt, asin
            def haversine(lon1, lat1, lon2, lat2):
                R = 6371
                dlon = radians(lon2 - lon1)
                dlat = radians(lat2 - lat1)
                a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
                return R * 2 * asin(sqrt(a))
            
            dist = haversine(st.session_state.point_a_wgs84[0], st.session_state.point_a_wgs84[1],
                           st.session_state.point_b_wgs84[0], st.session_state.point_b_wgs84[1])
            st.info(f"📏 航线距离: {dist:.2f} km | 飞行高度: {flight_alt} m")
        
        # 显示地图
        folium_static(m, width=800, height=600)

# ==================== 飞行监控页面 ====================
elif page == "飞行监控":
    st.title("📡 飞行监控")
    
    # 自动更新心跳（每2秒）
    if time.time() - st.session_state.last_update >= 2:
        st.session_state.heartbeat.update()
        st.session_state.last_update = time.time()
        st.rerun()
    
    # 显示最新心跳
    if st.session_state.heartbeat.history:
        latest = st.session_state.heartbeat.history[0]
        
        col1, col2, col3, col4, col5 = st.columns(5)
        with col1:
            st.metric("⏰ 时间", latest["timestamp"])
        with col2:
            st.metric("📍 纬度", f"{latest['lat']:.6f}")
        with col3:
            st.metric("📍 经度", f"{latest['lon']:.6f}")
        with col4:
            st.metric("📊 高度", f"{latest['altitude']} m")
        with col5:
            st.metric("🔋 电压", f"{latest['voltage']} V")
        
        st.markdown("---")
        st.subheader("📋 历史心跳记录")
        
        # 修复：创建正确的DataFrame格式，避免颜色列错误
        df = pd.DataFrame(st.session_state.heartbeat.history[:10])
        
        # 使用st.dataframe而不是st.map，避免颜色格式错误
        st.dataframe(df, use_container_width=True)
        
        # 可选：在地图上显示飞行轨迹（使用folium）
        st.subheader("🗺️ 飞行轨迹")
        m = folium.Map(location=[latest["lat"], latest["lon"]], zoom_start=16, 
                       tiles="https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
                       attr="高德地图")
        
        # 添加历史轨迹点
        for i, hb in enumerate(st.session_state.heartbeat.history[:10]):
            folium.CircleMarker(
                [hb["lat"], hb["lon"]],
                radius=4,
                color="blue" if i == 0 else "gray",
                fill=True,
                popup=f"时间: {hb['timestamp']}\n高度: {hb['altitude']}m"
            ).add_to(m)
        
        folium_static(m, width=800, height=400)
    else:
        st.info("⏳ 等待心跳数据...")
    
    # 手动刷新
    if st.button("🔄 立即刷新心跳", use_container_width=True):
        st.session_state.heartbeat.update()
        st.rerun()
    
    st.markdown("---")
    st.caption("💡 心跳包每2秒自动更新（模拟数据）")
