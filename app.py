import streamlit as st
import pandas as pd
import random
import time
import math
from datetime import datetime

# ==================== 坐标系转换函数 ====================
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

# ==================== 心跳包模拟 ====================
def generate_heartbeat():
    """生成一条模拟心跳数据"""
    return {
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "lat": 32.23775 + random.uniform(-0.003, 0.003),
        "lng": 118.7490 + random.uniform(-0.003, 0.003),
        "altitude": random.randint(40, 60),
        "voltage": round(random.uniform(11.5, 12.8), 1),
        "satellites": random.randint(8, 14),
    }

# 初始化 session_state
if "point_a" not in st.session_state:
    st.session_state.point_a = None  # [lng, lat] WGS84
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = time.time()

# 障碍物（WGS84坐标）
obstacles = [
    [118.7485, 32.2345],
    [118.7495, 32.2360],
    [118.7480, 32.2385],
    [118.7498, 32.2405],
    [118.7482, 32.2420],
]

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站", layout="wide")
st.sidebar.title("导航")
page = st.sidebar.radio("选择页面", ["航线规划", "飞行监控"])

# ==================== 航线规划页面 ====================
if page == "航线规划":
    st.title("🗺️ 航线规划")
    
    # 控制面板
    col1, col2 = st.columns([1, 1.5])
    
    with col1:
        st.subheader("控制面板")
        coord_sys = st.radio("输入坐标系", ["WGS-84", "GCJ-02 (高德/百度)"], index=1)
        st.markdown("---")
        
        st.markdown("**起点 A**")
        a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度", value=118.7490, format="%.6f", key="a_lng")
        if st.button("设置 A 点", use_container_width=True):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(a_lng, a_lat)
            else:
                wgs_lng, wgs_lat = a_lng, a_lat
            st.session_state.point_a = [wgs_lng, wgs_lat]
            st.success(f"✅ A点已设置")
        
        st.markdown("**终点 B**")
        b_lat = st.number_input("纬度", value=32.2433, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度", value=118.7490, format="%.6f", key="b_lng")
        if st.button("设置 B 点", use_container_width=True):
            if coord_sys == "GCJ-02 (高德/百度)":
                wgs_lng, wgs_lat = gcj02_to_wgs84(b_lng, b_lat)
            else:
                wgs_lng, wgs_lat = b_lng, b_lat
            st.session_state.point_b = [wgs_lng, wgs_lat]
            st.success(f"✅ B点已设置")
        
        st.markdown("---")
        flight_alt = st.number_input("设定飞行高度 (m)", value=50, step=5)
        st.markdown("---")
        
        if st.button("清除 A/B 点", use_container_width=True):
            st.session_state.point_a = None
            st.session_state.point_b = None
            st.info("已清除所有航点")
        
        # 显示当前坐标
        st.markdown("---")
        st.subheader("当前航点")
        st.write(f"A点: {st.session_state.point_a if st.session_state.point_a else '未设置'}")
        st.write(f"B点: {st.session_state.point_b if st.session_state.point_b else '未设置'}")
    
    with col2:
        st.subheader("地图 (OpenStreetMap)")
        
        # 准备地图数据
        map_data = []
        
        # 添加A点
        if st.session_state.point_a:
            map_data.append({"lat": st.session_state.point_a[1], "lon": st.session_state.point_a[0], "type": "A"})
        
        # 添加B点
        if st.session_state.point_b:
            map_data.append({"lat": st.session_state.point_b[1], "lon": st.session_state.point_b[0], "type": "B"})
        
        # 添加障碍物
        for obs in obstacles:
            map_data.append({"lat": obs[1], "lon": obs[0], "type": "障碍物"})
        
        if map_data:
            df = pd.DataFrame(map_data)
            
            # 根据类型设置颜色
            def get_color(row):
                if row['type'] == 'A':
                    return 'green'
                elif row['type'] == 'B':
                    return 'red'
                else:
                    return 'red'
            
            df['color'] = df.apply(get_color, axis=1)
            
            # 显示地图（使用 Streamlit 内置地图）
            st.map(df, latitude='lat', longitude='lon', color='color', size=100)
            
            # 显示图例
            st.caption("🟢 绿色: A点 | 🔴 红色: B点/障碍物")
        else:
            st.info("💡 请先在左侧设置 A 点和 B 点")
        
        st.markdown("---")
        st.caption("📌 提示：地图基于 OpenStreetMap，坐标使用 WGS-84 显示")

# ==================== 飞行监控页面 ====================
elif page == "飞行监控":
    st.title("📡 飞行监控 - 心跳包接收")
    
    # 自动更新心跳
    current_time = time.time()
    if current_time - st.session_state.last_heartbeat_time >= 2:
        new_hb = generate_heartbeat()
        st.session_state.heartbeat_history.insert(0, new_hb)
        if len(st.session_state.heartbeat_history) > 20:
            st.session_state.heartbeat_history.pop()
        st.session_state.last_heartbeat_time = current_time
        st.rerun()
    
    # 显示最新心跳
    if st.session_state.heartbeat_history:
        latest = st.session_state.heartbeat_history[0]
        
        col1, col2, col3, col4, col5 = st.columns(5)
        with col1:
            st.metric("⏰ 时间", latest["timestamp"])
        with col2:
            st.metric("📍 纬度", f"{latest['lat']:.6f}")
        with col3:
            st.metric("📍 经度", f"{latest['lng']:.6f}")
        with col4:
            st.metric("📊 高度", f"{latest['altitude']} m")
        with col5:
            st.metric("🔋 电压", f"{latest['voltage']} V")
        
        # 实时位置地图
        st.subheader("实时位置")
        pos_df = pd.DataFrame([{
            "lat": latest['lat'],
            "lon": latest['lng']
        }])
        st.map(pos_df, size=200)
        
        # 历史记录表格
        st.subheader("历史心跳记录")
        df = pd.DataFrame(st.session_state.heartbeat_history[:10])
        st.dataframe(df, use_container_width=True)
    else:
        st.info("⏳ 等待心跳数据...")
    
    # 控制按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🔄 立即刷新", use_container_width=True):
            new_hb = generate_heartbeat()
            st.session_state.heartbeat_history.insert(0, new_hb)
            if len(st.session_state.heartbeat_history) > 20:
                st.session_state.heartbeat_history.pop()
            st.rerun()
    
    with col2:
        if st.button("🗑️ 清空历史", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.rerun()
    
    st.markdown("---")
    st.caption("💡 心跳包每2秒自动更新一次，模拟无人机实时数据")
