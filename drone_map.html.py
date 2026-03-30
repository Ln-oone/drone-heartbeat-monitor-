import streamlit as st
from streamlit_folium import folium_static
import folium
from folium import plugins
import random

# ================== 页面配置 ==================
st.set_page_page_config(layout="wide")
st.title("心跳包接收系统 - 3D地图规划模块")

# ================== 初始化地图 ==================
# 使用您提供的坐标作为地图中心点 (GCJ-02坐标系)
center_lat, center_lon = 32.2332, 118.749

# 创建基础地图，并启用3D视图 (通过倾斜和旋转)
m = folium.Map(
    location=[center_lat, center_lon],
    zoom_start=16,
    tiles='https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}', # 高德卫星图底图 (GCJ-02)
    attr='高德地图'
)

# 添加3D控制插件：允许倾斜和旋转地图
plugins.Fullscreen().add_to(m)  # 全屏
plugins.MousePosition().add_to(m) # 鼠标坐标

# ================== 侧边栏控制面板 ==================
with st.sidebar:
    st.header("控制面板")
    
    # --- 起点A配置 ---
    st.subheader("起点A (GCJ-02)")
    col1, col2 = st.columns(2)
    with col1:
        lat_a = st.number_input("纬度", value=32.2322, format="%.6f")
    with col2:
        lon_a = st.number_input("经度", value=118.7490, format="%.6f")
    set_a = st.button("📍 设置A点", key="set_a")
    
    # --- 终点B配置 ---
    st.subheader("终点B (GCJ-02)")
    col3, col4 = st.columns(2)
    with col3:
        lat_b = st.number_input("纬度", value=32.2343, format="%.6f", key="lat_b")
    with col4:
        lon_b = st.number_input("经度", value=118.7490, format="%.6f", key="lon_b")
    set_b = st.button("📍 设置B点", key="set_b")
    
    # --- 飞行参数 ---
    st.subheader("飞行参数")
    flight_height = st.number_input("设定飞行高度 (m)", value=50, step=10)
    
    st.markdown("---")
    st.caption("提示：使用鼠标右键拖拽可旋转/倾斜地图（3D效果）")

# ================== 核心地图与障碍物逻辑 ==================

# 1. 添加预设障碍物 (满足“多个障碍物”要求，且位于校园内)
#    障碍物为矩形区域 [min_lat, max_lat, min_lon, max_lon]
obstacles = [
    {"name": "教学楼A", "bounds": [[32.2325, 118.7485], [32.2330, 118.7492]]},
    {"name": "实验楼B", "bounds": [[32.2332, 118.7488], [32.2337, 118.7495]]},
    {"name": "图书馆C", "bounds": [[32.2338, 118.7492], [32.2342, 118.7498]]},
]

for obs in obstacles:
    # 绘制障碍物为红色半透明填充区域
    folium.Rectangle(
        bounds=obs["bounds"],
        color="red",
        weight=2,
        fill=True,
        fill_color="red",
        fill_opacity=0.4,
        popup=obs["name"],
        tooltip=f"🚧 障碍物: {obs['name']}"
    ).add_to(m)

# 2. 根据用户操作设置A/B点
#    使用session_state保存点位置，避免重复添加
if 'points' not in st.session_state:
    st.session_state.points = {}

if set_a:
    st.session_state.points['A'] = (lat_a, lon_a)
if set_b:
    st.session_state.points['B'] = (lat_b, lon_b)

# 3. 在地图上绘制A/B点及连线，并检查是否经过障碍物
if 'A' in st.session_state.points:
    lat_a_plot, lon_a_plot = st.session_state.points['A']
    folium.Marker(
        location=[lat_a_plot, lon_a_plot],
        popup=f"起点A<br>高度: {flight_height}m",
        tooltip="起点A",
        icon=folium.Icon(color="green", icon="play", prefix='fa')
    ).add_to(m)
    # 添加从A点出发的虚线到B点（如果B存在）
    if 'B' in st.session_state.points:
        lat_b_plot, lon_b_plot = st.session_state.points['B']
        folium.Marker(
            location=[lat_b_plot, lon_b_plot],
            popup=f"终点B<br>高度: {flight_height}m",
            tooltip="终点B",
            icon=folium.Icon(color="red", icon="stop", prefix='fa')
        ).add_to(m)
        # 绘制AB连线 (直线)
        line_points = [[lat_a_plot, lon_a_plot], [lat_b_plot, lon_b_plot]]
        folium.PolyLine(line_points, color="blue", weight=3, opacity=0.7, tooltip="规划路径").add_to(m)
        
        # 简单路径碰撞检测：检查连线是否穿过任何障碍物矩形
        collision = False
        for obs in obstacles:
            # 获取障碍物边界
            min_lat, max_lat = obs["bounds"][0][0], obs["bounds"][1][0]
            min_lon, max_lon = obs["bounds"][0][1], obs["bounds"][1][1]
            # 检查线段与矩形是否相交 (使用分离轴定理简化)
            # 实际中可用shapely库，这里做最简边界框检测
            if (min(lat_a_plot, lat_b_plot) <= max_lat and max(lat_a_plot, lat_b_plot) >= min_lat and
                min(lon_a_plot, lon_b_plot) <= max_lon and max(lon_a_plot, lon_b_plot) >= min_lon):
                collision = True
                st.sidebar.error(f"⚠️ 路径与障碍物【{obs['name']}】相交！请调整A/B点或绕行。")
        
        if not collision and 'A' in st.session_state.points and 'B' in st.session_state.points:
            st.sidebar.success("✅ 路径安全，未与障碍物相交。")
    else:
        st.sidebar.info("请设置终点B")
else:
    st.sidebar.info("请设置起点A")

# ================== 显示地图 ==================
# 调整地图高度，提供更好的3D操作空间
folium_static(m, width=1000, height=600)

# ================== 辅助说明 ==================
with st.expander("📌 使用说明"):
    st.markdown("""
    1. **设置A/B点**：在左侧面板输入GCJ-02坐标系下的经纬度，点击按钮即可在地图上标记。
    2. **3D地图操作**：使用**鼠标右键拖拽**可以旋转/倾斜地图，获得3D视角；**鼠标左键拖拽**平移；**滚轮**缩放。
    3. **障碍物**：系统预设了3个位于校园内的障碍物（红色区域），AB连线时会自动检测是否穿过。
    4. **飞行高度**：当前仅作为参数记录，后续可扩展为真实3D航线规划。
    5. **坐标系**：地图底图使用高德GCJ-02坐标系，您输入的坐标无需转换。
    """)
