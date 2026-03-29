
import streamlit as st
import time
import random
from datetime import datetime

# 页面配置
st.set_page_config(
    page_title="无人机心跳监控系统",
    page_icon="✈️",
    layout="wide"
)

# 初始化会话状态
if "drone_status" not in st.session_state:
    st.session_state.drone_status = "在线"
if "heartbeat_logs" not in st.session_state:
    st.session_state.heartbeat_logs = []

# 标题
st.title("✈️ 无人机心跳监控系统")
st.divider()

# 侧边栏配置
with st.sidebar:
    st.header("系统设置")
    drone_id = st.text_input("无人机ID", value="DRONE-001")
    check_interval = st.slider("心跳检测间隔(秒)", 1, 10, 3)
    st.divider()
    st.subheader("状态控制")
    start_monitor = st.button("启动监控", type="primary")
    stop_monitor = st.button("停止监控")
    reset_logs = st.button("清空日志")

# 主界面状态展示
col1, col2, col3 = st.columns(3)
with col1:
    st.metric("当前状态", st.session_state.drone_status)
with col2:
    st.metric("心跳次数", len(st.session_state.heartbeat_logs))
with col3:
    st.metric("最后心跳时间", 
              st.session_state.heartbeat_logs[-1]["time"] if st.session_state.heartbeat_logs else "无")

st.divider()

# 心跳日志展示
st.subheader("心跳日志")
log_container = st.container(height=400)

# 监控逻辑
if start_monitor:
    st.session_state.drone_status = "在线"
    st.success(f"已启动无人机 {drone_id} 心跳监控")
    
    # 模拟心跳检测循环
    while st.session_state.drone_status == "在线":
        # 生成模拟心跳数据
        heartbeat_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        signal_strength = random.randint(70, 100)
        battery = random.randint(30, 100)
        
        # 记录心跳日志
        log_entry = {
            "time": heartbeat_time,
            "drone_id": drone_id,
            "signal": signal_strength,
            "battery": battery,
            "status": "正常"
        }
        st.session_state.heartbeat_logs.append(log_entry)
        
        # 显示最新日志
        with log_container:
            for log in reversed(st.session_state.heartbeat_logs[-10:]):
                st.info(f"[{log['time']}] 无人机{log['drone_id']} | 信号强度: {log['signal']}% | 电量: {log['battery']}% | 状态: {log['status']}")
        
        # 检测间隔
        time.sleep(check_interval)
        
        # 模拟异常情况（10%概率）
        if random.random() < 0.1:
            st.session_state.drone_status = "离线"
            st.error(f"无人机 {drone_id} 心跳丢失！")
            break

if stop_monitor:
    st.session_state.drone_status = "离线"
    st.warning(f"已停止无人机 {drone_id} 监控")

if reset_logs:
    st.session_state.heartbeat_logs = []
    st.info("日志已清空")

# 底部提示
st.divider()
st.caption("系统说明：实时监控无人机心跳信号，记录信号强度、电量等关键数据，异常时自动报警")
