<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>无人机飞行规划与监控系统 - 3D地图 | 心跳监控</title>
    <!-- Cesium JS 核心库 (使用官方CDN，需要Ion Token，请替换为自己的或使用演示token) -->
    <link rel="stylesheet" href="https://cesium.com/downloads/cesiumjs/releases/1.113/Build/Cesium/Widgets/widgets.css">
    <script src="https://cesium.com/downloads/cesiumjs/releases/1.113/Build/Cesium/Cesium.js"></script>
    <!-- 坐标转换库 (GCJ-02 / WGS-84) -->
    <script src="https://cdn.jsdelivr.net/npm/coordtransform@2.1.2/index.js"></script>
    <!-- 样式与额外功能 -->
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            user-select: none; /* 避免拖拽时选中文本，不影响输入框 */
        }

        body {
            font-family: 'Segoe UI', 'Microsoft YaHei', 'PingFang SC', Roboto, Arial, sans-serif;
            overflow: hidden;
            height: 100vh;
            width: 100vw;
            background: #1a1e24;
            color: #eef2ff;
        }

        /* 顶部导航栏 */
        .navbar {
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 56px;
            background: rgba(18, 22, 35, 0.85);
            backdrop-filter: blur(12px);
            display: flex;
            align-items: center;
            padding: 0 28px;
            z-index: 100;
            border-bottom: 1px solid rgba(72, 187, 255, 0.3);
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
        }
        .logo {
            font-weight: 600;
            font-size: 1.3rem;
            background: linear-gradient(135deg, #3b82f6, #06b6d4);
            -webkit-background-clip: text;
            background-clip: text;
            color: transparent;
            margin-right: 40px;
        }
        .tab-buttons {
            display: flex;
            gap: 8px;
            background: rgba(0,0,0,0.3);
            border-radius: 40px;
            padding: 4px;
        }
        .tab-btn {
            background: transparent;
            border: none;
            color: #b9c7d9;
            font-size: 1rem;
            font-weight: 500;
            padding: 6px 24px;
            border-radius: 32px;
            cursor: pointer;
            transition: all 0.2s ease;
        }
        .tab-btn.active {
            background: #2d3a5e;
            color: white;
            box-shadow: 0 2px 8px rgba(0,120,255,0.3);
        }
        .system-status {
            margin-left: auto;
            display: flex;
            gap: 20px;
            font-size: 0.85rem;
            background: rgba(0,0,0,0.5);
            padding: 5px 16px;
            border-radius: 32px;
        }
        .status-badge {
            display: flex;
            align-items: center;
            gap: 6px;
        }
        .led {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #22c55e;
            box-shadow: 0 0 6px #22c55e;
        }
        /* 主要内容容器 */
        .main-container {
            position: absolute;
            top: 56px;
            left: 0;
            right: 0;
            bottom: 0;
            overflow: hidden;
        }
        .view {
            width: 100%;
            height: 100%;
            position: absolute;
            top: 0;
            left: 0;
            visibility: hidden;
            opacity: 0;
            transition: visibility 0.2s, opacity 0.2s;
        }
        .view.active {
            visibility: visible;
            opacity: 1;
            z-index: 10;
        }
        /* 航线规划视图：地图 + 右侧控制面板 */
        .planning-wrapper {
            display: flex;
            width: 100%;
            height: 100%;
        }
        #cesiumContainer {
            flex: 1;
            height: 100%;
            background: #0a0f1a;
        }
        .control-panel {
            width: 320px;
            background: rgba(20, 26, 38, 0.85);
            backdrop-filter: blur(16px);
            border-left: 1px solid rgba(59,130,246,0.4);
            padding: 20px 16px;
            display: flex;
            flex-direction: column;
            gap: 20px;
            overflow-y: auto;
            z-index: 20;
            box-shadow: -4px 0 20px rgba(0,0,0,0.3);
        }
        .panel-section {
            background: rgba(0,0,0,0.3);
            border-radius: 20px;
            padding: 14px 16px;
            borde
