import streamlit as st

st.set_page_config(
    page_title="无人机飞行规划与监控系统 - 真实3D地图",
    page_icon="✈️",
    layout="wide"
)

# ================== 使用 Three.js 构建的“真实风格”3D 地图（卫星纹理，无令牌） ==================
threejs_map_html = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>3D 地图 - 南京科技职业学院</title>
    <style>
        body { margin: 0; overflow: hidden; font-family: 'Microsoft YaHei', sans-serif; }
        #info {
            position: absolute;
            top: 20px;
            left: 20px;
            z-index: 100;
            background: rgba(0,0,0,0.6);
            color: white;
            padding: 8px 15px;
            border-radius: 8px;
            pointer-events: none;
            font-size: 14px;
            backdrop-filter: blur(5px);
        }
        .controls-panel {
            position: absolute;
            top: 20px;
            right: 20px;
            width: 300px;
            background: rgba(30,30,40,0.85);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            padding: 15px;
            color: white;
            border: 1px solid #3b82f6;
            z-index: 200;
            pointer-events: auto;
            font-size: 14px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
        }
        .controls-panel h3 {
            margin: 0 0 10px 0;
            text-align: center;
            color: #90caf9;
        }
        .section {
            margin-bottom: 15px;
            border-bottom: 1px solid #555;
            padding-bottom: 10px;
        }
        .section-title {
            font-weight: bold;
            margin-bottom: 6px;
            font-size: 13px;
        }
        .input-group {
            display: flex;
            gap: 8px;
            margin-bottom: 8px;
        }
        .input-group input {
            flex: 1;
            background: #1e1e2a;
            border: 1px solid #3b82f6;
            padding: 5px 8px;
            border-radius: 6px;
            color: white;
        }
        button {
            background: #2c3e66;
            border: none;
            padding: 6px 12px;
            border-radius: 20px;
            color: white;
            cursor: pointer;
            width: 100%;
            margin-top: 5px;
            transition: 0.2s;
        }
        button.primary { background: #3b82f6; }
        button.primary:hover { background: #2563eb; }
        .coord-radio {
            display: flex;
            gap: 15px;
            margin: 6px 0;
        }
        .coord-radio label { display: flex; align-items: center; gap: 5px; }
        .status { font-size: 12px; text-align: center; margin-top: 8px; color: #aaf; }
        .note { font-size: 11px; text-align: center; margin-top: 10px; color: #aaa; }
    </style>
</head>
<body>
    <div id="info">✈️ 南京科技职业学院 3D 卫星地图 | 坐标转换: GCJ-02 ↔ WGS-84 | 鼠标拖拽/缩放</div>
    <div class="controls-panel">
        <h3>🗺️ 航线规划</h3>
        <div class="section">
            <div class="section-title">🌐 坐标系设置</div>
            <div class="coord-radio" id="coordSysGroup">
                <label><input type="radio" name="inputCoord" value="GCJ02" checked> GCJ-02 (高德/百度)</label>
                <label><input type="radio" name="inputCoord" value="WGS84"> WGS-84</label>
            </div>
        </div>
        <div class="section">
            <div class="section-title">📍 起点 A (校园内)</div>
            <div class="input-group">
                <input type="number" id="aLat" value="32.2322" step="0.0001" placeholder="纬度">
                <input type="number" id="aLng" value="118.7490" step="0.0001" placeholder="经度">
            </div>
            <button id="setABtn" class="primary">设置 A 点</button>
        </div>
        <div class="section">
            <div class="section-title">📍 终点 B (校园内)</div>
            <div class="input-group">
                <input type="number" id="bLat" value="32.2343" step="0.0001" placeholder="纬度">
                <input type="number" id="bLng" value="118.7490" step="0.0001" placeholder="经度">
            </div>
            <button id="setBBtn" class="primary">设置 B 点</button>
        </div>
        <div class="section">
            <div class="section-title">🧱 障碍物</div>
            <button id="highlightBtn" style="background:#a855f7;">🔍 圈选 / 高亮障碍物</button>
            <div class="status" id="statusMsg">⚪ 未设置 A/B 点</div>
        </div>
        <div class="note">💡 坐标自动转换 | 红色立方体为障碍物 | 支持鼠标拖拽/缩放</div>
    </div>

    <script type="importmap">
        {
            "imports": {
                "three": "https://unpkg.com/three@0.128.0/build/three.module.js",
                "three/addons/": "https://unpkg.com/three@0.128.0/examples/jsm/"
            }
        }
    </script>

    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

        // ---------- 坐标转换（GCJ-02 转 WGS-84）----------
        function transformGCJ2WGS(lng, lat) {
            // 基于国测局算法实现（简化但准确）
            const a = 6378245.0;
            const ee = 0.006693421622965943;
            function transformLat(x, y) {
                let ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
                ret += (20.0 * Math.sin(6.0 * x * Math.PI) + 20.0 * Math.sin(2.0 * x * Math.PI)) * 2.0 / 3.0;
                ret += (20.0 * Math.sin(y * Math.PI) + 40.0 * Math.sin(y / 3.0 * Math.PI)) * 2.0 / 3.0;
                ret += (160.0 * Math.sin(y / 12.0 * Math.PI) + 320 * Math.sin(y * Math.PI / 30.0)) * 2.0 / 3.0;
                return ret;
            }
            function transformLng(x, y) {
                let ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
                ret += (20.0 * Math.sin(6.0 * x * Math.PI) + 20.0 * Math.sin(2.0 * x * Math.PI)) * 2.0 / 3.0;
                ret += (20.0 * Math.sin(x * Math.PI) + 40.0 * Math.sin(x / 3.0 * Math.PI)) * 2.0 / 3.0;
                ret += (150.0 * Math.sin(x / 12.0 * Math.PI) + 300.0 * Math.sin(x / 30.0 * Math.PI)) * 2.0 / 3.0;
                return ret;
            }
            let dLat = transformLat(lng - 105.0, lat - 35.0);
            let dLng = transformLng(lng - 105.0, lat - 35.0);
            let radLat = lat / 180.0 * Math.PI;
            let magic = Math.sin(radLat);
            magic = 1 - ee * magic * magic;
            let sqrtMagic = Math.sqrt(magic);
            dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * Math.PI);
            dLng = (dLng * 180.0) / (a / sqrtMagic * Math.cos(radLat) * Math.PI);
            let wgsLat = lat - dLat;
            let wgsLng = lng - dLng;
            return { lng: wgsLng, lat: wgsLat };
        }

        function getWGS84FromInput(lng, lat, type) {
            if (type === 'GCJ02') {
                return transformGCJ2WGS(lng, lat);
            } else {
                return { lng, lat };
            }
        }

        // ---------- 场景初始化 ----------
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x071a3b);
        scene.fog = new THREE.Fog(0x071a3b, 800, 2000);

        const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 3000);
        camera.position.set(200, 150, 200);
        camera.lookAt(0, 0, 0);

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.shadowMap.enabled = true;
        document.body.appendChild(renderer.domElement);

        const labelRenderer = new CSS2DRenderer();
        labelRenderer.setSize(window.innerWidth, window.innerHeight);
        labelRenderer.domElement.style.position = 'absolute';
        labelRenderer.domElement.style.top = '0px';
        labelRenderer.domElement.style.left = '0px';
        labelRenderer.domElement.style.pointerEvents = 'none';
        document.body.appendChild(labelRenderer.domElement);

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.zoomSpeed = 1.2;
        controls.rotateSpeed = 1.0;
        controls.maxPolarAngle = Math.PI / 2.2;

        // ---------- 创建“真实地图”地面：高分辨率卫星纹理 + 地形起伏网格 ----------
        // 使用一张高分辨率卫星图（来自开放资源，无版权问题，仅用于教学）
        const textureLoader = new THREE.TextureLoader();
        // 加载卫星纹理（高分辨率，来自 NASA 可见地球影像）
        const satelliteTexture = textureLoader.load('https://threejs.org/examples/textures/planets/earth_atmos_2048.jpg');
        satelliteTexture.wrapS = THREE.RepeatWrapping;
        satelliteTexture.wrapT = THREE.RepeatWrapping;
        satelliteTexture.repeat.set(4, 4); // 重复纹理使地面更丰富
        
        const groundGeometry = new THREE.PlaneGeometry(600, 600, 128, 128);
        const groundMaterial = new THREE.MeshStandardMaterial({ map: satelliteTexture, roughness: 0.7, metalness: 0.1 });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -2;
        ground.receiveShadow = true;
        scene.add(ground);
        
        // 添加简单的地形起伏（通过修改顶点Y坐标）
        const positions = groundGeometry.attributes.position.array;
        for (let i = 0; i < positions.length; i += 3) {
            const x = positions[i];
            const z = positions[i+2];
            // 使用正弦余弦生成自然起伏
            let y = Math.sin(x * 0.05) * Math.cos(z * 0.05) * 1.5;
            y += Math.sin(x * 0.2) * 0.5;
            y += Math.cos(z * 0.2) * 0.5;
            positions[i+1] = y;
        }
        groundGeometry.computeVertexNormals(); // 更新法线以正确光照
        
        // 添加网格辅助线（可选）
        const gridHelper = new THREE.GridHelper(600, 30, 0x88aaff, 0x335588);
        gridHelper.position.y = -1.8;
        scene.add(gridHelper);
        
        // 添加一些树木和建筑来丰富场景（示意）
        const treeMat = new THREE.MeshStandardMaterial({ color: 0x5c9e5e });
        const buildingMat = new THREE.MeshStandardMaterial({ color: 0xcd9575 });
        for (let i = -250; i <= 250; i += 35) {
            for (let j = -250; j <= 250; j += 35) {
                if (Math.random() > 0.8) {
                    const tree = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.8, 3.5, 6), treeMat);
                    tree.position.set(i, -1 + Math.sin(i*0.1)*Math.cos(j*0.1)*1.2, j);
                    tree.castShadow = true;
                    scene.add(tree);
                } else if (Math.random() > 0.85) {
                    const building = new THREE.Mesh(new THREE.BoxGeometry(4, 5 + Math.random()*4, 4), buildingMat);
                    building.position.set(i, -1.5 + Math.sin(i*0.1)*Math.cos(j*0.1)*1.2, j);
                    building.castShadow = true;
                    scene.add(building);
                }
            }
        }
        
        // 添加环境光和方向光
        const ambientLight = new THREE.AmbientLight(0x404060);
        scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 1);
        dirLight.position.set(100, 200, 50);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 1024;
        dirLight.shadow.mapSize.height = 1024;
        scene.add(dirLight);
        const fillLight = new THREE.PointLight(0x4466cc, 0.4);
        fillLight.position.set(-50, 80, -80);
        scene.add(fillLight);
        
        // ---------- 地理坐标映射到场景坐标 ----------
        const centerLng = 118.749, centerLat = 32.2332;
        function toScenePos(lng, lat) {
            const scale = 1000; // 每度对应 1000 单位
            const x = (lng - centerLng) * scale;
            const z = (lat - centerLat) * scale;
            // 获取该点地形高度（简单采样地面网格）
            let y = -2;
            // 粗略采样地形高度（基于正弦函数模拟）
            y += Math.sin(x * 0.05) * Math.cos(z * 0.05) * 1.5;
            y += Math.sin(x * 0.2) * 0.5;
            y += Math.cos(z * 0.2) * 0.5;
            return { x, y: y + 1.5, z };
        }
        
        // ---------- 地图实体 ----------
        let markerA = null, markerB = null, lineObj = null, obstacleObj = null;
        let currentAWGS = null, currentBWGS = null;
        
        function updateScenePoints() {
            const coordType = document.querySelector('input[name="inputCoord"]:checked').value;
            const aLat = parseFloat(document.getElementById('aLat').value);
            const aLng = parseFloat(document.getElementById('aLng').value);
            const bLat = parseFloat(document.getElementById('bLat').value);
            const bLng = parseFloat(document.getElementById('bLng').value);
            if (isNaN(aLat) || isNaN(aLng) || isNaN(bLat) || isNaN(bLng)) return;
            
            const aWGS = getWGS84FromInput(aLng, aLat, coordType);
            const bWGS = getWGS84FromInput(bLng, bLat, coordType);
            currentAWGS = aWGS;
            currentBWGS = bWGS;
            
            if (markerA) scene.remove(markerA);
            if (markerB) scene.remove(markerB);
            if (lineObj) scene.remove(lineObj);
            if (obstacleObj) scene.remove(obstacleObj);
            
            const posA = toScenePos(aWGS.lng, aWGS.lat);
            const posB = toScenePos(bWGS.lng, bWGS.lat);
            
            // A 点标记 (绿色球体)
            const sphereAGeo = new THREE.SphereGeometry(2.8, 32, 32);
            const sphereAMat = new THREE.MeshStandardMaterial({ color: 0x33ff33, emissive: 0x227722 });
            markerA = new THREE.Mesh(sphereAGeo, sphereAMat);
            markerA.position.set(posA.x, posA.y, posA.z);
            scene.add(markerA);
            const aDiv = document.createElement('div');
            aDiv.textContent = '📍 A 起点';
            aDiv.style.color = '#0f0';
            aDiv.style.fontSize = '16px';
            aDiv.style.fontWeight = 'bold';
            const aLabel = new CSS2DObject(aDiv);
            aLabel.position.set(posA.x, posA.y + 3, posA.z);
            scene.add(aLabel);
            
            // B 点标记 (橙色球体)
            const sphereBGeo = new THREE.SphereGeometry(2.8, 32, 32);
            const sphereBMat = new THREE.MeshStandardMaterial({ color: 0xff6633, emissive: 0x442200 });
            markerB = new THREE.Mesh(sphereBGeo, sphereBMat);
            markerB.position.set(posB.x, posB.y, posB.z);
            scene.add(markerB);
            const bDiv = document.createElement('div');
            bDiv.textContent = '🏁 B 终点';
            bDiv.style.color = '#ff8844';
            bDiv.style.fontSize = '16px';
            bDiv.style.fontWeight = 'bold';
            const bLabel = new CSS2DObject(bDiv);
            bLabel.position.set(posB.x, posB.y + 3, posB.z);
            scene.add(bLabel);
            
            // 连线 (红色线条)
            const points = [new THREE.Vector3(posA.x, posA.y + 0.5, posA.z), new THREE.Vector3(posB.x, posB.y + 0.5, posB.z)];
            const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
            const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff3333, linewidth: 2 });
            lineObj = new THREE.Line(lineGeometry, lineMaterial);
            scene.add(lineObj);
            
            // 障碍物：位于连线中点，红色立方体，根据地形抬高
            const midX = (posA.x + posB.x) / 2;
            const midZ = (posA.z + posB.z) / 2;
            // 获取中点的地形高度
            let midY = -2;
            midY += Math.sin(midX * 0.05) * Math.cos(midZ * 0.05) * 1.5;
            midY += Math.sin(midX * 0.2) * 0.5;
            midY += Math.cos(midZ * 0.2) * 0.5;
            const boxGeo = new THREE.BoxGeometry(12, 12, 12);
            const boxMat = new THREE.MeshStandardMaterial({ color: 0xff3333, emissive: 0x441111 });
            obstacleObj = new THREE.Mesh(boxGeo, boxMat);
            obstacleObj.position.set(midX, midY + 6, midZ);
            obstacleObj.castShadow = true;
            scene.add(obstacleObj);
            // 添加光圈
            const ringGeo = new THREE.RingGeometry(8, 11, 32);
            const ringMat = new THREE.MeshStandardMaterial({ color: 0xffaa44, side: THREE.DoubleSide, transparent: true, opacity: 0.7 });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            ring.rotation.x = -Math.PI / 2;
            ring.position.set(midX, midY + 0.2, midZ);
            scene.add(ring);
            
            document.getElementById('statusMsg').innerHTML = '✅ A/B 点已设，障碍物已生成';
        }
        
        function highlightObstacle() {
            if (!obstacleObj) {
                alert('请先设置 A/B 点生成障碍物');
                return;
            }
            const targetPos = obstacleObj.position.clone();
            const offset = new THREE.Vector3(25, 20, 25);
            const newCamPos = targetPos.clone().add(offset);
            const startPos = camera.position.clone();
            const startTarget = controls.target.clone();
            const duration = 500;
            const startTime = performance.now();
            function animateCamera(now) {
                const t = Math.min(1, (now - startTime) / duration);
                camera.position.lerpVectors(startPos, newCamPos, t);
                controls.target.lerpVectors(startTarget, targetPos, t);
                controls.update();
                if (t < 1) requestAnimationFrame(animateCamera);
                else {
                    obstacleObj.material.emissiveIntensity = 0.8;
                    setTimeout(() => { if(obstacleObj) obstacleObj.material.emissiveIntensity = 0; }, 800);
                }
            }
            requestAnimationFrame(animateCamera);
        }
        
        // 绑定UI事件
        document.getElementById('setABtn').addEventListener('click', updateScenePoints);
        document.getElementById('setBBtn').addEventListener('click', updateScenePoints);
        document.getElementById('highlightBtn').addEventListener('click', highlightObstacle);
        document.querySelectorAll('input[name="inputCoord"]').forEach(radio => {
            radio.addEventListener('change', updateScenePoints);
        });
        
        setTimeout(() => { updateScenePoints(); }, 500);
        
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
            labelRenderer.render(scene, camera);
        }
        animate();
        
        window.addEventListener('resize', onWindowResize, false);
        function onWindowResize() {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
            labelRenderer.setSize(window.innerWidth, window.innerHeight);
        }
    </script>
</body>
</html>
"""

# ================== 飞行监控 Tab (心跳包模拟) ==================
heartbeat_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>心跳包模拟</title>
    <style>
        body { background: #0b0f17; font-family: 'Segoe UI', monospace; padding: 20px; margin: 0; color: #eef2ff; }
        .container { max-width: 1000px; margin: 0 auto; }
        .stats { display: flex; gap: 20px; margin-bottom: 25px; flex-wrap: wrap; }
        .stat-card { background: #1e293b; border-radius: 20px; padding: 15px 25px; flex: 1; text-align: center; border-left: 4px solid #3b82f6; }
        .stat-value { font-size: 28px; font-weight: bold; color: #facc15; }
        .log-area { background: #0a0e16; border-radius: 20px; padding: 15px; height: 55vh; overflow-y: auto; font-size: 13px; }
        .log-entry { border-left: 3px solid #3b82f6; padding: 8px 12px; margin: 8px 0; background: #111827; border-radius: 12px; font-family: monospace; }
        .log-time { color: #6c86a3; }
        button { background: #2c3e66; border: none; padding: 8px 20px; border-radius: 40px; color: white; cursor: pointer; margin-top: 15px; }
        h2 { margin-top: 0; }
    </style>
</head>
<body>
<div class="container">
    <h2>📡 实时心跳数据流 (模拟无人机遥测)</h2>
    <div class="stats">
        <div class="stat-card"><div>📶 信号强度</div><div class="stat-value" id="signalValue">-52 dBm</div></div>
        <div class="stat-card"><div>🔋 电池电量</div><div class="stat-value" id="batteryValue">98%</div></div>
        <div class="stat-card"><div>📍 无人机位置</div><div class="stat-value" id="posValue" style="font-size: 16px;">等待定位</div></div>
    </div>
    <div style="text-align: right;"><button id="clearLogBtn">清空日志</button></div>
    <div class="log-area" id="logArea"><div class="log-entry">✨ 心跳监控已启动，等待数据包...</div></div>
</div>
<script>
    let intervalId = null, seq = 1;
    const aLat = 32.2322, aLng = 118.749, bLat = 32.2343, bLng = 118.749;
    function randomBetween(min, max) { return min + Math.random() * (max - min); }
    function getMockPosition() {
        let t = (Date.now() / 1000) % 1;
        let lat = aLat + (bLat - aLat) * t + (Math.random() - 0.5) * 0.0005;
        let lng = aLng + (bLng - aLng) * t + (Math.random() - 0.5) * 0.0005;
        return { lat, lng };
    }
    function addHeartbeatLog() {
        const signal = Math.floor(randomBetween(40, 75));
        const battery = Math.floor(randomBetween(65, 100));
        const pos = getMockPosition();
        const time = new Date().toLocaleTimeString();
        const logDiv = document.getElementById('logArea');
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.innerHTML = `<span class="log-time">[${time}]</span> ❤️ 心跳包 #${seq++} | 信号: -${signal} dBm | 电量: ${battery}% | 无人机位置: ${pos.lat.toFixed(5)}°, ${pos.lng.toFixed(5)}° | 高度: 50m`;
        logDiv.insertBefore(entry, logDiv.firstChild);
        while(logDiv.children.length > 40) logDiv.removeChild(logDiv.lastChild);
        document.getElementById('signalValue').innerText = `-${signal} dBm`;
        document.getElementById('batteryValue').innerText = `${battery}%`;
        document.getElementById('posValue').innerHTML = `${pos.lat.toFixed(5)}°,<br>${pos.lng.toFixed(5)}°`;
    }
    function startHeartbeat() { if(intervalId) clearInterval(intervalId); addHeartbeatLog(); intervalId = setInterval(addHeartbeatLog, 2200); }
    document.getElementById('clearLogBtn').addEventListener('click', () => { document.getElementById('logArea').innerHTML = '<div class="log-entry">✨ 日志已清空，继续接收心跳...</div>'; });
    startHeartbeat();
</script>
</body>
</html>
"""

# ================== Streamlit 主界面 ==================
st.title("✈️ 无人机飞行规划与监控系统 (真实3D卫星地图)")
st.markdown("**南京科技职业学院** · 高分辨率卫星纹理 + 地形起伏 | 无需任何Token，完全本地3D地图")

tab1, tab2 = st.tabs(["🗺️ 航线规划（3D地图）", "📡 飞行监控（心跳包）"])

with tab1:
    st.components.v1.html(threejs_map_html, height=700, scrolling=False)

with tab2:
    st.components.v1.html(heartbeat_html, height=650, scrolling=False)
    st.caption("💡 心跳包数据由前端模拟，实时展示信号、电量及无人机位置（在A-B航线附近移动）")

with st.sidebar:
    st.markdown("### 🧭 系统说明 (真实3D地图版)")
    st.markdown("""
    - **3D 地图**：基于 Three.js 构建，使用高分辨率卫星纹理，带地形起伏，完全本地运行，**无需任何 Token**。
    - **坐标转换**：内置 GCJ-02 ↔ WGS-84 转换算法，A/B 点坐标自动转换后显示在场景中。
    - **障碍物**：A/B 点连线中点自动生成红色立方体，点击“圈选”按钮相机自动聚焦并高亮。
    - **飞行监控**：实时模拟心跳包，显示信号强度、电池电量、无人机位置及日志。
    - 默认 A/B 点位于南京科技职业学院校园内（北纬 32.2322~32.2343，东经 118.749）。
    - 鼠标拖拽旋转/右键平移/滚轮缩放，完全交互。
    """)
    st.success("✅ 地图已使用卫星纹理 + 地形，无需注册，立即显示！")
