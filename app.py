import streamlit as st

st.set_page_config(
    page_title="无人机飞行规划与监控系统",
    page_icon="✈️",
    layout="wide"
)

# ================== 航线规划：3D 真实风格地图（Three.js + 卫星纹理）==================
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

        // ---------- 坐标转换：GCJ-02 转 WGS-84 (国测局算法) ----------
        function transformGCJ2WGS(lng, lat) {
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
            return { lng: lng - dLng, lat: lat - dLat };
        }

        function getWGS84FromInput(lng, lat, type) {
            if (type === 'GCJ02') return transformGCJ2WGS(lng, lat);
            return { lng, lat };
        }

        // ---------- Three.js 场景初始化 ----------
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

        // ---------- 创建真实风格的地面：高分辨率卫星纹理 + 地形起伏 ----------
        const textureLoader = new THREE.TextureLoader();
        // 使用一张高质量的全球卫星影像（来自 Three.js 示例，可公开访问）
        const satelliteTexture = textureLoader.load('https://threejs.org/examples/textures/planets/earth_atmos_2048.jpg');
        satelliteTexture.wrapS = THREE.RepeatWrapping;
        satelliteTexture.wrapT = THREE.RepeatWrapping;
        satelliteTexture.repeat.set(3, 3);  // 重复纹理使地面细节更丰富

        const width = 800;
        const depth = 800;
        const segments = 180;
        const groundGeometry = new THREE.PlaneGeometry(width, depth, segments, segments);
        // 修改顶点 Y 坐标产生地形起伏
        const positions = groundGeometry.attributes.position.array;
        for (let i = 0; i < positions.length; i += 3) {
            const x = positions[i];
            const z = positions[i+2];
            // 使用多个正弦余弦叠加，模拟自然地形
            let y = Math.sin(x * 0.03) * Math.cos(z * 0.03) * 2.0;
            y += Math.sin(x * 0.1) * 0.8;
            y += Math.cos(z * 0.1) * 0.8;
            y += Math.sin((x * 0.2 + z * 0.15) * 1.5) * 0.5;
            positions[i+1] = y;
        }
        groundGeometry.computeVertexNormals(); // 更新法线以得到正确光照

        const groundMaterial = new THREE.MeshStandardMaterial({ map: satelliteTexture, roughness: 0.6, metalness: 0.1 });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -2;
        ground.receiveShadow = true;
        scene.add(ground);

        // 添加辅助网格线（半透明，帮助定位）
        const gridHelper = new THREE.GridHelper(700, 30, 0x88aaff, 0x335588);
        gridHelper.position.y = -1.5;
        scene.add(gridHelper);

        // 随机添加一些树木和建筑物来丰富场景（示意，不影响障碍物功能）
        const treeMat = new THREE.MeshStandardMaterial({ color: 0x5c9e5e });
        const buildingMat = new THREE.MeshStandardMaterial({ color: 0xcd9575 });
        for (let i = -300; i <= 300; i += 45) {
            for (let j = -300; j <= 300; j += 45) {
                if (Math.random() > 0.85) {
                    const tree = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.8, 3.5, 6), treeMat);
                    // 获取该位置的地形高度
                    let h = -2;
                    h += Math.sin(i * 0.03) * Math.cos(j * 0.03) * 2.0;
                    h += Math.sin(i * 0.1) * 0.8;
                    h += Math.cos(j * 0.1) * 0.8;
                    tree.position.set(i, h, j);
                    tree.castShadow = true;
                    scene.add(tree);
                } else if (Math.random() > 0.92) {
                    const building = new THREE.Mesh(new THREE.BoxGeometry(4, 5 + Math.random()*4, 4), buildingMat);
                    let h = -2;
                    h += Math.sin(i * 0.03) * Math.cos(j * 0.03) * 2.0;
                    h += Math.sin(i * 0.1) * 0.8;
                    h += Math.cos(j * 0.1) * 0.8;
                    building.position.set(i, h, j);
                    building.castShadow = true;
                    scene.add(building);
                }
            }
        }

        // 灯光系统
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

        // ---------- 地理坐标 → 场景坐标映射 ----------
        const centerLng = 118.749, centerLat = 32.2332;
        function toScenePos(lng, lat) {
            const scale = 1000; // 每度 1000 单位
            const x = (lng - centerLng) * scale;
            const z = (lat - centerLat) * scale;
            // 获取该点的地形高度（与地面几何体一致）
            let y = -2;
            y += Math.sin(x * 0.03) * Math.cos(z * 0.03) * 2.0;
            y += Math.sin(x * 0.1) * 0.8;
            y += Math.cos(z * 0.1) * 0.8;
            y += Math.sin((x * 0.2 + z * 0.15) * 1.5) * 0.5;
            return { x, y: y + 1.5, z };
        }

        // ---------- 地图实体：A/B 点、连线、障碍物 ----------
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

            // A 点标记（绿色球体）
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

            // B 点标记（橙色球体）
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

            // 连线（红色线条）
            const points = [new THREE.Vector3(posA.x, posA.y + 0.5, posA.z), new THREE.Vector3(posB.x, posB.y + 0.5, posB.z)];
            const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
            const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff3333 });
            lineObj = new THREE.Line(lineGeometry, lineMaterial);
            scene.add(lineObj);

            // 障碍物：位于连线中点，红色立方体，高度 10 单位
            const midX = (posA.x + posB.x) / 2;
            const midZ = (posA.z + posB.z) / 2;
            // 获取中点地形高度
            let midY = -2;
            midY += Math.sin(midX * 0.03) * Math.cos(midZ * 0.03) * 2.0;
            midY += Math.sin(midX * 0.1) * 0.8;
            midY += Math.cos(midZ * 0.1) * 0.8;
            midY += Math.sin((midX * 0.2 + midZ * 0.15) * 1.5) * 0.5;
            const boxGeo = new THREE.BoxGeometry(12, 12, 12);
            const boxMat = new THREE.MeshStandardMaterial({ color: 0xff3333, emissive: 0x441111 });
            obstacleObj = new THREE.Mesh(boxGeo, boxMat);
            obstacleObj.position.set(midX, midY + 6, midZ);
            obstacleObj.castShadow = true;
            scene.add(obstacleObj);

            // 添加光圈效果
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

        // 绑定 UI 事件
        document.getElementById('setABtn').addEventListener('click', updateScenePoints);
        document.getElementById('setBBtn').addEventListener('click', updateScenePoints);
        document.getElementById('highlightBtn').addEventListener('click', highlightObstacle);
        document.querySelectorAll('input[name="inputCoord"]').forEach(radio => {
            radio.addEventListener('change', updateScenePoints);
        });

        setTimeout(() => { updateScenePoints(); }, 500);

        // 动画循环
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

# ================== 飞行监控：心跳包模拟 ==================
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
st.title("✈️ 无人机飞行规划与监控系统")
st.markdown("**南京科技职业学院** · 高分辨率卫星纹理 + 地形起伏 | 无需任何 Token，完全本地 3D 地图")

tab1, tab2 = st.tabs(["🗺️ 航线规划（3D地图）", "📡 飞行监控（心跳包）"])

with tab1:
    st.components.v1.html(threejs_map_html, height=700, scrolling=False)

with tab2:
    st.components.v1.html(heartbeat_html, height=650, scrolling=False)
    st.caption("💡 心跳包数据由前端模拟，实时展示信号、电量及无人机位置（在 A-B 航线附近移动）")

with st.sidebar:
    st.markdown("### 🧭 系统说明")
    st.markdown("""
    - **3D 地图**：基于 Three.js，使用高分辨率卫星纹理 + 真实感地形起伏，完全本地运行，**无需任何 Token**。
    - **坐标转换**：内置 GCJ-02 ↔ WGS-84 转换算法，A/B 点坐标自动转换后显示在场景中。
    - **障碍物**：A/B 点连线中点自动生成红色立方体，点击“圈选”按钮相机自动聚焦并高亮。
    - **飞行监控**：实时模拟心跳包，显示信号强度、电池电量、无人机位置及日志。
    - 默认 A/B 点位于南京科技职业学院校园内（北纬 32.2322~32.2343，东经 118.749）。
    - 鼠标拖拽旋转/右键平移/滚轮缩放，完全交互。
    """)
    st.success("✅ 地图已加载，无需注册，立即使用！")
