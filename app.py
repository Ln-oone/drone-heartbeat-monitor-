import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
import shutil
import logging
from datetime import datetime
from typing import List, Dict, Tuple, Optional
import pandas as pd
import numpy as np
from collections import deque
from dataclasses import dataclass, asdict
from enum import Enum

# ==================== 日志配置 ====================
if not logging.getLogger().handlers:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('flight_system.log'),
            logging.StreamHandler()
        ]
    )
logger = logging.getLogger(__name__)

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="南京科技职业学院 - 无人机地面站系统", 
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==================== 常量定义 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

CONFIG_FILE = "obstacle_config.json"
BACKUP_DIR = "backups"
DEFAULT_SAFETY_RADIUS_METERS = 5
DEFAULT_FLIGHT_ALTITUDE = 50
MAX_FLIGHT_ALTITUDE = 200
MIN_FLIGHT_ALTITUDE = 10
MAX_HISTORY_SIZE = 100
MAX_FLIGHT_LOG_SIZE = 1000
MAX_BACKUP_FILES = 10

GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

os.makedirs(BACKUP_DIR, exist_ok=True)

# ==================== 枚举类型 ====================
class FlightDirection(Enum):
    BEST = "最佳航线"
    LEFT = "向左绕行"
    RIGHT = "向右绕行"

class MapType(Enum):
    SATELLITE = "satellite"
    VECTOR = "vector"

# ==================== 数据模型 ====================
@dataclass
class HeartbeatData:
    timestamp: str
    flight_time: float
    lat: float
    lng: float
    altitude: float
    voltage: float
    satellites: int
    speed: float
    progress: float
    arrived: bool
    safety_violation: bool
    remaining_distance: float

@dataclass
class Obstacle:
    name: str
    polygon: List[List[float]]
    height: float = 30.0
    
    def to_dict(self) -> Dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Obstacle':
        return cls(
            name=data.get('name', '未命名'),
            polygon=data.get('polygon', []),
            height=data.get('height', 30.0)
        )

# ==================== 几何计算器 ====================
class GeometryCalculator:
    """几何计算工具类"""
    
    @staticmethod
    def point_in_polygon(point: List[float], polygon: List[List[float]]) -> bool:
        """判断点是否在多边形内"""
        x, y = point
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
        return inside
    
    @staticmethod
    def segments_intersect(p1: List[float], p2: List[float], 
                          p3: List[float], p4: List[float]) -> bool:
        """判断两线段是否相交"""
        def orientation(p: List[float], q: List[float], r: List[float]) -> int:
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if abs(val) < 1e-10:
                return 0
            return 1 if val > 0 else 2
        
        def on_segment(p: List[float], q: List[float], r: List[float]) -> bool:
            return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                    min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))
        
        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)
        
        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and on_segment(p1, p3, p2):
            return True
        if o2 == 0 and on_segment(p1, p4, p2):
            return True
        if o3 == 0 and on_segment(p3, p1, p4):
            return True
        if o4 == 0 and on_segment(p3, p2, p4):
            return True
        return False
    
    @staticmethod
    def line_intersects_polygon(p1: List[float], p2: List[float], 
                               polygon: List[List[float]]) -> bool:
        """判断线段是否与多边形相交"""
        if GeometryCalculator.point_in_polygon(p1, polygon) or \
           GeometryCalculator.point_in_polygon(p2, polygon):
            return True
        
        n = len(polygon)
        for i in range(n):
            p3 = polygon[i]
            p4 = polygon[(i + 1) % n]
            if GeometryCalculator.segments_intersect(p1, p2, p3, p4):
                return True
        return False
    
    @staticmethod
    def distance(p1: List[float], p2: List[float]) -> float:
        """计算两点间距离（度数）"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    @staticmethod
    def distance_meters(p1: List[float], p2: List[float]) -> float:
        """计算两点间距离（米）"""
        lat_mid = (p1[1] + p2[1]) / 2
        return GeometryCalculator.distance(p1, p2) * 111000 * math.cos(math.radians(lat_mid))
    
    @staticmethod
    def meters_to_deg(meters: float, lat: float = 32.23) -> Tuple[float, float]:
        """将米转换为度数"""
        lat_rad = math.radians(lat)
        lat_deg = meters / 111000
        lng_deg = meters / (111000 * math.cos(lat_rad))
        return lng_deg, lat_deg
    
    @staticmethod
    def get_polygon_bounds(polygon: List[List[float]]) -> Optional[Dict]:
        """获取多边形边界"""
        if not polygon:
            return None
        min_lng = min(p[0] for p in polygon)
        max_lng = max(p[0] for p in polygon)
        min_lat = min(p[1] for p in polygon)
        max_lat = max(p[1] for p in polygon)
        return {
            'min_lng': min_lng, 'max_lng': max_lng,
            'min_lat': min_lat, 'max_lat': max_lat,
            'center_lng': (min_lng + max_lng) / 2,
            'center_lat': (min_lat + max_lat) / 2
        }

# ==================== 避障算法 ====================
class ObstacleAvoider:
    """障碍物避让算法"""
    
    def __init__(self, safety_radius: float = 5.0):
        self.safety_radius = safety_radius
        self.geometry = GeometryCalculator()
    
    def get_blocking_obstacles(self, start: List[float], end: List[float], 
                               obstacles: List[Obstacle], flight_altitude: float) -> List[Obstacle]:
        """获取阻挡路径的障碍物"""
        blocking = []
        for obs in obstacles:
            if obs.height > flight_altitude:
                if self.geometry.line_intersects_polygon(start, end, obs.polygon):
                    blocking.append(obs)
        return blocking
    
    def find_left_boundary_path(self, start: List[float], end: List[float], 
                               obstacles: List[Obstacle]) -> List[List[float]]:
        """向左绕行 - 严格沿障碍物左侧边界绕行"""
        if not obstacles:
            return [start, end]
        
        # 收集所有边界点
        all_boundary_points = []
        for obs in obstacles:
            all_boundary_points.extend(obs.polygon)
        
        if not all_boundary_points:
            return [start, end]
        
        # 计算安全偏移量
        offset_lng, offset_lat = self.geometry.meters_to_deg(self.safety_radius * 1.5, start[1])
        
        # 获取最左侧X坐标
        leftmost_x = min(p[0] for p in all_boundary_points)
        boundary_x = leftmost_x - offset_lng
        
        # 获取Y范围
        min_y = min(p[1] for p in all_boundary_points)
        max_y = max(p[1] for p in all_boundary_points)
        
        # 构建绕行路径
        full_path = [start.copy()]
        
        # 水平移动到左侧边界
        waypoint1 = [boundary_x, start[1]]
        full_path.append(waypoint1)
        
        # 垂直移动绕过障碍物
        if end[1] > start[1]:
            waypoint2 = [boundary_x, max_y + offset_lat * 2]
        else:
            waypoint2 = [boundary_x, min_y - offset_lat * 2]
        full_path.append(waypoint2)
        
        # 水平移动到终点X坐标
        waypoint3 = [end[0], waypoint2[1]]
        full_path.append(waypoint3)
        
        # 垂直移动到终点
        full_path.append(end.copy())
        
        # 去重
        unique_path = []
        for point in full_path:
            if not unique_path:
                unique_path.append(point)
            else:
                last = unique_path[-1]
                if abs(last[0] - point[0]) > 1e-10 or abs(last[1] - point[1]) > 1e-10:
                    unique_path.append(point)
        
        return unique_path
    
    def find_right_path(self, start: List[float], end: List[float], 
                       obstacles: List[Obstacle], flight_altitude: float) -> List[List[float]]:
        """向右绕行"""
        blocking_obs = self.get_blocking_obstacles(start, end, obstacles, flight_altitude)
        
        if not blocking_obs:
            return [start, end]
        
        # 收集所有边界点
        all_boundary_points = []
        for obs in blocking_obs:
            all_boundary_points.extend(obs.polygon)
        
        if not all_boundary_points:
            return [start, end]
        
        # 获取最右侧X坐标
        rightmost_x = max(p[0] for p in all_boundary_points)
        offset_lng, _ = self.geometry.meters_to_deg(self.safety_radius * 2, start[1])
        right_x = rightmost_x + offset_lng
        
        # 创建绕行点
        waypoint = [right_x, (start[1] + end[1]) / 2]
        
        return [start, waypoint, end]
    
    def create_avoidance_path(self, start: List[float], end: List[float],
                             obstacles: List[Obstacle], flight_altitude: float,
                             direction: FlightDirection) -> List[List[float]]:
        """创建避障路径"""
        high_obstacles = [obs for obs in obstacles if obs.height > flight_altitude]
        
        if direction == FlightDirection.LEFT:
            return self.find_left_boundary_path(start, end, high_obstacles)
        elif direction == FlightDirection.RIGHT:
            return self.find_right_path(start, end, high_obstacles, flight_altitude)
        else:  # BEST
            left_path = self.find_left_boundary_path(start, end, high_obstacles)
            right_path = self.find_right_path(start, end, high_obstacles, flight_altitude)
            
            left_len = sum(self.geometry.distance(left_path[i], left_path[i+1]) 
                          for i in range(len(left_path)-1))
            right_len = sum(self.geometry.distance(right_path[i], right_path[i+1]) 
                           for i in range(len(right_path)-1))
            
            return left_path if left_len <= right_len else right_path

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj: List[float]):
        self.history = deque(maxlen=MAX_HISTORY_SIZE)
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = DEFAULT_FLIGHT_ALTITUDE
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.safety_violation = False
        self.start_time = None
        self.flight_log = deque(maxlen=MAX_FLIGHT_LOG_SIZE)
        self.last_update_time = None
        self.geometry = GeometryCalculator()
        
    def set_path(self, path: List[List[float]], altitude: float = 50, 
                 speed: float = 50, safety_radius: float = 5):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.safety_radius = safety_radius
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.safety_violation = False
        self.start_time = datetime.now()
        self.last_update_time = None
        
        self.total_distance = 0.0
        for i in range(len(path) - 1):
            self.total_distance += self.geometry.distance(path[i], path[i + 1])
        
        logger.info(f"飞行任务开始: 总距离={self.total_distance*111000:.1f}m, 高度={altitude}m")
    
    def update_and_generate(self, obstacles: List[Obstacle]) -> Optional[HeartbeatData]:
        if not self.simulating or self.path_index >= len(self.path) - 1:
            if self.simulating:
                self.simulating = False
                logger.info("飞行任务完成")
            return None
        
        current_time = time.time()
        if self.last_update_time is None:
            delta_time = 0.2
        else:
            delta_time = min(0.5, current_time - self.last_update_time)
        self.last_update_time = current_time
        
        start = self.path[self.path_index]
        end = self.path[self.path_index + 1]
        segment_distance = self.geometry.distance(start, end)
        
        speed_m_per_s = 5 * (self.speed / 100)
        move_distance = speed_m_per_s * delta_time
        
        self.distance_traveled += move_distance
        
        if self.total_distance > 0:
            self.progress = min(1.0, self.distance_traveled / self.total_distance)
        
        if self.distance_traveled >= segment_distance and self.distance_traveled > 0:
            self.path_index += 1
            self.distance_traveled = 0
            if self.path_index < len(self.path):
                self.current_pos = self.path[self.path_index].copy()
            else:
                self.simulating = False
                logger.info("到达目的地")
                return self._generate_heartbeat(True)
        else:
            if segment_distance > 0:
                t = min(1.0, max(0.0, self.distance_traveled / segment_distance))
                lng = start[0] + (end[0] - start[0]) * t
                lat = start[1] + (end[1] - start[1]) * t
                self.current_pos = [lng, lat]
        
        return self._generate_heartbeat(False)
    
    def _generate_heartbeat(self, arrived: bool = False) -> HeartbeatData:
        flight_time = (datetime.now() - self.start_time).total_seconds() if self.start_time else 0
        
        heartbeat = HeartbeatData(
            timestamp=datetime.now().strftime("%H:%M:%S"),
            flight_time=flight_time,
            lat=self.current_pos[1],
            lng=self.current_pos[0],
            altitude=self.flight_altitude,
            voltage=round(22.2 + random.uniform(-0.5, 0.5), 1),
            satellites=random.randint(8, 14),
            speed=round(5 * (self.speed / 100), 1),
            progress=self.progress if self.total_distance > 0 else 0,
            arrived=arrived,
            safety_violation=self.safety_violation,
            remaining_distance=max(0, self.total_distance - self.distance_traveled) * 111000
        )
        
        self.history.appendleft(heartbeat)
        self.flight_log.append(heartbeat)
        
        return heartbeat
    
    def export_flight_data(self) -> pd.DataFrame:
        return pd.DataFrame([asdict(h) for h in self.flight_log])

# ==================== 障碍物管理器 ====================
class ObstacleManager:
    """障碍物管理器"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.obstacles: List[Obstacle] = []
        self.geometry = GeometryCalculator()
        self.load()
    
    def load(self) -> bool:
        """加载障碍物配置"""
        if not os.path.exists(self.config_file):
            logger.warning(f"配置文件 {self.config_file} 不存在")
            return False
        
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                obstacles_data = data.get('obstacles', [])
                
                self.obstacles = []
                for obs_data in obstacles_data:
                    try:
                        obstacle = Obstacle.from_dict(obs_data)
                        if len(obstacle.polygon) >= 3:
                            self.obstacles.append(obstacle)
                        else:
                            logger.warning(f"跳过无效障碍物: {obstacle.name}")
                    except Exception as e:
                        logger.error(f"解析障碍物失败: {e}")
                
                logger.info(f"成功加载 {len(self.obstacles)} 个障碍物")
                return True
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析失败: {e}")
            return False
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return False
    
    def save(self) -> bool:
        """保存障碍物配置"""
        try:
            self._backup()
            
            data = {
                'obstacles': [obs.to_dict() for obs in self.obstacles],
                'count': len(self.obstacles),
                'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'version': 'v27.6'
            }
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            logger.info(f"成功保存 {len(self.obstacles)} 个障碍物")
            return True
            
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")
            return False
    
    def add(self, obstacle: Obstacle) -> bool:
        """添加障碍物"""
        try:
            if len(obstacle.polygon) < 3:
                logger.error("多边形至少需要3个顶点")
                return False
            self.obstacles.append(obstacle)
            return True
        except Exception as e:
            logger.error(f"添加障碍物失败: {e}")
            return False
    
    def remove(self, index: int) -> bool:
        """移除障碍物"""
        try:
            if 0 <= index < len(self.obstacles):
                self.obstacles.pop(index)
                return True
            return False
        except Exception as e:
            logger.error(f"移除障碍物失败: {e}")
            return False
    
    def update(self, index: int, obstacle: Obstacle) -> bool:
        """更新障碍物"""
        try:
            if 0 <= index < len(self.obstacles):
                self.obstacles[index] = obstacle
                return True
            return False
        except Exception as e:
            logger.error(f"更新障碍物失败: {e}")
            return False
    
    def clear(self) -> bool:
        """清除所有障碍物"""
        try:
            self.obstacles.clear()
            return True
        except Exception as e:
            logger.error(f"清除障碍物失败: {e}")
            return False
    
    def _backup(self) -> Optional[str]:
        """备份配置文件"""
        if os.path.exists(self.config_file):
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            backup_name = f"{BACKUP_DIR}/{self.config_file}.{timestamp}.bak"
            try:
                shutil.copy(self.config_file, backup_name)
                logger.info(f"配置文件已备份到: {backup_name}")
                self._cleanup_old_backups()
                return backup_name
            except Exception as e:
                logger.error(f"备份失败: {e}")
        return None
    
    def _cleanup_old_backups(self):
        """清理旧备份文件"""
        try:
            backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(self.config_file)]
            if len(backup_files) > MAX_BACKUP_FILES:
                backup_files.sort()
                for old_file in backup_files[:-MAX_BACKUP_FILES]:
                    os.remove(os.path.join(BACKUP_DIR, old_file))
                    logger.info(f"已删除旧备份: {old_file}")
        except Exception as e:
            logger.error(f"清理备份文件失败: {e}")
    
    def get_high_obstacles(self, flight_altitude: float) -> List[Obstacle]:
        """获取高于飞行高度的障碍物"""
        return [obs for obs in self.obstacles if obs.height > flight_altitude]
    
    def is_straight_path_blocked(self, start: List[float], end: List[float], 
                                 flight_altitude: float) -> bool:
        """检查直线路径是否被阻挡"""
        high_obstacles = self.get_high_obstacles(flight_altitude)
        for obs in high_obstacles:
            if self.geometry.line_intersects_polygon(start, end, obs.polygon):
                return True
        return True if high_obstacles else False

# ==================== 地图管理器 ====================
class MapManager:
    """地图管理器"""
    
    def __init__(self, geometry: GeometryCalculator):
        self.geometry = geometry
    
    def create_planning_map(self, center_gcj: List[float], points_gcj: Dict,
                           obstacles: List[Obstacle], flight_history: List = None,
                           planned_path: List = None, map_type: str = "satellite",
                           flight_altitude: float = 50, drone_pos: List = None,
                           direction: FlightDirection = FlightDirection.BEST,
                           safety_radius: float = 5) -> folium.Map:
        """创建规划地图"""
        if map_type == "satellite":
            tiles = GAODE_SATELLITE_URL
            attr = "高德卫星地图"
        else:
            tiles = GAODE_VECTOR_URL
            attr = "高德矢量地图"
        
        m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=17, 
                      tiles=tiles, attr=attr)
        
        # 添加绘图工具
        draw = plugins.Draw(
            export=True, position='topleft',
            draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 
                                     'color': '#ff0000', 'fillColor': '#ff0000', 
                                     'fillOpacity': 0.4},
                         'polyline': False, 'rectangle': False, 'circle': False, 
                         'marker': False, 'circlemarker': False},
            edit_options={'edit': True, 'remove': True}
        )
        m.add_child(draw)
        
        # 添加障碍物
        for obs in obstacles:
            coords = obs.polygon
            if coords and len(coords) >= 3:
                if obs.height > flight_altitude:
                    color = "red"
                    fill_opacity = 0.5
                    popup = f"🚧 {obs.name}\n⚠️ 高度: {obs.height}m (需避让)"
                else:
                    color = "orange"
                    fill_opacity = 0.3
                    popup = f"🏢 {obs.name}\n高度: {obs.height}m (安全)"
                
                folium.Polygon([[c[1], c[0]] for c in coords], 
                              color=color, weight=3, fill=True, 
                              fill_color=color, fill_opacity=fill_opacity, 
                              popup=popup).add_to(m)
        
        # 添加起点终点
        if points_gcj.get('A'):
            folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], 
                         popup="🟢 起点", 
                         icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
        if points_gcj.get('B'):
            folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], 
                         popup="🔴 终点", 
                         icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
        
        # 添加规划路径
        if planned_path and len(planned_path) > 1:
            path_locations = [[p[1], p[0]] for p in planned_path]
            
            if direction == FlightDirection.LEFT:
                line_color = "purple"
                line_label = f"向左绕行（{len(planned_path)-2}个绕行点）"
            elif direction == FlightDirection.RIGHT:
                line_color = "orange"
                line_label = f"向右绕行（{len(planned_path)-2}个绕行点）"
            else:
                line_color = "green"
                line_label = "最佳航线"
            
            folium.PolyLine(path_locations, color=line_color, weight=4, 
                          opacity=0.9, popup=f"✈️ {line_label}").add_to(m)
            
            # 添加绕行点标记
            for i, point in enumerate(planned_path[1:-1]):
                folium.CircleMarker([point[1], point[0]], radius=7, color=line_color, 
                                  fill=True, fill_color="white", fill_opacity=0.9, 
                                  popup=f"绕行点 {i+1}").add_to(m)
        
        # 添加无人机位置
        if drone_pos:
            folium.Circle(
                radius=safety_radius,
                location=[drone_pos[1], drone_pos[0]],
                color="blue",
                weight=2,
                fill=True,
                fill_color="blue",
                fill_opacity=0.2,
                popup=f"🛡️ 安全半径: {safety_radius}米"
            ).add_to(m)
        
        # 添加历史轨迹
        if flight_history and len(flight_history) > 1:
            trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
            if len(trail) > 1:
                folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, 
                              popup="历史轨迹").add_to(m)
        
        return m

# ==================== UI组件 ====================
class SidebarComponent:
    """侧边栏组件"""
    
    def __init__(self):
        self.auto_backup = True
        self.flight_altitude = DEFAULT_FLIGHT_ALTITUDE
        self.safety_radius = DEFAULT_SAFETY_RADIUS_METERS
        self.drone_speed = 50
        self.map_type = MapType.SATELLITE
    
    def render(self) -> Tuple[MapType, int, int, int]:
        """渲染侧边栏"""
        st.sidebar.title("🎛️ 导航菜单")
        page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
        
        map_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
        self.map_type = MapType.SATELLITE if map_choice == "卫星影像" else MapType.VECTOR
        
        st.sidebar.markdown("---")
        st.sidebar.subheader("⚡ 无人机速度设置")
        self.drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, 
                                             value=50, step=5)
        
        st.sidebar.markdown("---")
        st.sidebar.subheader("✈️ 无人机飞行高度")
        self.flight_altitude = st.sidebar.slider("飞行高度 (m)", min_value=MIN_FLIGHT_ALTITUDE, 
                                                max_value=MAX_FLIGHT_ALTITUDE, 
                                                value=DEFAULT_FLIGHT_ALTITUDE, step=5)
        
        st.sidebar.markdown("---")
        st.sidebar.subheader("🛡️ 安全半径设置")
        self.safety_radius = st.sidebar.slider("安全半径 (米)", min_value=1, max_value=20, 
                                              value=DEFAULT_SAFETY_RADIUS_METERS, step=1)
        
        st.sidebar.markdown("---")
        st.sidebar.subheader("💾 自动保存")
        self.auto_backup = st.sidebar.checkbox("自动保存障碍物", value=True)
        
        return self.map_type, page, self.drone_speed, self.flight_altitude, self.safety_radius

# ==================== 主程序 ====================
class DroneGroundStation:
    """无人机地面站主程序"""
    
    def __init__(self):
        self.obstacle_manager = ObstacleManager()
        self.geometry = GeometryCalculator()
        self.map_manager = MapManager(self.geometry)
        self.sidebar = SidebarComponent()
        self.avoider = ObstacleAvoider()
        
        self._init_session_state()
    
    def _init_session_state(self):
        """初始化session state"""
        if "points_gcj" not in st.session_state:
            st.session_state.points_gcj = {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()}
        if "heartbeat_sim" not in st.session_state:
            st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
        if "simulation_running" not in st.session_state:
            st.session_state.simulation_running = False
        if "flight_history" not in st.session_state:
            st.session_state.flight_history = []
        if "planned_path" not in st.session_state:
            st.session_state.planned_path = None
        if "current_direction" not in st.session_state:
            st.session_state.current_direction = FlightDirection.BEST
        if "pending_obstacle" not in st.session_state:
            st.session_state.pending_obstacle = None
        
    def update_planned_path(self, flight_altitude: float, safety_radius: float):
        """更新规划路径"""
        st.session_state.planned_path = self.avoider.create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            self.obstacle_manager.obstacles,
            flight_altitude,
            st.session_state.current_direction
        )
        self.avoider.safety_radius = safety_radius
    
    def render_planning_page(self, flight_altitude: int, safety_radius: int, drone_speed: int):
        """渲染航线规划页面"""
        st.header("🗺️ 航线规划 - 智能避障")
        
        # 检查路径是否被阻挡
        high_obstacles = self.obstacle_manager.get_high_obstacles(flight_altitude)
        straight_blocked = self.obstacle_manager.is_straight_path_blocked(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            flight_altitude
        )
        
        if straight_blocked:
            st.warning(f"⚠️ 有 {len(high_obstacles)} 个障碍物高于飞行高度({flight_altitude}m)，已自动规划避障路径")
        else:
            st.success("✅ 直线航线畅通无阻（所有障碍物高度 ≤ 飞行高度）")
        
        col1, col2 = st.columns([1, 1.5])
        
        with col1:
            self._render_control_panel(flight_altitude, safety_radius, drone_speed)
        
        with col2:
            self._render_planning_map(flight_altitude, safety_radius, straight_blocked)
    
    def _render_control_panel(self, flight_altitude: int, safety_radius: int, drone_speed: int):
        """渲染控制面板"""
        st.subheader("🎮 控制面板")
        
        with st.expander("📍 起点/终点设置", expanded=True):
            self._render_point_settings()
        
        with st.expander("🤖 路径规划策略", expanded=True):
            self._render_path_strategy(flight_altitude, safety_radius)
        
        with st.expander("✈️ 飞行控制", expanded=True):
            self._render_flight_control(flight_altitude, safety_radius, drone_speed)
        
        st.markdown("### 📍 当前坐标")
        st.write(f"🟢 A点: ({st.session_state.points_gcj['A'][0]:.6f}, {st.session_state.points_gcj['A'][1]:.6f})")
        st.write(f"🔴 B点: ({st.session_state.points_gcj['B'][0]:.6f}, {st.session_state.points_gcj['B'][1]:.6f})")
        
        dist = self.geometry.distance_meters(st.session_state.points_gcj['A'], 
                                            st.session_state.points_gcj['B'])
        st.caption(f"📏 直线距离: {dist:.0f} 米")
    
    def _render_point_settings(self):
        """渲染点设置"""
        st.markdown("#### 🟢 起点 A")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], 
                                   format="%.6f", key="a_lat", step=0.000001)
        with col_a2:
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], 
                                   format="%.6f", key="a_lng", step=0.000001)
        
        if st.button("📍 设置 A 点", use_container_width=True, type="primary"):
            st.session_state.points_gcj['A'] = [a_lng, a_lat]
            st.rerun()
        
        st.markdown("#### 🔴 终点 B")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], 
                                   format="%.6f", key="b_lat", step=0.000001)
        with col_b2:
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], 
                                   format="%.6f", key="b_lng", step=0.000001)
        
        if st.button("📍 设置 B 点", use_container_width=True, type="primary"):
            st.session_state.points_gcj['B'] = [b_lng, b_lat]
            st.rerun()
    
    def _render_path_strategy(self, flight_altitude: int, safety_radius: int):
        """渲染路径策略"""
        col_dir1, col_dir2, col_dir3 = st.columns(3)
        
        with col_dir1:
            if st.button("🔄 最佳航线", use_container_width=True,
                        type="primary" if st.session_state.current_direction == FlightDirection.BEST else "secondary"):
                st.session_state.current_direction = FlightDirection.BEST
                self.update_planned_path(flight_altitude, safety_radius)
                st.rerun()
        
        with col_dir2:
            if st.button("⬅️ 向左绕行", use_container_width=True,
                        type="primary" if st.session_state.current_direction == FlightDirection.LEFT else "secondary"):
                st.session_state.current_direction = FlightDirection.LEFT
                self.update_planned_path(flight_altitude, safety_radius)
                st.rerun()
        
        with col_dir3:
            if st.button("➡️ 向右绕行", use_container_width=True,
                        type="primary" if st.session_state.current_direction == FlightDirection.RIGHT else "secondary"):
                st.session_state.current_direction = FlightDirection.RIGHT
                self.update_planned_path(flight_altitude, safety_radius)
                st.rerun()
        
        st.info(f"📌 当前绕行策略: **{st.session_state.current_direction.value}**")
        
        if st.button("🔄 重新规划路径", use_container_width=True):
            self.update_planned_path(flight_altitude, safety_radius)
            if st.session_state.planned_path:
                waypoint_count = len(st.session_state.planned_path) - 2
                st.success(f"✅ 已规划避障路径，{waypoint_count}个绕行点")
            st.rerun()
    
    def _render_flight_control(self, flight_altitude: int, safety_radius: int, drone_speed: int):
        """渲染飞行控制"""
        col_met1, col_met2, col_met3 = st.columns(3)
        with col_met1:
            st.metric("当前飞行高度", f"{flight_altitude} m")
        with col_met2:
            st.metric("速度系数", f"{drone_speed}%")
        with col_met3:
            st.metric("🛡️ 安全半径", f"{safety_radius} 米")
        
        if st.session_state.planned_path:
            waypoint_count = len(st.session_state.planned_path) - 2
            st.metric("🎯 绕行点数量", waypoint_count)
            
            total_dist = 0
            for i in range(len(st.session_state.planned_path)-1):
                total_dist += self.geometry.distance_meters(
                    st.session_state.planned_path[i], 
                    st.session_state.planned_path[i+1]
                )
            st.caption(f"📏 规划路径总长: {total_dist:.0f} 米")
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                if st.session_state.points_gcj['A'] and st.session_state.points_gcj['B']:
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], 
                                                            st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_altitude, drone_speed, safety_radius)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    waypoint_count = len(path) - 2
                    st.success(f"🚁 飞行已开始！{'路径中有' + str(waypoint_count) + '个绕行点' if waypoint_count > 0 else '直线飞行'}")
                    st.rerun()
                else:
                    st.error("请先设置起点和终点")
        
        with col_btn2:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                st.session_state.simulation_running = False
                st.session_state.heartbeat_sim.simulating = False
                st.info("飞行已停止")
    
    def _render_planning_map(self, flight_altitude: int, safety_radius: int, straight_blocked: bool):
        """渲染规划地图"""
        st.subheader("🗺️ 规划地图")
        st.caption("🟣 向左绕行（沿边界绕行）| 🟠 向右绕行（1个绕行点）| 🟢 最佳航线")
        
        flight_trail = [[hb.lng, hb.lat] for hb in st.session_state.heartbeat_sim.history if hasattr(hb, 'lng')]
        center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
        
        if st.session_state.planned_path is None:
            self.update_planned_path(flight_altitude, safety_radius)
        
        drone_pos = st.session_state.heartbeat_sim.current_pos if st.session_state.heartbeat_sim.simulating else None
        
        m = self.map_manager.create_planning_map(
            center, st.session_state.points_gcj, 
            self.obstacle_manager.obstacles, flight_trail, 
            st.session_state.planned_path, "satellite", 
            flight_altitude, drone_pos, 
            st.session_state.current_direction, safety_radius
        )
        
        output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
        
        # 处理新绘制的障碍物
        if output and output.get("last_active_drawing"):
            last = output["last_active_drawing"]
            if last and last.get("geometry") and last["geometry"].get("type") == "Polygon":
                coords = last["geometry"].get("coordinates", [])
                if coords and len(coords) > 0:
                    poly = [[p[0], p[1]] for p in coords[0]]
                    if len(poly) >= 3 and st.session_state.pending_obstacle is None:
                        st.session_state.pending_obstacle = poly
                        st.rerun()
        
        # 处理待添加的障碍物
        if st.session_state.pending_obstacle is not None:
            self._render_pending_obstacle(flight_altitude, safety_radius)
    
    def _render_pending_obstacle(self, flight_altitude: int, safety_radius: int):
        """渲染待添加的障碍物"""
        st.markdown("---")
        st.subheader("📝 添加新障碍物")
        st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle)} 个顶点")
        
        col_name1, col_name2 = st.columns(2)
        with col_name1:
            new_name = st.text_input("障碍物名称", f"建筑物{len(self.obstacle_manager.obstacles) + 1}")
        with col_name2:
            new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, 
                                        value=30, step=5, key="height_input")
        
        col_ok, col_cancel = st.columns(2)
        with col_ok:
            if st.button("✅ 确认添加", use_container_width=True, type="primary"):
                new_obstacle = Obstacle(
                    name=new_name,
                    polygon=st.session_state.pending_obstacle,
                    height=new_height
                )
                if self.obstacle_manager.add(new_obstacle):
                    if self.sidebar.auto_backup:
                        self.obstacle_manager.save()
                    self.update_planned_path(flight_altitude, safety_radius)
                    st.session_state.pending_obstacle = None
                    st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
                    st.rerun()
                else:
                    st.error("添加失败，请检查多边形是否有效")
        
        with col_cancel:
            if st.button("❌ 取消", use_container_width=True):
                st.session_state.pending_obstacle = None
                st.rerun()
    
    def render_monitoring_page(self, flight_altitude: int, safety_radius: int, drone_speed: int):
        """渲染飞行监控页面"""
        st.header("📡 飞行监控 - 实时心跳包")
        st.caption(f"✈️ 当前飞行高度: {flight_altitude} 米 | 🧭 避障策略: {st.session_state.current_direction.value} | 🛡️ 安全半径: {safety_radius} 米")
        
        # 更新心跳
        current_time = time.time()
        if st.session_state.simulation_running:
            if current_time - st.session_state.last_hb_time >= 0.2:
                try:
                    new_hb = st.session_state.heartbeat_sim.update_and_generate(self.obstacle_manager.obstacles)
                    if new_hb:
                        st.session_state.last_hb_time = current_time
                        st.session_state.flight_history.append([new_hb.lng, new_hb.lat])
                        if len(st.session_state.flight_history) > 200:
                            st.session_state.flight_history.pop(0)
                        if not st.session_state.heartbeat_sim.simulating:
                            st.session_state.simulation_running = False
                            st.success("🏁 无人机已安全到达目的地！")
                        st.rerun()
                except Exception as e:
                    st.error(f"更新心跳时出错: {e}")
                    logger.error(f"心跳更新错误: {e}")
        else:
            st.session_state.last_hb_time = current_time
        
        # 显示最新数据
        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            
            col1, col2, col3, col4, col5, col6 = st.columns(6)
            col1.metric("⏰ 时间", latest.timestamp)
            col2.metric("📍 纬度", f"{latest.lat:.6f}")
            col3.metric("📍 经度", f"{latest.lng:.6f}")
            col4.metric("📊 高度", f"{latest.altitude} m")
            col5.metric("🔋 电压", f"{latest.voltage} V")
            col6.metric("🛰️ 卫星", latest.satellites)
            
            col7, col8, col9 = st.columns(3)
            col7.metric("💨 速度", f"{latest.speed} m/s")
            col8.metric("⚡ 速度系数", f"{drone_speed}%")
            col9.metric("📏 剩余距离", f"{latest.remaining_distance:.0f} m")
            
            if latest.safety_violation:
                st.error("⚠️ 警告：无人机进入安全半径危险区域！请立即检查！")
            
            progress = latest.progress
            st.progress(progress, text=f"✈️ 飞行进度: {progress*100:.1f}%")
            
            if latest.arrived:
                st.success("🎉 无人机已到达目的地！飞行任务完成！")
                if self.sidebar.auto_backup and st.session_state.heartbeat_sim.flight_log:
                    df = st.session_state.heartbeat_sim.export_flight_data()
                    csv = df.to_csv(index=False)
                    st.download_button("📥 下载本次飞行日志", csv, 
                                     f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            
            # 显示地图
            self._render_monitoring_map(latest, flight_altitude, safety_radius)
            
            # 显示图表和历史数据
            self._render_monitoring_charts()
            self._render_flight_history()
    
    def _render_monitoring_map(self, latest, flight_altitude: int, safety_radius: int):
        """渲染监控地图"""
        st.subheader("📍 实时位置")
        
        tiles = GAODE_SATELLITE_URL
        monitor_map = folium.Map(location=[latest.lat, latest.lng], zoom_start=17, 
                                tiles=tiles, attr="高德地图")
        
        # 添加障碍物
        for obs in self.obstacle_manager.obstacles:
            coords = obs.polygon
            if coords and len(coords) >= 3:
                color = "red" if obs.height > flight_altitude else "orange"
                folium.Polygon([[c[1], c[0]] for c in coords], 
                              color=color, weight=2, fill=True, fill_opacity=0.3, 
                              popup=f"🚧 {obs.name}").add_to(monitor_map)
        
        # 添加规划路径
        if st.session_state.planned_path and len(st.session_state.planned_path) > 1:
            if st.session_state.current_direction == FlightDirection.LEFT:
                line_color = "purple"
            elif st.session_state.current_direction == FlightDirection.RIGHT:
                line_color = "orange"
            else:
                line_color = "green"
            folium.PolyLine([[p[1], p[0]] for p in st.session_state.planned_path], 
                          color=line_color, weight=3, opacity=0.7).add_to(monitor_map)
        
        # 添加安全半径
        folium.Circle(
            radius=safety_radius,
            location=[latest.lat, latest.lng],
            color="blue",
            weight=2,
            fill=True,
            fill_color="blue",
            fill_opacity=0.2,
            popup=f"🛡️ 安全半径: {safety_radius}米"
        ).add_to(monitor_map)
        
        # 添加历史轨迹
        trail = [[hb.lat, hb.lng] for hb in st.session_state.heartbeat_sim.history[:30] 
                if hasattr(hb, 'lat')]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.7).add_to(monitor_map)
        
        # 添加当前位置标记
        folium.Marker([latest.lat, latest.lng], 
                     popup=f"当前位置\n高度: {latest.altitude}m\n速度: {latest.speed}m/s", 
                     icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(monitor_map)
        
        # 添加起点终点
        if st.session_state.points_gcj['A']:
            folium.Marker([st.session_state.points_gcj['A'][1], st.session_state.points_gcj['A'][0]], 
                        popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(monitor_map)
        if st.session_state.points_gcj['B']:
            folium.Marker([st.session_state.points_gcj['B'][1], st.session_state.points_gcj['B'][0]], 
                        popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(monitor_map)
        
        folium_static(monitor_map, width=800, height=400)
    
    def _render_monitoring_charts(self):
        """渲染监控图表"""
        st.subheader("📈 实时数据图表")
        col_ch1, col_ch2 = st.columns(2)
        
        with col_ch1:
            if len(st.session_state.heartbeat_sim.history) > 1:
                alt_df = pd.DataFrame({
                    "序号": list(range(min(30, len(st.session_state.heartbeat_sim.history)))),
                    "高度(m)": [h.altitude for h in st.session_state.heartbeat_sim.history[:30]]
                })
                st.line_chart(alt_df, x="序号", y="高度(m)")
        
        with col_ch2:
            if len(st.session_state.heartbeat_sim.history) > 1:
                speed_df = pd.DataFrame({
                    "序号": list(range(min(30, len(st.session_state.heartbeat_sim.history)))),
                    "速度(m/s)": [h.speed for h in st.session_state.heartbeat_sim.history[:30]]
                })
                st.line_chart(speed_df, x="序号", y="速度(m/s)")
    
    def _render_flight_history(self):
        """渲染飞行历史记录"""
        st.subheader("📋 历史心跳记录")
        
        if st.session_state.heartbeat_sim.history:
            history_data = []
            for h in list(st.session_state.heartbeat_sim.history)[:10]:
                history_data.append({
                    'timestamp': h.timestamp,
                    'flight_time': f"{h.flight_time:.1f}s",
                    'lat': f"{h.lat:.6f}",
                    'lng': f"{h.lng:.6f}",
                    'altitude': h.altitude,
                    'speed': h.speed,
                    'voltage': h.voltage,
                    'satellites': h.satellites
                })
            
            history_df = pd.DataFrame(history_data)
            st.dataframe(history_df, use_container_width=True)
            
            if st.button("📊 导出完整飞行数据", use_container_width=True):
                df = st.session_state.heartbeat_sim.export_flight_data()
                csv = df.to_csv(index=False)
                st.download_button("下载CSV文件", csv, 
                                 f"flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    
    def render_obstacle_page(self, flight_altitude: int, safety_radius: int):
        """渲染障碍物管理页面"""
        st.header("🚧 障碍物管理")
        
        col_info1, col_info2 = st.columns(2)
        with col_info1:
            if os.path.exists(CONFIG_FILE):
                file_size = os.path.getsize(CONFIG_FILE)
                mod_time = datetime.fromtimestamp(os.path.getmtime(CONFIG_FILE)).strftime("%Y-%m-%d %H:%M:%S")
                st.info(f"📁 配置文件: {CONFIG_FILE}\n📏 大小: {file_size} bytes\n🕐 修改时间: {mod_time}")
            else:
                st.warning(f"⚠️ 配置文件 {CONFIG_FILE} 不存在，请先保存数据")
        
        with col_info2:
            backup_count = len([f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)])
            st.info(f"💾 备份文件数量: {backup_count} 个\n📂 备份目录: {BACKUP_DIR}")
        
        st.subheader("💾 数据管理")
        st.info(f"当前共 **{len(self.obstacle_manager.obstacles)}** 个障碍物 | 🛡️ 安全半径: {safety_radius} 米")
        
        # 数据操作按钮
        col_save1, col_save2, col_save3, col_save4 = st.columns(4)
        
        with col_save1:
            if st.button("💾 保存到JSON", use_container_width=True, type="primary"):
                if self.obstacle_manager.save():
                    st.success(f"✅ 已保存到 {CONFIG_FILE}")
                    st.balloons()
                else:
                    st.error("保存失败")
        
        with col_save2:
            if st.button("📂 加载JSON", use_container_width=True):
                if self.obstacle_manager.load():
                    self.update_planned_path(flight_altitude, safety_radius)
                    st.success(f"✅ 成功加载 {len(self.obstacle_manager.obstacles)} 个障碍物")
                    st.rerun()
                else:
                    st.warning("⚠️ 加载失败或文件为空")
        
        with col_save3:
            if self.obstacle_manager.obstacles:
                config_data = {
                    'obstacles': [obs.to_dict() for obs in self.obstacle_manager.obstacles],
                    'count': len(self.obstacle_manager.obstacles),
                    'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    'version': 'v27.6'
                }
                st.download_button(
                    label="📥 下载配置",
                    data=json.dumps(config_data, ensure_ascii=False, indent=2),
                    file_name=CONFIG_FILE,
                    mime="application/json",
                    use_container_width=True
                )
        
        with col_save4:
            if st.button("🔄 恢复上次备份", use_container_width=True):
                self._restore_backup(flight_altitude, safety_radius)
        
        st.markdown("---")
        st.subheader("📝 障碍物列表")
        
        col_list, col_map = st.columns([1, 1.5])
        
        with col_list:
            self._render_obstacle_list(flight_altitude, safety_radius)
        
        with col_map:
            self._render_obstacle_map(flight_altitude)
    
    def _restore_backup(self, flight_altitude: int, safety_radius: int):
        """恢复备份"""
        try:
            backup_files = [f for f in os.listdir(BACKUP_DIR) if f.startswith(CONFIG_FILE)]
            if backup_files:
                backup_files.sort(key=lambda x: os.path.getctime(os.path.join(BACKUP_DIR, x)), reverse=True)
                latest_backup = backup_files[0]
                
                with open(f"{BACKUP_DIR}/{latest_backup}", 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    obstacles_data = data.get('obstacles', [])
                    
                    if obstacles_data:
                        self.obstacle_manager.obstacles = [Obstacle.from_dict(obs) for obs in obstacles_data]
                        self.update_planned_path(flight_altitude, safety_radius)
                        st.success(f"✅ 已从备份恢复: {latest_backup} (共 {len(obstacles_data)} 个障碍物)")
                        st.rerun()
                    else:
                        st.warning("备份文件中没有障碍物数据")
            else:
                st.info("没有找到备份文件")
        except Exception as e:
            st.error(f"恢复备份失败: {e}")
    
    def _render_obstacle_list(self, flight_altitude: int, safety_radius: int):
        """渲染障碍物列表"""
        if self.obstacle_manager.obstacles:
            for i, obs in enumerate(self.obstacle_manager.obstacles):
                color = "🔴" if obs.height > flight_altitude else "🟠"
                with st.container():
                    st.markdown(f"**{color} {obs.name}**")
                    col_h, col_edit, col_del = st.columns([1, 1.5, 0.8])
                    with col_h:
                        st.write(f"高度: {obs.height}m")
                    with col_edit:
                        new_h = st.number_input("", value=int(obs.height), min_value=1, max_value=200,
                                               step=5, key=f"edit_{i}", label_visibility="collapsed")
                        if new_h != obs.height:
                            obs.height = new_h
                            if self.sidebar.auto_backup:
                                self.obstacle_manager.save()
                            self.update_planned_path(flight_altitude, safety_radius)
                            st.rerun()
                    with col_del:
                        if st.button("🗑️", key=f"del_{i}"):
                            if self.obstacle_manager.remove(i):
                                if self.sidebar.auto_backup:
                                    self.obstacle_manager.save()
                                self.update_planned_path(flight_altitude, safety_radius)
                                st.rerun()
                    st.markdown("---")
        else:
            st.info("暂无任何障碍物，可以在地图上绘制添加")
        
        if st.button("🗑️ 清除所有障碍物", use_container_width=True):
            if self.obstacle_manager.clear():
                if self.sidebar.auto_backup:
                    self.obstacle_manager.save()
                self.update_planned_path(flight_altitude, safety_radius)
                st.success("已清除所有障碍物")
                st.rerun()
    
    def _render_obstacle_map(self, flight_altitude: int):
        """渲染障碍物地图"""
        st.subheader("🗺️ 障碍物分布图")
        
        tiles = GAODE_SATELLITE_URL
        obs_map = folium.Map(location=[SCHOOL_CENTER_GCJ[1], SCHOOL_CENTER_GCJ[0]], 
                            zoom_start=16, tiles=tiles, attr="高德地图")
        
        for obs in self.obstacle_manager.obstacles:
            coords = obs.polygon
            if coords and len(coords) >= 3:
                color = "red" if obs.height > flight_altitude else "orange"
                folium.Polygon([[c[1], c[0]] for c in coords], 
                              color=color, weight=3, fill=True, fill_opacity=0.5, 
                              popup=f"{obs.name}\n高度: {obs.height}m").add_to(obs_map)
        
        folium.Marker([DEFAULT_A_GCJ[1], DEFAULT_A_GCJ[0]], 
                     popup="起点", icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(obs_map)
        folium.Marker([DEFAULT_B_GCJ[1], DEFAULT_B_GCJ[0]], 
                     popup="终点", icon=folium.Icon(color='red', icon='stop', prefix='fa')).add_to(obs_map)
        
        folium_static(obs_map, width=700, height=500)
        
        st.caption("🔴 红色: 需要避让（高于飞行高度）| 🟠 橙色: 安全（低于飞行高度）")
    
    def run(self):
        """运行主程序"""
        st.markdown("""
        <style>
        .stButton button { width: 100%; transition: all 0.3s ease; }
        .stButton button:hover { transform: translateY(-2px); box-shadow: 0 2px 5px rgba(0,0,0,0.2); }
        </style>
        """, unsafe_allow_html=True)
        
        st.title("🏫 南京科技职业学院 - 无人机地面站系统")
        st.markdown("---")
        
        # 渲染侧边栏
        map_type, page, drone_speed, flight_altitude, safety_radius = self.sidebar.render()
        
        # 显示侧边栏信息
        self._render_sidebar_info(flight_altitude, safety_radius)
        
        # 初始化时间戳
        if "last_hb_time" not in st.session_state:
            st.session_state.last_hb_time = time.time()
        
        # 更新避障器的安全半径
        self.avoider.safety_radius = safety_radius
        
        # 根据页面渲染不同内容
        if page == "🗺️ 航线规划":
            self.render_planning_page(flight_altitude, safety_radius, drone_speed)
        elif page == "📡 飞行监控":
            self.render_monitoring_page(flight_altitude, safety_radius, drone_speed)
        elif page == "🚧 障碍物管理":
            self.render_obstacle_page(flight_altitude, safety_radius)
    
    def _render_sidebar_info(self, flight_altitude: int, safety_radius: int):
        """渲染侧边栏信息"""
        st.sidebar.markdown("---")
        
        straight_blocked = self.obstacle_manager.is_straight_path_blocked(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            flight_altitude
        )
        high_obstacles = len(self.obstacle_manager.get_high_obstacles(flight_altitude))
        
        st.sidebar.info(
            f"🏫 南京科技职业学院\n"
            f"🚧 障碍物: {len(self.obstacle_manager.obstacles)} 个\n"
            f"📈 高于飞行高度: {high_obstacles} 个\n"
            f"📌 直线路径: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}\n"
            f"✈️ 飞行高度: {flight_altitude} m\n"
            f"🛡️ 安全半径: {safety_radius} 米\n"
            f"✨ 绕行策略: 向右=1个绕行点 | 向左=沿边界绕行"
        )
        
        col_refresh1, col_refresh2 = st.sidebar.columns(2)
        with col_refresh1:
            if st.button("🔄 刷新数据", use_container_width=True):
                self.obstacle_manager.load()
                self.update_planned_path(flight_altitude, safety_radius)
                st.success("数据已刷新")
                st.rerun()
        
        with col_refresh2:
            if st.button("📊 导出日志", use_container_width=True):
                if st.session_state.heartbeat_sim.flight_log:
                    df = st.session_state.heartbeat_sim.export_flight_data()
                    csv = df.to_csv(index=False)
                    st.download_button("下载飞行日志", csv, 
                                     f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
                else:
                    st.warning("暂无飞行数据")

# ==================== 主程序入口 ====================
def main():
    """主程序入口"""
    app = DroneGroundStation()
    app.run()

if __name__ == "__main__":
    main()
