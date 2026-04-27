def find_left_path(start: List[float], end: List[float], obstacles_gcj: List[Dict], flight_altitude: float, safety_radius: float = 5) -> List[List[float]]:
    """
    向左绕行：从顶部绕过障碍物
    路径：起点 → 向上很远 → 向右很远 → 终点
    """
    blocking_obs = get_blocking_obstacles(start, end, obstacles_gcj, flight_altitude)
    
    if not blocking_obs:
        return [start, end]
    
    # 计算障碍物最右侧和最上侧
    max_lng = -float('inf')
    max_lat = -float('inf')
    min_lat = float('inf')
    
    for obs in blocking_obs:
        for point in obs.get('polygon', []):
            max_lng = max(max_lng, point[0])
            max_lat = max(max_lat, point[1])
            min_lat = min(min_lat, point[1])
    
    if max_lng == -float('inf'):
        return [start, end]
    
    # 安全偏移（50米转度）
    safe_lng, safe_lat = meters_to_deg(50)
    
    # 计算障碍物高度
    obstacle_height = max_lat - min_lat
    
    # 点1：向上飞（距离＝障碍物高度×3 + 50米×3）
    waypoint_up = [start[0], max_lat + obstacle_height * 3 + safe_lat * 3]
    
    # 点2：向右飞（距离＝障碍物高度×2 + 50米×2）
    waypoint_right = [max_lng + obstacle_height * 2 + safe_lng * 2, waypoint_up[1]]
    
    # 终点
    waypoint_end = end
    
    return [start, waypoint_up, waypoint_right, waypoint_end]
