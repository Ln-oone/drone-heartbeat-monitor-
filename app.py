# ==================== 增强版障碍物管理 ====================

class ObstacleManager:
    """障碍物管理器 - 支持全选、批量操作等"""
    
    @staticmethod
    def create_obstacle_list_ui(obstacles: List[Dict], flight_altitude: int, 
                                  on_change_callback=None) -> Tuple[List[bool], List[Dict]]:
        """
        创建带复选框的障碍物列表UI
        
        Returns:
            (selected_states, modified_obstacles)
        """
        modified_obstacles = []
        selected_states = []
        
        st.markdown("### 📋 障碍物列表")
        
        # 全选控制栏
        col_select_all, col_batch_delete, col_batch_height, col_info = st.columns([1, 1, 2, 1])
        
        with col_select_all:
            select_all = st.checkbox("☑️ 全选", key="select_all_obstacles")
        
        with col_batch_delete:
            if st.button("🗑️ 批量删除", use_container_width=True, type="secondary"):
                return None, None  # 返回特殊信号表示批量删除
        
        with col_batch_height:
            col_h1, col_h2 = st.columns([1, 1])
            with col_h1:
                batch_height = st.number_input("批量高度", min_value=1, max_value=200, 
                                              value=30, step=5, key="batch_height")
            with col_h2:
                if st.button("📏 批量设置高度", use_container_width=True):
                    return batch_height, None  # 返回批量高度值
        
        with col_info:
            st.caption(f"共 {len(obstacles)} 个障碍物")
        
        st.markdown("---")
        
        # 创建可滚动的障碍物列表容器
        list_container = st.container()
        
        with list_container:
            for i, obs in enumerate(obstacles):
                height = obs.get('height', 30)
                color = "🔴" if height > flight_altitude else "🟠"
                name = obs.get('name', f'障碍物{i+1}')
                
                # 确定选中状态（全选影响）
                is_selected = select_all if select_all else False
                if 'selected' in obs and not select_all:
                    is_selected = obs['selected']
                
                selected_states.append(is_selected)
                
                # 创建带边框的障碍物卡片
                with st.container():
                    col_check, col_info, col_actions = st.columns([0.5, 2, 1.5])
                    
                    with col_check:
                        checked = st.checkbox("", value=is_selected, key=f"select_{i}")
                        selected_states[i] = checked
                        obs['selected'] = checked
                    
                    with col_info:
                        st.markdown(f"**{color} {name}**")
                        st.caption(f"📏 高度: {height}m | 📍 顶点: {len(obs.get('polygon', []))}")
                        
                        # 显示ID和创建时间
                        obs_id = obs.get('id', 'N/A')
                        created_time = obs.get('created_time', 'N/A')
                        if created_time != 'N/A' and isinstance(created_time, str):
                            try:
                                dt = datetime.fromisoformat(created_time)
                                created_time = dt.strftime("%Y-%m-%d %H:%M")
                            except:
                                pass
                        st.caption(f"🆔 {obs_id[:12]}... | 🕐 {created_time}")
                    
                    with col_actions:
                        # 单独编辑高度
                        new_height = st.number_input("高度", value=height, min_value=1, max_value=200,
                                                    step=5, key=f"height_{i}", label_visibility="collapsed")
                        if new_height != height:
                            obs['height'] = new_height
                            modified_obstacles.append(obs)
                        
                        # 单独删除按钮
                        if st.button("❌", key=f"delete_{i}", help="删除此障碍物"):
                            return [-1, i], None  # 返回删除索引信号
                    
                    st.markdown("---")
        
        return selected_states, modified_obstacles
    
    @staticmethod
    def create_batch_edit_dialog(obstacles: List[Dict]) -> Dict:
        """创建批量编辑对话框"""
        with st.expander("✏️ 批量编辑工具", expanded=False):
            st.markdown("#### 批量属性编辑")
            
            col_name, col_height, col_color = st.columns(3)
            
            with col_name:
                name_prefix = st.text_input("名称前缀", value="建筑物")
            
            with col_height:
                default_height = st.number_input("默认高度(m)", min_value=1, max_value=200, value=30, step=5)
            
            with col_color:
                color_choice = st.selectbox("显示颜色", ["自动", "红色(需避让)", "橙色(安全)"], index=0)
            
            col_apply, col_cancel = st.columns(2)
            with col_apply:
                if st.button("✅ 应用到选中项", use_container_width=True, type="primary"):
                    return {
                        'action': 'apply',
                        'name_prefix': name_prefix,
                        'default_height': default_height,
                        'color_choice': color_choice
                    }
            with col_cancel:
                if st.button("❌ 取消", use_container_width=True):
                    return {'action': 'cancel'}
        
        return None
    
    @staticmethod
    def create_import_export_ui(obstacles: List[Dict]) -> Dict:
        """创建导入导出界面"""
        with st.expander("💾 导入/导出功能", expanded=False):
            col_export, col_import, col_merge = st.columns(3)
            
            with col_export:
                if obstacles:
                    export_data = {
                        'obstacles': obstacles,
                        'count': len(obstacles),
                        'export_time': datetime.now().isoformat(),
                        'version': '2.0'
                    }
                    st.download_button(
                        label="📤 导出障碍物",
                        data=json.dumps(export_data, ensure_ascii=False, indent=2),
                        file_name=f"obstacles_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                        mime="application/json",
                        use_container_width=True
                    )
            
            with col_import:
                uploaded_file = st.file_uploader("📥 导入障碍物", type=['json'], key="obstacle_import")
                if uploaded_file is not None:
                    try:
                        import_data = json.load(uploaded_file)
                        return {'action': 'import', 'data': import_data}
                    except Exception as e:
                        st.error(f"导入失败: {e}")
            
            with col_merge:
                if st.button("🔄 合并配置", use_container_width=True):
                    return {'action': 'merge'}
        
        return None

# ==================== 在main函数中替换障碍物管理页面 ====================

def main():
    # ... 前面的代码保持不变 ...
    
    # ==================== 障碍物管理页面（增强版） ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理 - 高级功能")
        
        # 顶部状态栏
        col_stats1, col_stats2, col_stats3, col_stats4 = st.columns(4)
        with col_stats1:
            st.metric("📊 障碍物总数", len(st.session_state.obstacles_gcj))
        with col_stats2:
            high_obs = sum(1 for obs in st.session_state.obstacles_gcj 
                          if obs.get('height', 30) > flight_alt)
            st.metric("🔴 需避让障碍物", high_obs)
        with col_stats3:
            safe_obs = len(st.session_state.obstacles_gcj) - high_obs
            st.metric("🟠 安全障碍物", safe_obs)
        with col_stats4:
            total_vertices = sum(len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj)
            st.metric("📍 总顶点数", total_vertices)
        
        st.markdown("---")
        
        # 创建标签页
        tab_list, tab_map, tab_tools = st.tabs(["📋 列表视图", "🗺️ 地图视图", "🔧 高级工具"])
        
        with tab_list:
            # 使用增强版障碍物管理器
            manager = ObstacleManager()
            
            # 批量删除按钮
            col_batch1, col_batch2, col_batch3, col_batch4 = st.columns(4)
            with col_batch1:
                if st.button("🗑️ 删除选中项", use_container_width=True, type="primary"):
                    # 删除选中的障碍物
                    selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) 
                                       if obs.get('selected', False)]
                    if selected_indices:
                        # 从后往前删除，避免索引错误
                        for i in reversed(selected_indices):
                            st.session_state.obstacles_gcj.pop(i)
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.success(f"已删除 {len(selected_indices)} 个障碍物")
                        st.rerun()
            
            with col_batch2:
                if st.button("📏 批量设置高度", use_container_width=True):
                    selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) 
                                       if obs.get('selected', False)]
                    if selected_indices:
                        # 显示批量高度设置对话框
                        st.session_state.show_batch_height_dialog = True
            
            with col_batch3:
                if st.button("🏷️ 批量重命名", use_container_width=True):
                    selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) 
                                       if obs.get('selected', False)]
                    if selected_indices:
                        st.session_state.show_batch_rename_dialog = True
            
            with col_batch4:
                if st.button("✨ 批量操作", use_container_width=True):
                    st.session_state.show_batch_operations = True
            
            # 批量高度设置对话框
            if st.session_state.get('show_batch_height_dialog', False):
                with st.expander("📏 批量设置高度", expanded=True):
                    col_h1, col_h2 = st.columns(2)
                    with col_h1:
                        new_batch_height = st.number_input("统一高度(米)", min_value=1, max_value=200, 
                                                          value=30, step=5, key="batch_height_input")
                    with col_h2:
                        if st.button("确认设置", use_container_width=True, type="primary"):
                            selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) 
                                               if obs.get('selected', False)]
                            for i in selected_indices:
                                st.session_state.obstacles_gcj[i]['height'] = new_batch_height
                            if st.session_state.auto_backup:
                                save_obstacles(st.session_state.obstacles_gcj)
                            st.session_state.planned_path = create_avoidance_path(
                                st.session_state.points_gcj['A'],
                                st.session_state.points_gcj['B'],
                                st.session_state.obstacles_gcj,
                                flight_alt,
                                st.session_state.current_direction,
                                safety_radius
                            )
                            st.session_state.show_batch_height_dialog = False
                            st.success(f"已为 {len(selected_indices)} 个障碍物设置高度为 {new_batch_height}m")
                            st.rerun()
                    if st.button("取消"):
                        st.session_state.show_batch_height_dialog = False
                        st.rerun()
            
            # 批量重命名对话框
            if st.session_state.get('show_batch_rename_dialog', False):
                with st.expander("🏷️ 批量重命名", expanded=True):
                    col_n1, col_n2 = st.columns(2)
                    with col_n1:
                        name_prefix = st.text_input("名称前缀", value="建筑物")
                        start_number = st.number_input("起始编号", min_value=1, value=1, step=1)
                    with col_n2:
                        name_suffix = st.text_input("名称后缀", value="")
                    if st.button("确认重命名", use_container_width=True, type="primary"):
                        selected_indices = [i for i, obs in enumerate(st.session_state.obstacles_gcj) 
                                           if obs.get('selected', False)]
                        for idx, i in enumerate(selected_indices):
                            new_name = f"{name_prefix}{start_number + idx}{name_suffix}"
                            st.session_state.obstacles_gcj[i]['name'] = new_name
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.show_batch_rename_dialog = False
                        st.success(f"已重命名 {len(selected_indices)} 个障碍物")
                        st.rerun()
                    if st.button("取消"):
                        st.session_state.show_batch_rename_dialog = False
                        st.rerun()
            
            # 显示障碍物列表（带复选框）
            if st.session_state.obstacles_gcj:
                # 创建带复选框的列表
                st.markdown("#### 📋 障碍物列表（可多选）")
                st.caption("💡 提示：勾选复选框后可使用批量操作功能")
                
                # 使用 columns 布局显示障碍物
                items_per_row = 2
                rows = (len(st.session_state.obstacles_gcj) + items_per_row - 1) // items_per_row
                
                for row in range(rows):
                    cols = st.columns(items_per_row)
                    for col_idx in range(items_per_row):
                        idx = row * items_per_row + col_idx
                        if idx < len(st.session_state.obstacles_gcj):
                            obs = st.session_state.obstacles_gcj[idx]
                            with cols[col_idx]:
                                with st.container(border=True):
                                    height = obs.get('height', 30)
                                    color = "🔴" if height > flight_alt else "🟠"
                                    name = obs.get('name', f'障碍物{idx+1}')
                                    
                                    # 复选框
                                    checked = st.checkbox("", key=f"select_card_{idx}", 
                                                         value=obs.get('selected', False))
                                    st.session_state.obstacles_gcj[idx]['selected'] = checked
                                    
                                    st.markdown(f"**{color} {name}**")
                                    st.caption(f"📏 高度: {height}m")
                                    st.caption(f"📍 顶点: {len(obs.get('polygon', []))}个")
                                    
                                    # 快速编辑
                                    new_h = st.number_input("高度", value=height, min_value=1, max_value=200,
                                                           step=5, key=f"quick_edit_{idx}", label_visibility="collapsed")
                                    if new_h != height:
                                        obs['height'] = new_h
                                        if st.session_state.auto_backup:
                                            save_obstacles(st.session_state.obstacles_gcj)
                                        st.session_state.planned_path = create_avoidance_path(
                                            st.session_state.points_gcj['A'],
                                            st.session_state.points_gcj['B'],
                                            st.session_state.obstacles_gcj,
                                            flight_alt,
                                            st.session_state.current_direction,
                                            safety_radius
                                        )
                                        st.rerun()
                                    
                                    if st.button("🗑️ 删除", key=f"delete_card_{idx}", use_container_width=True):
                                        st.session_state.obstacles_gcj.pop(idx)
                                        if st.session_state.auto_backup:
                                            save_obstacles(st.session_state.obstacles_gcj)
                                        st.session_state.planned_path = create_avoidance_path(
                                            st.session_state.points_gcj['A'],
                                            st.session_state.points_gcj['B'],
                                            st.session_state.obstacles_gcj,
                                            flight_alt,
                                            st.session_state.current_direction,
                                            safety_radius
                                        )
                                        st.rerun()
            else:
                st.info("暂无任何障碍物，可以在「地图视图」中绘制添加")
        
        with tab_map:
            st.subheader("🗺️ 障碍物地图视图")
            st.caption("✏️ 使用左上角绘制工具绘制新障碍物 | 🖱️ 点击障碍物查看详细信息 | 🎨 红色=需避让，橙色=安全")
            
            # 地图控制选项
            col_map_controls1, col_map_controls2, col_map_controls3 = st.columns(3)
            with col_map_controls1:
                map_view_type = st.radio("地图类型", ["卫星影像", "矢量街道"], index=0, horizontal=True)
                map_type_view = "satellite" if map_view_type == "卫星影像" else "vector"
            
            with col_map_controls2:
                show_labels = st.checkbox("显示障碍物标签", value=True)
            
            with col_map_controls3:
                auto_refresh = st.checkbox("自动刷新地图", value=False)
            
            # 创建带绘制工具的地图
            drawing_tool = ObstacleDrawingTool()
            obs_map = drawing_tool.create_drawing_map(
                SCHOOL_CENTER_GCJ,
                st.session_state.obstacles_gcj,
                flight_alt,
                map_type_view
            )
            
            # 添加绘制完成回调
            map_output = st_folium(obs_map, width=800, height=600, 
                                   key="obstacle_map_view",
                                   returned_objects=["last_active_drawing"])
            
            # 处理新绘制的障碍物
            if map_output and map_output.get("last_active_drawing"):
                polygon = ObstacleDrawingTool.process_drawing(map_output["last_active_drawing"])
                if polygon and not st.session_state.get('pending_obstacle_from_map'):
                    if validate_polygon(polygon):
                        st.session_state.pending_obstacle_from_map = polygon
                        st.rerun()
            
            # 添加新障碍物对话框
            if st.session_state.get('pending_obstacle_from_map'):
                st.markdown("---")
                st.subheader("📝 添加新障碍物")
                st.info(f"已检测到新绘制的多边形，共 {len(st.session_state.pending_obstacle_from_map)} 个顶点")
                
                col_name1, col_name2 = st.columns(2)
                with col_name1:
                    new_name = st.text_input("障碍物名称", f"建筑物{len(st.session_state.obstacles_gcj) + 1}")
                with col_name2:
                    new_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=200, 
                                                value=flight_alt + 10 if flight_alt < 100 else flight_alt,
                                                step=5, key="map_height_input")
                
                col_ok, col_cancel = st.columns(2)
                with col_ok:
                    if st.button("✅ 确认添加", use_container_width=True, type="primary"):
                        new_obstacle = {
                            "name": new_name,
                            "polygon": st.session_state.pending_obstacle_from_map,
                            "height": new_height,
                            "id": f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(st.session_state.obstacles_gcj)}",
                            "created_time": datetime.now().isoformat(),
                            "selected": False
                        }
                        st.session_state.obstacles_gcj.append(new_obstacle)
                        if st.session_state.auto_backup:
                            save_obstacles(st.session_state.obstacles_gcj)
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            st.session_state.obstacles_gcj,
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.session_state.pending_obstacle_from_map = None
                        st.success(f"✅ 已添加 {new_name}，高度 {new_height} 米")
                        st.rerun()
                with col_cancel:
                    if st.button("❌ 取消", use_container_width=True):
                        st.session_state.pending_obstacle_from_map = None
                        st.rerun()
        
        with tab_tools:
            st.subheader("🔧 高级工具")
            
            # 统计信息
            st.markdown("### 📊 障碍物统计分析")
            col_stat1, col_stat2, col_stat3 = st.columns(3)
            with col_stat1:
                # 高度分布
                heights = [obs.get('height', 30) for obs in st.session_state.obstacles_gcj]
                if heights:
                    st.write("**高度分布**")
                    height_df = pd.DataFrame(heights, columns=["高度(m)"])
                    st.bar_chart(height_df)
                else:
                    st.info("暂无数据")
            
            with col_stat2:
                # 障碍物类型统计
                high_obs_list = [obs for obs in st.session_state.obstacles_gcj 
                                if obs.get('height', 30) > flight_alt]
                safe_obs_list = [obs for obs in st.session_state.obstacles_gcj 
                                if obs.get('height', 30) <= flight_alt]
                st.write("**障碍物分类**")
                st.metric("需避让", len(high_obs_list))
                st.metric("安全通过", len(safe_obs_list))
            
            with col_stat3:
                # 顶点统计
                vertices_counts = [len(obs.get('polygon', [])) for obs in st.session_state.obstacles_gcj]
                if vertices_counts:
                    st.write("**顶点统计**")
                    st.metric("平均顶点数", f"{sum(vertices_counts)/len(vertices_counts):.1f}")
                    st.metric("最大顶点数", max(vertices_counts))
                    st.metric("最小顶点数", min(vertices_counts))
            
            st.markdown("---")
            
            # 数据验证工具
            st.markdown("### 🔍 数据验证")
            col_validate1, col_validate2 = st.columns(2)
            with col_validate1:
                if st.button("验证所有障碍物", use_container_width=True):
                    invalid_obstacles = []
                    for i, obs in enumerate(st.session_state.obstacles_gcj):
                        polygon = obs.get('polygon', [])
                        if len(polygon) < 3:
                            invalid_obstacles.append(f"{obs.get('name')}: 顶点数不足")
                        elif not validate_polygon(polygon):
                            invalid_obstacles.append(f"{obs.get('name')}: 多边形自相交")
                    
                    if invalid_obstacles:
                        st.error(f"发现 {len(invalid_obstacles)} 个无效障碍物:")
                        for err in invalid_obstacles:
                            st.write(f"- {err}")
                    else:
                        st.success("✅ 所有障碍物数据有效！")
            
            with col_validate2:
                if st.button("修复数据格式", use_container_width=True):
                    for obs in st.session_state.obstacles_gcj:
                        if 'selected' not in obs:
                            obs['selected'] = False
                        if 'id' not in obs:
                            obs['id'] = f"obs_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{random.randint(1000,9999)}"
                        if 'created_time' not in obs:
                            obs['created_time'] = datetime.now().isoformat()
                    if st.session_state.auto_backup:
                        save_obstacles(st.session_state.obstacles_gcj)
                    st.success("数据格式已修复")
                    st.rerun()
            
            st.markdown("---")
            
            # 导入导出
            st.markdown("### 💾 数据备份与恢复")
            col_backup1, col_backup2, col_backup3 = st.columns(3)
            with col_backup1:
                if st.button("创建完整备份", use_container_width=True):
                    backup_config()
                    st.success("备份创建成功")
            with col_backup2:
                if st.button("恢复出厂设置", use_container_width=True):
                    if st.checkbox("确认清除所有障碍物？"):
                        st.session_state.obstacles_gcj = []
                        save_obstacles([])
                        st.session_state.planned_path = create_avoidance_path(
                            st.session_state.points_gcj['A'],
                            st.session_state.points_gcj['B'],
                            [],
                            flight_alt,
                            st.session_state.current_direction,
                            safety_radius
                        )
                        st.success("已恢复出厂设置")
                        st.rerun()
            with col_backup3:
                if st.button("导出完整报告", use_container_width=True):
                    report = {
                        'export_time': datetime.now().isoformat(),
                        'obstacles_count': len(st.session_state.obstacles_gcj),
                        'obstacles': st.session_state.obstacles_gcj,
                        'flight_altitude': flight_alt,
                        'safety_radius': safety_radius,
                        'statistics': {
                            'high_obstacles': high_obs,
                            'safe_obstacles': safe_obs,
                            'total_vertices': total_vertices,
                            'avg_height': sum(heights)/len(heights) if heights else 0
                        }
                    }
                    st.download_button(
                        "下载报告",
                        data=json.dumps(report, ensure_ascii=False, indent=2),
                        file_name=f"obstacle_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
                        mime="application/json"
                    )

# ... 其余代码保持不变 ...
