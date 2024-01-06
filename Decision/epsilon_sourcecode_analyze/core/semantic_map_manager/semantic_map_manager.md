类： SemanticMapManager
成员变量：
1. time_stamp:应该是时间戳
2. pred_time = 5.0:预测总时间
3. pred_step_ = 0.2: 预测迭代步长
4. nearest_lane_range_ = 1.5:有点像从车道中线到车道线的距离
5. lane_range_ = 10.0：应该是自车周围可见的车道横向范围
6. max_distance_to_lane = 2.0:距离车道线的最大距离
7. bool has_fast_lut = false:暂时看不出来具体用途
8. local_lanes:局部车道的集合
9. local_to_segment_lut:
10. segmengt_to_local_lut:
11. local_lane_length_forward = 250:前向最远可视车道线距离
12. local_lane_length_backward = 150:后向最远可视车道线距离
13. ego_id :自车的id
14. agent_config_path:配置文件路径
15. use_right_hand_axis:使用右手准则
16. ego_vehicle:自车的车辆实例
17. obstacle_map:障碍物map
18. `std::set<std::array<decimal_t,2>> obstacle_grids`:障碍物网格位置
19. VehicleSet:surrounding_vehicles:周围车辆类集合
20. semantic_surrounding_vehicles:周围车辆可能的语义集合
21. key_vehicle:关键车辆
22. semantic_key_vehicles:关键车辆的语义
23. `vector<int> key_vehicle_ids`:关键车辆的id
24. `vector<int>uncertain_vehicle_ids`:不确定车辆的id
25. `LaneNet whole_lane_net`:整个路网
26. `LaneNet surrounding_lane_net` 自车周围路网
27. `SemanticLaneSet semantic_lane_set`:路网语义
28. `SemanticBehivior ego_behavior`:语义行为
29. `unordered_map<int,vec_E<State>> openloop_pred_trajs`:开环预测轨迹
30. `TicToc global_timer`:全局时间
31. `TrafficSignalManager: traffic_singal_manager`:交通灯manager
32. `ConfigLoader *p_config_loader`:
33. `RssChecker rss_checker`rss安全检测
34. `is_simple_lane_structure` 是否是简单道路结构

成员函数：
1. ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,bool *res):
该函数从函数名称来看主要是用于碰撞检测，接受一组坐标并返回结果。  
函数实现主要通过obstacle_map_.CheckIfEqualUsingGlobalPosition(p,GridMap2D::OCCUPIED,res),从该函数来看，应该是检测每一个障碍物是否与该点碰撞来确定。进一步细节需要跳到obstacle_map的实现去看。
2. ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,ObstacleMapType *res);


3.
```C++
MobilRuleBasedBehaviorPrediction(vehicle,nearby_vehicles,*res){
//输入:vheicle、nearby_vehicles  
//输出:ProbDistOfLatBehaviors *res(横向行为的概率)
//1. 初始化
    vec_E<common::Lane> lanes;
    vec_E<common::Vehicle> leading_vehicles;
    vec_E<common::Vehicle> following_vehicles;
    vec_E<common::FrenetState> leading_frenet_states;
    vec_E<common::FrenetState> follow_frenet_states;
//2. 构建 behaviors{keep,left,right}
    std::vector<LateralBehavior> behavior{LateralBehavior::kLaneKeeping,LateralBehavior::kLaneChangeLeft,LateralBehavior::kLaneChangeRight};
//3.遍历behaviors
      for (const auto &behavior : behaviors) {
        //3.1 根据当前行为来获取可能的reflane
        GetRefLaneForStateByBehavior(vehicle.state,vector<int>(),behavior,lane_radius,lane_radius,false,&ref_lane)
        //3.2 确定当前vehicle的前车和后车
        bool has_leading_vehicle_false,has_following_vehicle = false;
        Vehicle leanding_vehicle,following_vehilce;
        FrenetState leading_frenet_state,following_frenet_state;
        GetLeadingFollowingVehiclesFrenetStateOnLane(ref_lane,vehicle.state(),nearby_vehicles,&has_leading_vehicle,&leading_vehicle, &leading_frenet_state, &has_following_vehicle,&following_vehicle, &following_frenet_state);
        //填入相关vehicle
        lanes.push_back(ref_lane);
        leading_vehicles.push_back(leading_vehicle);
        following_vehicles.push_back(following_vehicle);
        leading_frenet_states.push_back(leading_frenet_state);
        follow_frenet_states.push_back(following_frenet_state);

}
//4. 对行为进行预测
if (common::MobilBehaviorPrediction::LateralBehaviorPrediction(
          vehicle, lanes, leading_vehicles, leading_frenet_states,
          following_vehicles, follow_frenet_states, nearby_vehicles,
          res) != kSuccess) {
    return kWrongStatus;
  }

}
```

```C++
成员函数
GetRefLaneForStateByBehavior(State,navi_path,behavior,max_forward_len,max_back_len,is_high_quality,*lane):
1. 构造三自由度状态: Vec3f state_3dof(state.vec_position(0),state.vec_position(1),state.angle);
2. 声明: int current_lane_id;dicimal_t distance_to_lane,arc_len;
3. if(GetNearestLaneIdUsingState(state_3dof,navi_path,&current_lane_id,&distance_to_lane,&arc_len) != kSuccess){
    return kWongStatus.
}
//如果函数GetNearestLaneIdUsingState()返回值不是success,则返回wrong。
//该函数意思为获取当前距离和角度最近的那条车道的相关数据:id,dist,arc_len
4. if(distance_to_lane >max_distance_to_lane) return kWongStatus;
5. int target_lane_id
6. if(GetTargetLaneId(current_lane_id,behavior,&target_lane_id)!=kSuccess){
    return kWrongStatus;
    //获取当前目标车道的id
}
7. if(agent_config_info_.enable_fast_lane_lut && has_fast_lut_){
    if(segment_to_local_lut_.end() != segment_to_local_lut_.find(target_lane_id)){
        //我们仅选择几条候选车道的第一条车道
        int id = *segment_to_local_lut_.at(target_lane_id).begin();
        *lane = local_lanes.at(id);
        return kSuccess;
    }
8. //reflane的长度应该符合最大速度和最大前面仿真时间，当前设置为30m/s和7.5s
   vec_Vecf<2> samples;
   if(GetLocalLaneSamplesByState(state,target_lane_id,navi_path,max_forward_len,max_back_len,&samples) != kSuccess){
    return kWrongStatus;
    //通过当前状态获取局部车道的采样点
   }
9. if(kSuccess != GetLaneBySampledPoints(samples,is_high_quality,lane)){
    //利用采样点来生成样条曲线。
    reuturn kWrongStatus;
    }
10. return kSuccess;
}
```

```C++
ErrorType SemanticMapManager::GetLocalSamplesByState(state,lane_id,navi_path,max_reflane_dist,max_backward_dist,vec_Vecf<2> *samples)const{
1. if(semantic_lane_set_.semantic_lanes.count(lane_id) == 0){
        return kWrongStatus;
    }
2. decimal_t arclen = 0.0;
3. Lane target_lane = semantic_lane_set.semantic_lanes.at(lane_id).lane;
4. target_lane.GetArcLengthByVecPosition(state.vec_position,&arclen);
//1. 通过当前状态位置获取lane上离该点最近的s
5. decimal_t accum_dist_backward = 0,vector<int> ids_back;
//2. 计算accum_dist_backward
6. if(arclen < max_backward_dist){//如果当前位置s小于最大后向距离，则执行：
    accum_dist_backward += arclen;
    int id_tmp = lane_id;//id_tmp首先等于当前id
    while(accum_dist_backward <max_backward_dist){
        //获取id_tmp的父id
        vector<int> father_ids = semantic_lane_set_.semantic_lanes.at(id_tmp).father_id;
        //如果父id不为空
        if(!father_ids.empty()){
            int father_id = father_ids.frong();//取father_ids里面最前的
            //遍历father_ids
            for(auto &id:father_ids){
                //如果id在navi_path里面存在，则father_id = id,并跳出;
                if(find(navi_path.begin(),navi_path.end(),id)!=navi_path().end()){
                father_id = id;
                break;
                }
            }
            //accum_dist_backward 加上 father_id的长度
            accum_dist_backward+= semantic_lane_set_.semantic_lanes.at(father_id).length;
            ids_back.push_back(father_id);
            id_tmp = father_id;
        }else{
            break;
        } 
    }
   }else{
    accum_dist_backward = arclen;
   }

   std::reverse(ids_back.begin(),ids_back.end());

//3. 同样的方法计算accum_dist_forward;
    std::vector<int> ids_front;
    decimal_t accum_dist_forward = 0.0;
    //省略部分代码
//4. 将ids_back和idsfront合并到一起
  std::vector<int> lane_id_all;
  lane_id_all.insert(lane_id_all.end(), ids_back.begin(), ids_back.end());
  lane_id_all.insert(lane_id_all.end(), ids_front.begin(), ids_front.end());
//5. 计算raw_samples
//具体做法是从surrounding_lane中找到lane_id_all的所有lane的points填入到raw_samples中
    vec_Vecf<2> raw_samples;
    for(const auto &id : lane_id_all){
        //如果raw_samples不为空且在surrounding_lane_set里面有这个id的lane且lanepoints>0，这样做应该是避免重复点
        if(raw_samples.empty() && (int)surrounding_lane_net.lane_set.at(id).lane_points.size()>0){
            //取该lane的第一个点。
            raw_samples.push_back(surrounding_lane_net_.lane_set.at(id).lane_points[0]);
        }
        for (int i = 1;i < (int)surrounding_lane_net_.lane_set.at(id).lane_points.size();++i) {
            raw_samples.push_back(surrounding_lane_net_.lane_set.at(id).lane_points[i]);
        }
    }
//6. 由采样得到的raw_samples拟合出long_lane出来
    common::Lane long_lane;
    if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=kSuccess) {
    return kWrongStatus;
  }
//7.获取得到需要的samples
    decimal_t acc_dist_tmp;
    decimal_t sample_start = std::max(0.0,accum_dist_backward-max_backward_dist);
    decimal_t forward_sample_len = std::min(max_reflane_dist,accum_dist_forward);
    //sampleLane作用就是从sample_start开始到end，对longLane上的点进行采样得到一组采样点。
    SampleLane(long_lane,sample_start,sample_start+forward_sample_len+std::min(accum_dist_backward,max_backward_dist),1.0,samples,&acc_dist_tmp);

    return kSuccess;
}
```

```C++
ErrorType GetLeadingVehicleOnLane(Lane &ref_lane,State ref_state,VehicleSet &vehicle_set,decimal_t &lat_range,Vehicle *leading_vehicle,decimal_t *distance_residual_ratio)const{
    //1. 初始化frenet坐标系
    common::StateTransformer stf(ref_lane);
    common::FrenetState ref_fs;
    Vecf<2> lane_pt;
    if(stf.GetFrenetStateFromState(ref_state,&ref_fs)!=kSuccess){
        return kWronogStatus;
    }
    ref_lane.GetPositionByArcLength(ref_fs.vec_s[0],&lane_pt);

    //2.
    const decimal_t lane_width = 3.5;
    const decimal_t search_lat_radius = lat_range;
    const decimal_t max_forward_search_dist = 120.0;
    decimal_t search_lon_offset = 0.0;
    decimal_t resolution = search_lat_radius / 1.4;

    int leading_vehicle_id = kInvalidAgentId;

    bool find_leading_vehicle_in_set = false;
    bool find_occupied = false;
    Vehicle virtual_vehicle;
    //寻找leading vehicle
    for(decimal_t s = ref_fs.vec_s[0] + resolution + search_lon_offset;s <ref_fs.vec_s[0] + max_forward_search_dist +search_lon_offset; s+= resolution){
        decimal_t delta_s = s - ref_fs.vec_s[0];
        ref_lane.GetPositionByArcLenght(s,&lane_pt);

        for(const auto &entry : vehicle_set.vehicles){
            if(entry.second.id() == kInvalidAgentId){
                continue;
            }
            //如果该车的距离与自车的距离小于search_lat_radius，则找到
            if((lane_pt - entry.second.state().vec_position).squaredNorm() <  search_lat_radius * search_lat_radius){
                find_leading_vehicle_in_set = true;
                leading_vehicle_id = entry.frist;
                *distance_residual_ratio = (max_forward_search_dist - delta_s) / max_forward_search_dist;
                break;
            }
        }
        if (find_leading_vehicle_in_set)
            break;
    }
    if (find_leading_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(leading_vehicle_id);
    *leading_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
    
}
```

```C++
ErrorType MobilBehaviorPrediction::LateralBehaviorPrediction(Vehicel &vehicle,vec_E<lane> &lanes,vec_E<Vehicle> &leading_vehicles,vec_E<FrenetState> &leading_frenet_states,vec_E<Vehicle> &following_vehicles,vec_E<FrenetState> &follow_frenet_states,VehicleSet &nearby_vehicles,ProbDistOfLatBehaviors *res){
//1. 检测lanes是否有三条
    if(lanes.size() != 3){
        return kWrongStatus;
    }
// MOBIL paper中要用的参数
    decimal_t acc_c = 0.0;
    decimal_t acc_o = 0.0,acc_o_tilda = 0.0;
    decimal_t politeness_coeff = 0.0;

    bool is_lcl_safe = false;
    bool is_lcr_safe = false;

    decimal_t mobil_gain_left = -kInf;
    decimal_t mobil_gain_right = -kInf;

//2. 如果车速很低，就确认为lane keeping
    decimal_t desired_vel = vehicle.state().velocity;
    if (fabs(desired_vel) < kBigEPS) {
        res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
        res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
        res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0);
        res->is_valid = true;
    return kSuccess;
    }

//3. 对于当前车道
    Lane lk_lane = lanes[0];
    if(lk_lane.IsVaild()){
        Vehicle leading_vehicle = leading_vehicles[0];
        leading_fs = leading_frenet_states[0];
        following_vehicle = following_vehicles[0];
        following_fs = follow_frenet_states[0];

        StateTransformer stf(lk_lane);
        FrenetState ego_frenet_state;
        stf.GetFrenetStateFromState(vehicle.state(),&ego_frenet_state);

        if(MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(ego_frenet_state,leading_vehicle,leading_fs,following_vehicle,following_fs,&acc_o,&acc_o_tilda,&acc_c) != kSuccess){
            return kWrongStates;
        }
    }else{
        return kWrongStates;
    }
// 4.对于左侧车道
//计算方式与上述差不多，多了计算左侧变道的增益
    mobil_gain_left = d_a_c + politeness_coeff * (d_a_n + d_a_o);
// 5.右侧同样的方式
// 6. 最后计算变道的概率
 RemapGainsToProb(is_lcl_safe,mobil_gain_left,is_lcr_safe,mobil_gain_right,*res);
 return kSuccess;
}

```

```C++
ErrorType TrajectoryPredictionForVehicle(Vehicle &vehicle,Lane &lane,decimal_t &t_pred,decimal_t &t_step,vec_E<State> *traj){
    planning::OnLaneFsPredicitor::GetPredictedTrajectory(lane,vehicle,t_pred,traj);
}
```