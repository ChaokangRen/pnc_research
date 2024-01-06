# 类 eudm_planner分析：
相关数据结构：
```C++
enum class LatSimMode{
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
};

struct ForwardSimEgoAgent{
    decimal_t lat_range;

    //udpate on scenario-level
    OnLaneForwardSimulation::Param sim_param;

    LatSimMode seq_lat_mode;
    LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined};
    LateralBehavior seq_lat_behavior;
    bool is_cancel_behavior;
    decimal_t operation_at_seconds{0.0};

    //udate on layer-level
    LongitudinalBehavior lon_behavior{LongitudianlBehavior::kMaintain};
    LateralBehavior lat_behavior{LatralBehavior::kUndefined};

    Lane current_lane;
    StateTransformer current_stf;
    Lane target_lane;
    StateTransformer target_stf;
    Lane longterm_lane;
    StateTransformer longterm_stf;
    
    Vec2i target_gap_ids; //optional,gap ids are fixed in each layer,bug gap is changing in each step

    Vehicle vehicle;
}

struct ForwardSimAgent{
    int id = kInvalidAgentId;
    Vehicle vehicle;

    OnlaneForwardSimulation::Parm sim_param;

    ProbDistOfLatBehaviors lat_probs;
    LateralBehaviors lat_behavior{LateralBehavior::kUndefined};

    Lane lane;
    StateTransformer stf;

    decimal_t lat_range;
}

struct ForwardSimAgentSet{
    std::unordered_map<int,ForwardSimAgent> forward_sim_agents;
}

struct EfficiencyCost{
    decimal_t ego_to_desried_vel = 0.0;
    decimal_t leading_to_desried_vel = 0.0;
    decimal_t ave() const{
        return (ego_to_desired_vel + leading_to_desried_vel) / 2.0;
    }
}

struct SafetyCost{
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const {return (rss + occu_lane) / 2.0;}
}

struct NavigationCost{
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() {return lane_change_preference;}
}

struct CostStructure{
    // associate cost with micro action using this index
    int valid_sample_index_ub;
    //efficiency
    EfficiencyCost efficiency;
    //safety
    SafetyCost safety;
    //navigation
    NavigationCost navigation;

    decimal_t wight = 1.0;
    decimal_t ave() const{
        return (efficiency.ave() + safety.ave() + navigation.ave()) * wight;
    }
}

```
成员变量：
```C++
1. EudmPlannerMapItf* map_itf_{nullptr}; //调用semantic_map的指针
2. DcpTree* dcp_tree_ptr_;
3. Cfg cfg_;
4. LaneChangeInfo lc_info_;
5. decimal_t desired_velocity_{5.0};
6. decimal_t sim_time_total = 0.0;
7. std::set<int> pre_deleted_seq_ids_;
8. int ego_lane_id_{kInvalidLaneId};
9. vector<int> potential_lcl_lane_ids_;
10. vector<int> potential_lcr_lane_ids_;
11. vector<int> potential_lk_lane_ids_;
12. Lane rss_lane_;
13. StateTransformer rss_stf_;
14. RssChecker::RssConfig rss_config_;
15. RssChecker::RssConfig rss_config_strict_as_front_;
16. RssChecker::RssConfig rss_config_strict_as_rear_;

17. OnLaneForwardSimulation::Param ego_sim_param_;
18. OnLaneForwardSimulation::Param agent_sim_param_;

19. decimal_t time_stamp_;
20. int ego_id_;
21. Vehicle ego_vehicle_;

22. int winner_id_ = 0;
23. decimal_t winner_score_ = 0.0;
24. vector<int> sim_res_;
25. vector<int> risky_res_;
26. vector<string> sim_info_;
27. vector<decimal_t> final_cost_;
28. vector<vector<CostStructure>> progress_cost_;
29. vector<CostStructure> tail_cost;
30. vec_E<vec_E<Vehicle>> forward_trajs_;
31. vector<vector<LateralBehavior>> forward_lat_behaviors_;
32. vector<vector<LongitudinalBehavior>> forward_lon_behaviors_;
33. vec_E<std::unordered_map<int,vec_E<Vehicle>>> surround_trajs_;
33. decimal_t time_cost_ = 0.0;

```

core function
```C++
ErrorType EudmPlanner::RunOnce(){
    TicToc timer_runonce;

    // * Get current nearest lane id
    if(!map_itf_){
        return kWrongStatus;
    }
    if(map_itf_->GetEogVehicle(&ego_vehicle_) != kSuccess){
        return KWrongStatus;
    }
    ego_id_ = ego_vehicle_.id();
    time_stamp_ = ego_vehicle_.state().time_stamp;

    int ego_lane_id_by_pos = kInvalidLaneId;
    if(map_itf_ -> GetEgoLaneIdByPosition(std::vector<int>(),&ego_lane_id_by_pos)!=kSuccess){
        return kWrongStatus;
    }
    ego_lane_id_ = ego_lane_id_by_pos;

    decimal_t forward_rss_check_range = 130;
    decimal_t backward_rss_check_range = 130;
    decimal_t forward_lane_len = forward_rss_check_range;
    //对lane keep获取reflane
    if(map_itf_->GetRefLaneForStateByBehavior(ego_vehicle_.state(),std::vector<int>(),LateralBehavior::kLaneKeeping,forward_lane_len,backward_lane_len,false,&rss_lane_)!=kSuccess){
        //
    }

    if(rss_lane.IsValid()){
        rss_stf_ = StateTransformer(rss_lane_);
    }

    pre_deleted_seq_ids_.clear();
    int n_sequence = dcp_tree_ptr_->actrion_script().size();
    for(int i = 0;i < n_sequence;i++){
        auto action_seq = dcp_tree_ptr_->action_script()[i];
        int num_actions = action_seq.size();
        //如果当前这条动作链里面有从左到右或者从右到左的行为变化，则将该条链记录到pre_deleted_seq_ids，这样做应该是不允许这样变化
        for(int j = 0;j < num_actions;j++){
            if((action_seq[j-1].lat == DcpLatAction::kLaneChangeLeft && action_seq[j].lat == DcpLatAction::kLaneChangeRight)||(action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)){
            pre_deleted_seq_ids_ = insert(i);
           }
        }
    }

    TicToc timer;
    if(RunEudm() != kSuccess){
        return kWrongStatus;
    }
}

ErrorType EudmPlanner::RunEudm(){
    //*get relevant information
//1. 获取周围的key vehicles
    SemanticVehicleSet surrounding_semantic_vehicles;
    if(map_itf_->GetKeySemanticVehcles(&surrounding_senmantic_vehicles) != kSuccess){
        return kWrongStatus;
    }
//2. 对周围agents的仿真类实例surronding_fsagnets
    ForwardSimAgentSet surrouding_fsagents;
    GetSurroundingForwardSimAgents(surrounding_segmantic_vehicles,&surrounding_fsagents);

//

    auto action_script = dcp_tree_ptr_->action_script();
    int n_sequence = action_script.size();

    //prepare for multi-threding
    std::vector<std::thread> thread_set(n_sequence);
    PrepareMultiTreadContainers(n_sequence);

    //*threading
    TicToc timer;
    for(int i = 0;i < n_sequence;++i){
        thread_set[i] = std::thread(&EudmPlanner::SimulateActionSequence,this,ego_vehicle_,surrounding_fsagnets,action_script[i],i);
    }
    for(int i = 0;i < n_sequence;++i){
        thread_set[i].join();
    }
}

ErrorType EudmPlanner::GetSurroundingForwardSimAgents(const common::SemanticVehicleSet& surrounding_semantic_vehicles,ForwardSimAgentSet* fsagents)const {
    for(const auto &psv : surrounding_semantic_vehicle.semantic_vehicles){
        ForwardSimAgent fsagent;

        int id = psv.second.vehicle.id();
        fsagent.id = id;
        fsagent.vehicle = psv.second.vechicle;

        fsagnet.sim_param = agent_sim_param_;
        
        State state = psv.second.vehicle.state();

        // * lon
        //如果当前agent的加速 > 0，则设置期望速度为恒定，否则加上加速度的影响
        if(state.acceleration >= 0){
            fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
        }else{
            decimal_t est_vel = std::max(0.0,state.velocity +state.acceleration * sim_time_total);
            fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
        }

        // * lat
        fsagent.lat_probs = psv.second.probs_lat_behaviors;
        fsagent.lat_behavior = psv.second.lat_behavior;

        fsagent.lane = psv.second.lane;
        fsagent.stf = StateTransformer(fsagent.lane);

        // * other
        fsagent.lat_range = cfg.sim().agent().cooperative_lat_range();
        fsagents->forward_sim_agents.insert(std::make_pair(id,fsagent));
    }
}

ErrorType EudmPlanner::SimulateActionSequence(Vehicle& ego_vehicle,ForwardSimAgentSet& surrounding_fsagents,vector<DcpAction>& action_seq,const int &seq_id){
    //1. 如果在pre_deleted_seq_ids里面找到seq_id，那么返回
    if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
        sim_res_[seq_id] = 0;
        sim_info_[seq_id] = std::string("(Pre-deleted)");
        return kWrongStatus;
    }

    int n_sub_thread = 1;

    std::vector<int> sub_sim_res(n_sub_threads);
    std::vector<int> sub_risky_res(n_sub_threads);
    std::vector<std::string> sub_sim_info(n_sub_threads);
    std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
    std::vector<CostStructure> sub_tail_cost(n_sub_threads);
    vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
    std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
        n_sub_threads);
    std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
        n_sub_threads);
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(n_sub_threads);

    SimulateScenario(ego_vehicle,surrounding_fsagents,action_seq,seq_id,0,&sub_sim_res,&sub_risky_res,&sub_sim_info,&sub_progress_cost,&sub_tail_cost,&sub_forward_trajs,&sub_forward_lat_behaviors,&sub_forward_lon_behaviors,&sub_surround_trajs);

    if (sub_sim_res.front() == 0) {
        sim_res_[seq_id] = 0;
        sim_info_[seq_id] = sub_sim_info.front();
        return kWrongStatus;
    }

    // ~ Here use the default scenario
    sim_res_[seq_id] = 1;
    risky_res_[seq_id] = sub_risky_res.front();
    sim_info_[seq_id] = sub_sim_info.front();
    progress_cost_[seq_id] = sub_progress_cost.front();
    tail_cost_[seq_id] = sub_tail_cost.front();
    forward_trajs_[seq_id] = sub_forward_trajs.front();
    forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
    forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
    surround_trajs_[seq_id] = sub_surround_trajs.front();

    return kSuccess;
}

ErrorType EudmPlanner::SimulateScenario(Vehicle &ego_vehicle,ForwardSimAgentSet &surrounding_fsagents,vector<DcpAction> &action_seq,const int& seq_id,const int &sub_seq_id,vector<int> *sub_sim_res,,vector<int> *sub_risky_res,vector<string> *sub_sim_info,vector<vector<ConstStructrue>> *sub_progress_cost,vector<CostStructure> * sub_tail_cost,vec_E<vec_E<Vehicle>> *sub_forward_trajs,vector<vector<LateralBehavior>> *sub_forward_lat_behaviors,vector<vector<LongitudinalBehavior>> *sub_forward_lon_behaviors,vec_E<unordered_map<int,vec_E<Vehicle>>> *sub_surround_trajs){
//1.  构造向量 ego_traj_multilatyers
    vec_E<Vehicle> ego_traj_multilayers{ego_vehicle};
//2. 构造map结构的surround_trajs_multilayers,包含他车的trajs_multilayers
    std::unordered_map<int,vec_E<Vehicle>> surround_trajs_multilayers;
// 塞入他车起始时刻的vehicle状态
    for(const auto &p_fsa:surrounding_fsagnets.forward_sim_agents){
        surrounding_trajs_multilayers.insert(std::pair<int,vec_E<Vehicle>>(p_fsa.first,vec_E<Vehicle>({p_fsa.second.vehicle})));
    }
//构造自车的横向和纵向行为语义层。
    vector<LateralBehavior> ego_lat_behavior_multilayers;
    vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
    vector<CostStructure> cost_multilayers;
    std::set<int> risky_ids_multilayers;

//setup ego longitudinal sim config
//对自车纵向仿真进行配置
    ForwardSimEgoAgent ego_fsagent_this_layer;
    ego_fsagent_this_layer.vehicle = ego_vehicle;
    UpdateSimSetupForScenario(action_seq,&ego_fsagent_this_layer);

    ForwardSimAgentSet surrounding_fsagent_this_layer = surrounding_fsagents;

    int action_ref_lane_id = ego_lane_id;
    bool is_sub_seq_risky = false;
    vector<DcpAcion> action_seq_sim = action_seq;

    for(int i = 0;i < static_cast<int>(action_seq_sim.size());++i){
        auto action_this_layer = action_seq_sim[i];

        //对于每个action，我们在此更新context，比如lane,stf,desired_vel,social_force_masks
        //对于每个action,context信息不会改变，因此我们可以在每一步中使用它，一个低级的反应式控制器可以实现且不会占用太多资源

        if(kSuccess != UpdateSimSetupForLayer(action_this_layer,surrounding_fsagents_this_layer,&ego_fsagent_this_layer)){
            (*sub_sim_res)[sub_seq_id] = 0;
            (*sub_sim_info)[sub_seq_id] += std::string("(update setup F)");
            return kWrongStatus;
        }

        //simulate this action(layer)
        vec_E<common::Vehicle> ego_traj_multisteps;
        std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
        if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
            (*sub_sim_res)[sub_seq_id] = 0;
            (*sub_sim_info)[sub_seq_id] +=
            std::string("(Sim ") + std::to_string(i) + std::string(" F)");
        return kWrongStatus;
        }
    }
}

ErrorType EudmPlanner::UpdateSimSetupForScenario(const vector<DcpAction> &actin_seq,ForwardSimEgoAgent *ego_fsagent) const{
    //Get the type of lateral action sequence
    LateralBehavior seq_lat_behavior;
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    //找出当前序列是否存在变道，如果有那么在何时变道，变道行为是左还是右
    ClassifyActionSeq(action_seq,&operator_at_seconds,&seq_lat_behavior,&is_cancel_behavior);

    //填充数据
    ego_fsagent->operation_at_seconds = operation_at_seconds;
    ego_fsagent->is_cancel_behavior = is_cancel_behavior;
    ego_fsagent->seq_lat_behavior = seq_lat_behavior;

    //get action sequence type
    if(is_cancel_behavior){
        ego_fsagent->lat_behavior_long_term = LateralBehavior::kLaneKeeping;
        ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
    }else{
        if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
            ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
        } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
            ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
        } else {
            ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
        }
        ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
    }

    decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
    simulator::IntelligentDriverModel::Param idm_param_tmp;
    idm_param_tmp = ego_sim_param_.idm_param;

    switch(action_seq[1].lon){
    case DcpLonAction::kAccelerate:{
    //如果纵向行为是加速：idm的期望速度是当前车速加上cfg的acc_cmd_vel_gap()和当前期望速度的更小值。
        idm_param_tmp.kDesiredVelocity = std::min(desried_vel +cfg_.sim().acc_cmd_vel_gap(),desired_velocity_);
    //最小间距是
        idm_param_tmp.kMinimumSpacing *= 
            (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
    //最小headway time是：
        idm_param_tmp.kDesiredHeadwayTime *= (1.0 - cfg_.sim().ego().log_aggressive_ratio());
        break;
    }
    case DcpLatAction::kDecelerate:{
        idm_param_tmp.kDesiredVelocity = std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(),0.0),desired_velocity_);
        break;
    }
    case DcpLonAction::kMaintain:{
        dim_param_tmp.kDesiredVelocity = std::min(desired_vel,desired_velocity_);
        break;
    }
    default:{
        assert(false);
    }
    }
    ego_fsagent->sim_param = ego_sim_param_;
    ego_fsagent->sim_param.idm_param = idm_param_tmp;
    ego_fsagent->cfg_.sim().ego().cooperative_lat_range();

    return kSuccess;
}

ErrorType EudmPlanner::ClassifyActionSeq(const vector<DcpAction> &action_seq,decimal_t *operation_at_seconds,LateralBehavior *lat_behavior,bool *is_cancel_operation)const{
    decimal_t duration = 0.0;
    decimal_t operation_at = 0.0;

    bool find_lat_active_behavior = false;
    *is_cancel_operation = false;

    for(const auto &action : action_seq){
        //如果find_lat_actiove_behavior == false
        if(!find_lat_active_behavior){
            //如果当前行为是change left
            if(action.lat == DcpLatAction::kLaneChangeLeft){
                *operation_at_seconds = duration;
                *lat_behavior = common::kLaneChangeLeft;
                find_lat_active_behavior = true;
            }
            //如果当前行为是change right
            if (action.lat == DcpLatAction::kLaneChangeRight) {
                *operation_at_seconds = duration;
                *lat_behavior = common::LateralBehavior::kLaneChangeRight;
                find_lat_active_behavior = true;
            }
        }else{
            if(action.lat == DcpLatAction::kLaneKeeping){
                *is_cancel_operation = true;
            }
        }
        duration += action.t;
    }
    if(!find_lat_active_behavior){
        *operation_at_seconds = duration + cfg_.sim().duration().layer();
        *lat_behavior == commmon::LateralBehavior::kLaneKeeping;
        *is_cancel_operation = false;
    }
    return kSuccess;
}

ErrorType EudmPlanner::UpdateSimSetupForLayer(const DcpAction &action,
                                    const ForwardSimAgentSet &other_fsagent,
                                    ForwardSimEgoAgent *ego_fsagent) const{
    LateralBehavior lat_behavior;
    LongitudinalBehavior lon_behavior;
    //把action中的横向行为和纵向行为取出来
    if(TranslateDcpActionToLonLatBehavior(action,&lat_behavior,&lon_behavior) != kSuccess){
        return kWrongStatus;
    }
    ego_fsagent->lat_behavior = lat_behavior;
    ego_fsagnet->lon_behavior = lon_behavior;

    //获取仿真车道的长度
    auto state = ego_fsagent->vehicle.state();
    decimal_t forward_lane_len =std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),cfg_.sim().ref_line().forward_len_min()),cfg_.sim().ref_line().forward_len_max());

    //获取current lane
    common::: Lane_current;
    if(map_itf_->GetRefLaneForStateByBehavior(state,std::vector<int>(),LateralBehavior::kLaneKeeping,forward_lane_len,cfg_.sim().ref_line().backward_len_max(),false,&lane_current) != kSuccess){
        return kWrongStatus;
    }

    ego_fsagent->current_lane = lane_current;
    ego_fsagnet->current_stf = common::StateTransformer(lane_current);
    
    common::Lane lane_target;
    if(map_itf_->GetRefLaneForStateByBehavior)(state,vector<int>,ego_fsagent->lat_behavior,forward_lane_len,cfg_.sim().ref_line().backward_len_max(),false,&lane_target) != kSuccess){
        return kWrongStatus;
    }

    ego_fsagent->target_lane = lane_target;
    ego_fsagent->target_stf = common::StageTransformer(lane_target);

    common::Lane lane_longterm;
    if(map_itf_->GetReflaneForStateByBehavior(state,vector<int>,ego_fsagent->lat_behavior_longterm,forward_lane_len,cfg_.sim().ref_line().backward_len_max(),false,&lane_longterm) != kSuccess){
        return kWrongStatus;
    }
    ego_fsagent->longterm_lane = lane_longterm;
    ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

    //如果当前自车的横向行为不是lane keeping
    if(ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping){
        common::VehicleSet other_vehicles;
        //遍历所有其他sim_agent,将id和vehicle组成一个pari
        for(const auto &pv : other_fsagent.forward_sim_agents){
            other_vehicles.vehicles.insert(std::make_pair(pv.first,pv.second.vehicle));
        }

        bool has_front_vehicle = false,has_rear_vehicle = false;
        common::Vehicle front_vehicle,rear_vehicle;
        common::FrenetState front_fs,rear_fs;
        //获取当前车辆的前车和后车
        mpt_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(ego_fsagent->target_lane,state,other_vehicles,&has_front_vehilce,&front_vehicle,&front_fs,&has_rear_vehicle,&rear_vehicle,&rear_fs);

        ego_fsagent->target_gap_ids(0) = has_front_vehicle?front_vehicle.id(),-1;
        ego_fsagent->target_gap_ids(1) = has_rear_vehicle? rear_vehicle.id(),-1;

        if(cfg_safety().rss_for_layers_enable()){
            //rss 检查
            common::FrenetState ego_fs;
            if(kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(ego_fsagent->vehicle.state(),&ego_fs)){
                return kWrongStatus;
            }
            //自车后轴中心到前保险杠距离
            decimal_t s_ego_fbumper = ego_fs.vec_s[0] + ego_fsagent->vehicle.param().length() / 2.0 + ego_fsagent->vehciel.param().d_cr();
            //自车后轴中心到后保险杠距离
            decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 + ego_fsagent->vehicle.param().d_cr();
            
            if(has_front_vehicle){
                //前车后轴中心到后保险杠距离
                decimal_t s_front_rbumper = front_fs_vec_s[0] - front_vehicle.param().length() / 2.0 + front_vehicle.param().d_cr();
                decimal_t rss_dist;
                common::RssChecker::CalculateSafeLongitudinalDistance(ego_fsagent->vehicle.state().velocity,front_vehicle.state().velocity.common::RssChecker::LongitudinalDirection::Front,rss_config_strict_as_rear_,&rss_dist);
                if(s_front_rbumper- s_ego_fbumper < rss_dist){
                    return kWrongStatus;
                }
            }
            //后车rss检查
            if (has_rear_vehicle) {
                decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
                decimal_t rss_dist;
                common::RssChecker::CalculateSafeLongitudinalDistance(
                ego_fsagent->vehicle.state().velocity,
                rear_vehicle.state().velocity,
                common::RssChecker::LongitudinalDirection::Rear,
                rss_config_strict_as_front_, &rss_dist);

                if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
                    // violate strict RSS
                    return kWrongStatus;
                }
            }
        }
    }
    return kSuccess;
}

ErrorType EudmPlanner::SimulateSingleAction(const DcpAction &action,const ForwardSimEgoAgent &ego_fsagent_this_layer,const ForwardSimAgentSet &surrounding_fsagents_this_layer,vec_E<common::Vehicle> *ego_traj,std::unordered_map<int,vec_E<common::Vehicle>> *surround_trajs){
    // prepare contaniners
    ego_traj->clear();
    surround_trajs->clear();

    for(const auto &v : surrounding_fsagents_this_layer.forward_sim_agents){
        surround_trajs->insert(std::pair<int,vec_E<common::Vehicle>>(v.first,vec_E<common::Vehicle>()));
    }

    //simulation time steps
    std::vector<decimal_t> dt_steps;
    GetSimTimeSteps(action,&dt_steps);

    ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
    ForwardSimAgentSet surrouding_fsagents_this_step = surrounding_fsagents_this_layer;

    for(int i = 0;i < static_cast<int>(dt_steps,size());i++){
        decimal_t sim_time_step = dt_steps[i];

        State ego_state_cache_this_step;
        std::unordered_map<int,State> others_state_cache_this_step;

        //将所有vehicle塞入all_sim_vehicles
        VehicleSet all_sim_vehicles;//include ego vehicle
        all_sim_vehicles.vehicles.insert(std::make_pair(ego_fsagent_this_step.vehicle.id(),ego_fsagent_this_step.vehicle));
        for(cosnt auto &v : surrouding_fsagents_this_step.forward_sim_agents){
            all_sim_vehicles.vehiclees.insert(std::make_pair(v.first,v.second.vehicle));
        }
        //对于自车agent
        {
            all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);
            State state_output;
            if(kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,all_sim_vehicles,sim_time_step,&state_output)){
                return kWrongStatus;
            }
            common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
            v_tmp.set_state(state_output);
            ego_traj->push_back(v_tmp);
            ego_state_cache_this_step = state_output;

            all_sim_vehicles.vehicles.at(ego_id).set_id(ego_id_);
        }

        //对于他车
        {
            for(const auto &p_fsa:surrounding_fsagents_this_step.forward_sim_agents){
                State state_output;
                if(kSuccess != SurroundingAgentForwardSim(p_fsa.second,all_sim_vehicles,sim_time_step,&state_output)){P
                    return kWrongStatus;
                }
                Vehicle v_tmp = p_fsa.second.vehicle;
                v_tmp.set_state(state_output);
                surround_trajs->at(p_fsa.first).push_back(v_tmp);
                others_state_cache_this_step.insert(std::make_pair(p_fsa.first).set_id(p_fsa.first));
            }
        }
        // * update sim state after steps
        ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
        for (const auto &ps : others_state_cache_this_step) {
            surrounding_fsagents_this_step.forward_sim_agents.at(ps.first).vehicle.set_state(ps.second);
        }
        return kSuccess;
    }
}

ErrorType EudmPlanner::GetSimTimeSteps(const DcpActin &action,std::vector<decimal_t> *dt_steps)const{
    decimal_t sim_time_resolution = cfg_.sim().duratin().step();
    decimal_t sim_time_total = action.t;
    int n_1 = std::floor(sim_time_total / sim_time_resolution);
    decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
    std::vector<decimal_t> steps(n_1,sim_time_resolution);
    if(fabs(dt_remain) > kEPS){
        steps.insert(steps.begin(),dt_remain);
    }
    *dt_steps = steps;
    return kSuccess;
}

ErrorType EudmPlanner::EgoAgentForwardSim(const ForwardSimEgoAgent &ego_fsagent,const common::VehicleSet &all_sim_vehicles,const decimal_t &sim_time_step,common::State *state_out)const{
    common::State state_output;
    //1.如果当前自车的行为是lanekeeping
    if(ego_fsagent.lat_behavior == kLaneKeeping){
        //lane keeping情况只考虑自车道上前车的影响
        Vehicle leading_vehicle;
        decimal_t distance_residual_ratio = 0.0;
        //1.1 获取前车state,以及自车到前车的距离比率
        if(map_itf_->GetLeadingVehicleOnLane(ego_fsagent.target_lane,ego_fsagent.vehicle.state(),all_sim_vehicles,ego_fsagent.lat_ranege,&leading_vehicle,&distance_residual_ratio) == kSuccess){
            bool is_collision = false;
            //检车自车与前车是否会发生碰撞
            map_itf_->CheckCollisionUsingState(ego_fsagent.vehicle.param(),ego_fsagent.vehicle.state(),leading_vehicle.param(),leading_vehicle.state(),&is_collision);
            if(is_collision){
                return kWrongStatus;
            }
        }

        decimal_t lat_track_offset = 0.0;
        if(planning::OnLaneForwardSimulation::PropagateOnceAdvancedLk(ego_fsagent.target_stf,ego_fsagent.vehicle,leading_vehicle,lat_track_offset,sim_time_step,ego_fsagent.sim_param,&state_output)!= kSuccess){
            return kWrongStatus;
        }
    }else{
        // Lane changing,consider multiple vehicles
        common::Vehicle current_leading_vehicle;
        decimal_t distance_residual_ratio = 0.0;

        if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
            // ~ with leading vehicle
            bool is_collision = false;
            map_itf_->CheckCollisionUsingState(
                ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
                current_leading_vehicle.param(), current_leading_vehicle.state(),
                &is_collision);
            if (is_collision) {
                return kWrongStatus;
            }
        }
        //获取前车和后车的gap
        common::Vehicle gap_front_vehicle;
        if(ego_fsagent.target_gap_ids(0) != -1){
            gap_front_vehicle = all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
        }
        common::Vehicle gap_rear_vehicle;
        if (ego_fsagent.target_gap_ids(1) != -1) {
            gap_rear_vehicle =all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
        
        decimal_t lat_track_offset = 0.0;
        auto sim_param = ego_fsagent.sim_param;
        // 如果自车id合法且evasive使能
        if(gap_rear_vehicle.id() != kInvalidAgentId && cfg_.sim().ego().evasie().evasive_enable()){
            FrenetState ego_on_tarlane_fs;
            FrenetState rear_on_tarlane_fs;
            //获取自车和后车在target lane的frenet坐标
            if(ego_fsagent.target_stf.GetFrenetStateFromState(ego_fsagent.vehicle.state()，&ego_on_tarlane_fs) == kSuccess && ego_fsagent.target_stf.GetFrenetStateFromState(gap_rear_vehicle.state()，&rear_on_tarlane_fs) == kSuccess){
                //rss check for evasive behavior
                bool is_rss_safe = true;
                common::RssChecker::LongitudinalViolateType type;
                decimal_t rss_vel_low,rss_vel_up;
                //rss检查：是否通过rss检查
                common::RssChecker::RssCheck(ego_fsagent.vehicle,gap_rear_vehicle,ego.fsagent.traget_stf,rss_config_strict_as_rear_,&is_rss_safe,&type,&rss_vel_low,&rss_vel_up){
                    //如果没有通过RSS检查
                    if(!is_rss_safe){
                        //如果返回的类型是TooSlow
                        if(type == common::RssChecker::LongitudinalViolateType::TooSlow){
                            //期望速度取当前期望速度和rss最低速度加上lon_extraspeed(10) (...不太理解为什么这样做)
                            sim_param.idm_param.kDesiredVelocity = std::max(sim_param.idm_param.kDesiredVelocity,rss_vel_low +cfg_.sim().ego().evasive().lon_extraspeed());
                            sim_param.idm_param.kDesiredHeadwayTime = cfg_.sim().ego().evasive().head_time();
                            sim_param.idm_param.kAcceleration = cfg_.sim().ego().evasive().lon_acc();
                            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
                        }
                    }
                    //如果virtual_barrier_enable使能
                    if(cfg_.sim().ego().evasive().virtual_barrier_enable()){
                        //如果自车到后车的距离小于后车车长加上车速乘barrier_tic
                        if(ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] < gap_rear_vehicle.param().length() + cfg_.sim().ego().evasice().virtual_barrier_tic() * gap_rear_vehicle.state().velocity){
                            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
                        }
                        common::FrenetState front_on_tarlan_fs;
                        if(gap_front_vehicle.id() != kInvalidAgentId && ego_fsagent.target_stf.GetFrenetStateFromState(gap_front_vehicle.state(), &front_on_tarlane_fs)==kSuccess) {
                            //如果前车到自车的距离小于自车车长加上自车车速乘以barrier_tic
                            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] < ego_fsagent.vehicle.param().length() + cfg_.sim().ego().evasive().virtual_barrier_tic() *   ego_fsagent.vehicle.state().velocity) {
                                lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
                            }
                        }
                    }
                }
            }
        }

        if(planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(ego_fsagent.current_stf, ego_fsagent.target_stf,ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param, &state_output) != kSuccess){
            return kWrongStatus;
        }
    }
    *state_out = state_output;
    return kSuccess;
}}
```