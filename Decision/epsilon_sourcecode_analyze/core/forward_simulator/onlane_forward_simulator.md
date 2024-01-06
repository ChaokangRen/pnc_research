```C++
static ErrorType PropagateOnce(StateTransformer& stf,Vehicle &ego_vhicle,Vehicle &leading_vehicle,decimal_t &dt,Param &param,State *desired_state){
    State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.state();
    auto sim_param = param;

    //1. 计算方向盘转角
    bool steer_calculation_failed = false;
    FrenetState current_fs;
    if(stf.GetFrenetStateFromState(current_state,&current_fs) != kSuccess || current_fs.vec_s[1] < -kEPS){
        steer_calculation_failed = true;
    }

    decimal_t steer,velocity;
    if(!steer_calcuation_failed){
        decimal_t approx_lookhead_dist = std::min(std::max(param.steer_control_min_lookahead_dist,current_state.velocity *param.steer_control_gain),param.steer_control_max_lookahead_dist);
        if(CalcualateSteer(stf,current_state,current_fs,wheelbase_len,Vec2f(apporx_lookahead_dist,0.0),&steer) != kSuccess){
            steer_calculation_failed = true;
        }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;

    if(param.auto_decelerate_if_lat_failed && steer_calculation_failed){
        sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0,sim_vel);

    //2. 计算速度
    FrenetState leading_fs;
    //如果车辆id异常或者前车在frenet坐标系中的位置异常
    if(leading_vehicle.id() == kInvalidAgentId || stf.GetFrenetStateFromState(leading_vehicle.state(),&leading_fs) != kSuccess){
        //不带前车计算期望速度
        CalculateVeclocityUsingIdm(current_state.velocity,dt,sim_param,&velocity);
    }else{
        //带前车计算期望速度
        //对于 IDM，减去车辆长度即可得到本车与领先车辆之间的“净”距离。
        decimal_t eqv_vehicle_len;
        GetIdmEquvalentVehicleLength(stf,ego_vehicle,leanding_vehicle,leading_fs,&eqv_vehicle_len);
        sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

        CalculateVelocityUsingIdm(current_fs.vec_s[0],current_state.velocity,leading_fs.vec_s[0],leading_vehicle.state(),dt,sim_param,&velocity);
    }

    //3. 获取期望的车辆状态
    CalculateDesiredState(current_state,steer,velocity,wheelbase_len,dt,sim_param,desired_state);
    return kSuccess;
}

static ErrorType PropageOnceAdvancedLK(const common::StateTransformer& stf,const common::Vehicle& ego_vehicle,const Vehicle& leading_vehicle,const decimal_t &lat_track_offset,const decimal_t & dt,const Param& param,State* desired_state){
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    //step1:计算转角
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if(stf.GetFrenetStateFromState(current_state,&current_fs)!= kSuccess || current_fs.vec_s[1] < -kEPS){
        //自车的frenet state非法或者自车处于倒挡
        steer_calculation_failed = true;
    }

    decimal_t steer,velocity;
    if(!steer_calculation_failed){
        decimal_t approx_look_dist = std::min(std::max(param.steer_control_min_lookahead_dist,current_state.velocity * param.steer_control_gain),param.steer_control_max_lookahead_dist);
        //计算出转角
        if(CalculateSteer(stf,current_state,current_fs,wheelbase_len,Vec2f(approx_lookahead_dist,lat_track_offset),&steer) != kSuccess){
            steer_calculation_failed = true;
        }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if(param.auto_decelerate_if_lat_failed && steer_calculation_failed){
        sim_vel = 0.0;
    }
    sim_param.idm_param.kDesriedVelocity = std::max(0.0,sim_vel);
    //2. 计算速度
    common::FrenetState leading_fs;
    //2.1 如果前车id非法或者无法获取前车的fs_state
    if(leading_vehicle.id() == kInvalidAgentId || stf.GetFrenetStateFromState(leading_vehicle.state(),&leading_fs) != kSuccess){
        //使用Idm模型计算车速
        CalculateVelocityUsingIdm(current_state.velocity,dt,sim_param,&velocity);
    }else{
        //对于 IDM，减去车辆长度即可得到本车与领先车辆之间的“净”距离。
        decimal_t eqv_vehicle_len;
        GetIdmEquivalentVehicleLength(stf,ego_vehicle,leading_vehicle,leading_fs,&eqv_vehicle_len);

        sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

        CalculateVelocityUsingIdm(current_fs.vec_s[0],current_state.velocity,dt,sim_param,&velocity);
    }

    //3. 用自行车模型获取期望状态
    CalculateDesiredState(current_state,steer,velocity,wheelbase_len,dt,sim_param,desired_state);
    return kSuccess;
}

static ErrorType PropagateOnceAdvancedLC(const StateTransformer& stf_current,const StateTransformer& stf_target,const Vehicle& ego_vehicle,const Vehicle &current_leading_vehicle,const Vehicle& gap_front_vehicle,const Vehicle& gap_rear_vehicle,const decimal_t& lat_track_offset,const decimal_t &dt,const Param& param,State* desired_state){
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    decimal_t steer,velocity;

    //1. 计算转角
    bool steer_calculation_failed = false;
    common::FrenetState ego_on_tarlane_fs;

    if(stf_target.GetFrenentStateFromState(current_state,&ego_on_tarlane_fs) != kSuccess || ego_on_tarlane_fs.vec_s[1]< -kEPS){
        steer_calculation_failed = true;
    }

    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf_target, current_state, ego_on_tarlane_fs,
                          wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    //2. 计算车速
    common::State target_state;
    //获取自车的目标状态
    if(kSuccess != GetTargetStateOnTargetLane(stf_target,ego_vehicle,gap_front_vehicle,gap_rear_vehicle,sim_param,&target_state)){
        target_state = current_state;
    }
    //获取目标状态在frenet下的状态
    common::FrenetState target_on_curlane_fs;
    if(stf_current.GetFrenetStateFromState(target_state,&target_on_curlane_fs) != kSuccess){ 
    }
    //获取自车当前状态在frenet下的状态
    common::FrenetState ego_on_curlane_fs;
    if(stf_current.GetFrenetStateFromState(current_state,&ego_on_curlane_fs)!= kSuccess){}

    simulator::ContextIntelligentDriveModel::CtxParam ctx_param(0.4,0.8);

    common::FrenetState current_leading_fs;
    //如果前车非法或者获取frenet状态失败
    if(current_leading_vehicle.id() == kInvalidAgentId || stf_current.GetFrenetStateFromState(current_leading_vehicle.state(),&current_leading_fs) != kSuccess){
        //without leading vehicle
        CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    }else{
        //有前车情况下
        decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf_current, ego_vehicle,
                                    current_leading_vehicle, current_leading_fs,
                                    &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          current_leading_fs.vec_s[0], current_leading_vehicle.state().velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    }
    //计算期望状态
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,sim_param, desired_state);
    return kSuccess;

}
```