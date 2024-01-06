类 EudmManager
定义的结构体：
1. enum class LaneChangeTriggerType{kStick,kActive}
2. 
```C
struct ReplanningContext{
    bool is_valid = false;
    decimal_t seq_start_time;
    std::vector<DcpAction> action_seq;
};

struct ActiveLaneChangeRequest{
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
}

struct LaneChangeProposal{
    bool vaild = false;
    decimal_t trigger_time = 0.0;
    decimal_t operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping; 
}

struct LaneChangeContext{
    bool completed = true;
    bool trigger_when_approriate = false;
    decimal_t trigger_time = 0.0;
    decimal_t desired_operation_time = 0.0;
    int ego_lane_id = 0;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
}
template <typename T> 
using vec_E = std::vector<T,Eigen::aligned_allocator<T>>;

struct Snapshot{
    bool vaild = false;
    int original_winner_id;
    int process_winner_id;
    State plan_state;
    vector<vector<DcpAction>> action_script;
    vector<bool> sim_res;
    vector<bool> risky_res;
    vector<string> sim_info;
    vector<decimal_t> final_cost;
    vector<vector<CostStructure>> progress_cost;
    vector<CostStructure> tail_cost;
    vec_E<vec_E<Vehicle>> forward_trajs;
    vector<vector<LateralBehavior>> forward_lat_behaviors;
    vector<vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int,vec_E<Vehicel>>> surrounds_trajs;
    common::Lane ref_lane;

    double plan_stamp = 0.0;
    double time_cost = 0.0;
}
```

类成员：
1. EudmPlanner bp_
2. EudmPlannerMapAdapter map_adapter_;
3. decimal_t work_rate{20.0}
4. int ego_lane_id_;
5. ReplanningContext context_;
7. Snapshot last_snapshot_;
8. planning::eudm::Task last_task_;
9. LaneChangeContext lc_context_;
10. LaneChangeProposal last_lc_proposal_;
11. std::vector<ActivateLaneChangeRequest> prelilminary_active_requests_;

成员函数：
EudmManager(){}
初始化函数：
void Init(const std::string& config_path,const decimal_t work_rate)
功能： 载入了google日志系统、对bp_进行了初始化

核心函数：
ErrorType Run(const decimal_t stamp,const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,const planning::eudm::Task& task)：
该函数主要分为四个阶段：1、准备阶段 2、 运行阶段 3、 总结阶段 4、重选择阶段
