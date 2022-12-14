
#pragma once

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
  
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /* main planning interface */
    bool reboundReplanForLoad(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
        const double trajectory_start_time, const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, 
        const bool flag_polyInit, 
        const bool have_local_traj);

    bool ReboundReplanForCable_try1(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations);
    bool ReboundReplanForCable_try2(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations, bool have_local,  double trajectory_start_time);

    bool computeInitReferenceState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, 
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &initMJO,
        const bool flag_polyInit);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, 
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints, 
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel);
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    void deliverTrajToOptimizer(void) { ploy_traj_opt_->setSwarmTrajs(&
    traj_.swarm_traj); };

    void setDroneIdtoOpt(void) { ploy_traj_opt_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return ploy_traj_opt_->getSwarmClearance(); }

    void fakeSwarmTrajs(void);

    PlanParameters pp_;
    // LocalTrajData local_data_;
    // GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    // SwarmTrajData swarm_trajs_;
    TrajContainer traj_, traj_1, traj_2, traj_3, traj_4, traj_cable_1, traj_cable_2;
    TrajContainer traj_cable_coef_;
    
    cable_load::Ptr cable_load_;

    int n_coef_iterate_;

    // ros::Publisher obj_pub_; //zx-todo

    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    bool start_flag_, reach_flag_;
    ros::Time global_start_time_;
    double start_time_, reach_time_, average_plan_time_;

    bool have_cable_local_tra = false;

    std::vector<Eigen::VectorXd> getResults();


  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    int continous_failures_count_{0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner