#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <traj_utils/Assignment.h>
#include "poly_traj_utils.hpp"
#include "munkres_algorithm.hpp"
#include <fstream>
#include <cable_load/cable_load.h>

namespace ego_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    
    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {
  
  private:
    GridMap::Ptr grid_map_;

    cable_load::Ptr cable_load_;
    
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;

    // poly_traj::MinJerkOpt jerkOpt_Cable0;
    // poly_traj::MinJerkOpt jerkOpt_Cable1;

    poly_traj::MinJerkOpt jerkOpt_uav1, jerkOpt_uav2, jerkOpt_uav3, jerkOpt_uav4;
    poly_traj::MinJerkOpt cableOpt_1, cableOpt_2;

    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstrainPoints cps_;

    int drone_id_;
    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_, variable_num_1, variable_num_2;     // optimization variables
    int piece_num_, piece_num_1, piece_num_2;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm
    Eigen::VectorXd initT1, initT2;

    std::vector<Eigen::Vector3d> points_positions;
    Eigen::Matrix<double, 6, 1> FM_init;
    Eigen::Matrix3d Transformb2e;

    string result_fn_;
    fstream result_file_;

    double collision_check_time_end_ = 0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_swarm_;                       // swarm weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double weight_cable_length_;
    double weight_cable_colli_;
    double cable_tolerance_;
    double load_dist_;

    double obs_clearance_;                   // safe distance between uav and obstacles
    double swarm_clearance_;                 // safe distance between uav and uav
    double max_vel_, max_acc_;               // dynamic limits
    
    int    formation_size_;
    bool   is_other_assigning_ = false;

    double t_now_;

    double cable_length_, load_mass_;
    double uav_obs_clearance_, uav_swarm_clearance_, cable_clearance_;
    double weight_uav_obs_, weight_uav_swarm_, weight_FM_feasibility_, weight_FM_norm_, weight_cable_collision_;


    Eigen::Vector3d load_position;


  public:

    PolyTrajOptimizer() {
         Transformb2e << 1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0;
    }
    ~PolyTrajOptimizer() {}

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map, const cable_load::Ptr &CLoad);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr1(void) { return &jerkOpt_uav1; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr2(void) { return &jerkOpt_uav2; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr3(void) { return &jerkOpt_uav3; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr4(void) { return &jerkOpt_uav4; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtrCable1(void) { return &cableOpt_1; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtrCable2(void) { return &cableOpt_2; }
    inline const poly_traj::MinJerkOpt *getcableOpt_1(void) { return &cableOpt_1; }
    inline const poly_traj::MinJerkOpt *getcableOpt_2(void) { return &cableOpt_2; }
    bool InitOptCable1(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time);
    bool InitOptCable2(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time); 
    
    bool getLastOptCable(Eigen::MatrixXd &coef1, Eigen::MatrixXd &coef2);
    bool setUAVPosition(double trajectory_start_time, bool have_local);
    
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }

    bool optCable1(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time);
    bool optCable2(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time);
    
    template <typename EIGENVEC>
    void addPVAGradCost2CT1(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);
    template <typename EIGENVEC>
    void addPVAGradCost2CT2(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);
    /* main planning API */
    bool OptimizeTrajectory_lbfgs_forLoad(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, double trajectory_start_time);
                  
    bool OptimizeTrajectory_lbfgs_forCable0(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations, std::vector< Eigen::Matrix<double,3,4> > &uav_positions_total);
    Eigen::Matrix<double, 3, 4> OptimizeTrajectory_lbfgs_forCable(Eigen::Vector3d acc, Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> &cable_coef);
    bool getInitCable(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations, Eigen::MatrixXd &cable_coefs);
                                        
    void astarWithMinTraj( const Eigen::MatrixXd &iniState, 
                           const Eigen::MatrixXd &finState,
                           std::vector<Eigen::Vector3d> &simple_path,
                           Eigen::MatrixXd &ctl_points,
                           poly_traj::MinJerkOpt &frontendMJ);

    std::vector<Eigen::VectorXd> results;
    // void setResults(Eigen::VectorXd result){results.push_back(result);}
  
  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback_forLoad(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionCallback_forCable(void *func_data, const double *x, double *grad, const int n);
    static double costFunction_Cable1(void *func_data, const double *x, double *grad, const int n);
    static double costFunction_Cable2(void *func_data, const double *x, double *grad, const int n);

    void addFeasibilityForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);
    void addCollisionForUAV(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);
    void addCollisionForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);

    void addSwarmForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);

    static int earlyExitCallback_forCable(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostP(const int i_dp,
                              const Eigen::Vector3d &p,
                              Eigen::Vector3d &gradp,
                              double &costp);            
    
    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);
    
    bool checkCollision(void);

  public:


    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace ego_planner
#endif