#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs_forLoad(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double trajectory_start_time)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = trajectory_start_time;
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);
    jerkOpt_.start_world_time = trajectory_start_time;

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1; //内部点坐标+时间坐标

    ros::Time t0 = ros::Time::now(), t1, t2;

    double q[variable_num_];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    Eigen::Map<Eigen::VectorXd> Vt(q + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_iterations = 60; // 200

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */

    t1 = ros::Time::now();

    int result = lbfgs::lbfgs_optimize(
        variable_num_,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback_forLoad,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);

    // test collision
    bool occ = false;
    occ = checkCollision();

    t2 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;

    printf("\033[32miter=%d, time(ms)=%5.3f, \n\033[0m", iter_num_, time_ms);
    // ROS_WARN("The optimization result is : %s", lbfgs::lbfgs_strerror(result));

    if (occ)
      return false;
    else
      return true;
  }

  bool PolyTrajOptimizer::InitOptCable1(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time)
  {
    int N = initT.size();
    cableOpt_1.reset(iniState, finState, N);
    cableOpt_1.generate(initInnerPts, initT);
    cableOpt_1.start_world_time = trajectory_start_time;
    return true;
  }

  bool PolyTrajOptimizer::InitOptCable2(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time)
  {
    int N = initT.size();
    cableOpt_2.reset(iniState, finState, N);
    cableOpt_2.generate(initInnerPts, initT);
    cableOpt_2.start_world_time = trajectory_start_time;
    return true;
  }

  bool PolyTrajOptimizer::optCable1(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }
    t_now_ = trajectory_start_time;
    cableOpt_1.start_world_time = trajectory_start_time;

    piece_num_1 = initT.size();
    initT1 =initT;

    cableOpt_1.reset(iniState, finState, piece_num_1);

    double final_cost;
    variable_num_1 = 3 * (piece_num_1 - 1); //内部点坐标 //t = initT

    ros::Time t0 = ros::Time::now();

    double q[variable_num_1];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_iterations = 60; // 200

    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */

    int result = lbfgs::lbfgs_optimize(
        variable_num_1,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunction_Cable1,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);
    
    return true;

  }

  double PolyTrajOptimizer::costFunction_Cable1(void *func_data, const double *x, double *grad, const int n)
  {
  
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_1 - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_1 - 1);

    Eigen::VectorXd gradT(opt->piece_num_1);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4);

    opt->cableOpt_1.generate(P, opt->initT1);

    opt->cableOpt_1.initGradCost(gradT, smoo_cost);
    opt->addPVAGradCost2CT1(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    Eigen::VectorXd result(6);
    result[0] = ros::Time::now().toSec();
    result[1] = smoo_cost;
    result.block<4,1>(2,0) = obs_swarm_feas_qvar_costs;
    opt->results.push_back(result);
    
    opt->cableOpt_1.getGrad2TP(gradT, gradP);

    return smoo_cost + obs_swarm_feas_qvar_costs.sum();
  }
  

  bool PolyTrajOptimizer::optCable2(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT, double trajectory_start_time)
  {
     if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = trajectory_start_time;
    cableOpt_2.start_world_time = trajectory_start_time;

    piece_num_2 = initT.size();
    initT2 =initT;

    cableOpt_2.reset(iniState, finState, piece_num_2);

    double final_cost;
    variable_num_2 = 3 * (piece_num_2 - 1); //内部点坐标 //t = initT

    ros::Time t0 = ros::Time::now();

    double q[variable_num_2];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_iterations = 60; // 200

    force_stop_type_ = DONT_STOP;

    /* ---------- optimize ---------- */
    int result = lbfgs::lbfgs_optimize(
        variable_num_2,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunction_Cable2,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);

    return true;
  }

  double PolyTrajOptimizer::costFunction_Cable2(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_2 - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_2 - 1);


    Eigen::VectorXd gradT(opt->piece_num_2);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4);

    opt->cableOpt_2.generate(P, opt->initT2);

    opt->cableOpt_2.initGradCost(gradT, smoo_cost);

    opt->addPVAGradCost2CT2(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost
    opt->cableOpt_2.getGrad2TP(gradT, gradP);

    fstream process_file_;
    process_file_.open(string("/home/tutu/process2.txt"), ios::app);
    process_file_ << smoo_cost << '\t' << obs_swarm_feas_qvar_costs.transpose() << '\t' << smoo_cost + obs_swarm_feas_qvar_costs.sum() << '\n';

    // cout << "Position2" << P << endl;
    // cout << "gradP2" << gradP << endl;

    return smoo_cost + obs_swarm_feas_qvar_costs.sum();
  }

  bool PolyTrajOptimizer::getInitCable(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations,  Eigen::MatrixXd &cable_coefs)
  {
    Eigen::Matrix<double, 6, 1> cable_coef;
    fstream process_file_;
    process_file_.open(string("/home/tutu/process.txt"), ios::app);

    const int N = accs.cols();
    cable_coefs.resize(6,N);
    for (size_t i = 0; i < N; i++)
    {
       OptimizeTrajectory_lbfgs_forCable(accs.col(i), positions.col(i), cable_coef);
       cable_coefs.col(i) = cable_coef;
    }
    return true;
  }

  bool PolyTrajOptimizer::getLastOptCable(Eigen::MatrixXd &coef1, Eigen::MatrixXd &coef2)
  {
    coef1 = cableOpt_1.getTraj().getPositions();
    coef2 = cableOpt_2.getTraj().getPositions();
    return true;
  }

  bool PolyTrajOptimizer::setUAVPosition(double trajectory_start_time, bool have_local)
  {
    Eigen::MatrixXd coef1, coef2;
    coef1 = cableOpt_1.getTraj().getPositions();
    coef2 = cableOpt_2.getTraj().getPositions();

    Eigen::MatrixXd coef;
    size_t N = coef1.cols();
    coef.resize(6,N);
    for (size_t i = 0; i < N; i++)
    {
      coef.col(i) << coef1.col(i), coef2.col(i);
    }

    fstream process_file_;
    process_file_.open(string("/home/tutu/coef.txt"), ios::app);
    process_file_ << coef << endl;

    Eigen::MatrixXd accs = jerkOpt_.getTraj().getAccs();
    Eigen::MatrixXd positions = jerkOpt_.getTraj().getPositions();
    Eigen::VectorXd durations = jerkOpt_.getTraj().getDurations();
    std::vector< Eigen::Matrix<double,3,4> > uav_positions_total;
    for (size_t i = 0; i < N; i++)
    {
      Eigen::Matrix<double, 6, 1> FM;
      Eigen::Vector3d gra_vector, acc_b;
      gra_vector << 0.0, 0.0, -9.8;
      acc_b << accs.coeff(0,i), -accs.coeff(1,i), -accs.coeff(2,i); 
      FM << load_mass_ * (acc_b + gra_vector), 0.0, 0.0, 0.0;
      Eigen::VectorXd FM_each = cable_load_->G_inv*FM + cable_load_->G_null_space*coef.col(i);
      Eigen::Matrix<double, 3, 4> uav_positions;
      for (size_t j = 0; j < 4; j++)
      {
        Eigen::Vector3d FMi = FM_each.block<3,1>(3*j,0);
        Eigen::Vector3d qi = FMi / FMi.norm();
        uav_positions.col(j) = positions.col(i) + Transformb2e * cable_length_ * qi;
      }

      fstream process_file_;
      process_file_.open(string("/home/tutu/uav.txt"), ios::app);
      process_file_ << FM_each.transpose() << '\t' << positions.col(i).transpose() << '\t' << uav_positions.col(0).transpose() << '\t' <<
      uav_positions.col(1).transpose() << '\t' << uav_positions.col(2).transpose() << '\t' << 
      uav_positions.col(3).transpose() << '\n';

      uav_positions_total.push_back(uav_positions);
    }
    
    for (size_t j = 0; j < 4; j++)
    {
      Eigen::Matrix3d uav_ini_pos, uav_fin_pos;
      Eigen::MatrixXd uav_iner_pos;
      uav_iner_pos.resize(3, N-2);

      if(!have_local)
      {
        uav_ini_pos << uav_positions_total[0].col(j), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
      }else{
        if(j==0){
          double t = trajectory_start_time - jerkOpt_uav1.start_world_time;
          uav_ini_pos << jerkOpt_uav1.getTraj().getPos(t), jerkOpt_uav1.getTraj().getVel(t), jerkOpt_uav1.getTraj().getAcc(t);
        }

        if(j==1){
          double t = trajectory_start_time - jerkOpt_uav2.start_world_time;
          uav_ini_pos << jerkOpt_uav2.getTraj().getPos(t), jerkOpt_uav2.getTraj().getVel(t), jerkOpt_uav2.getTraj().getAcc(t);
        }

        if(j==2)
        {
          double t = trajectory_start_time - jerkOpt_uav3.start_world_time;
          uav_ini_pos << jerkOpt_uav3.getTraj().getPos(t), jerkOpt_uav3.getTraj().getVel(t), jerkOpt_uav3.getTraj().getAcc(t);
        }

        if(j==3)
        {
          double t = trajectory_start_time - jerkOpt_uav4.start_world_time;
          uav_ini_pos << jerkOpt_uav4.getTraj().getPos(t), jerkOpt_uav4.getTraj().getVel(t), jerkOpt_uav4.getTraj().getAcc(t);
        }
      }
      
      uav_fin_pos << uav_positions_total[N-1].col(j), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
      for (size_t i = 1; i < N-1; i++)
      {
        uav_iner_pos.col(i-1) = uav_positions_total[i].col(j);
      }
      if(j==0){
        jerkOpt_uav1.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav1.generate(uav_iner_pos, durations);
        jerkOpt_uav1.start_world_time = trajectory_start_time;
      }
      if(j==1){
        jerkOpt_uav2.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav2.generate(uav_iner_pos, durations);
        jerkOpt_uav2.start_world_time = trajectory_start_time;
      }
      if(j==2){
        jerkOpt_uav3.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav3.generate(uav_iner_pos, durations);
        jerkOpt_uav3.start_world_time = trajectory_start_time;
      }
      if(j==3){
        jerkOpt_uav4.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav4.generate(uav_iner_pos, durations);
        jerkOpt_uav4.start_world_time = trajectory_start_time;
      }
    }
    
    return true;
  }


  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs_forCable0(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations, std::vector< Eigen::Matrix<double,3,4> > &uav_positions_total)
  { 
    Eigen::Matrix<double,3,4> uav_positions;
    Eigen::Matrix<double, 6, 1> cable_coef;
 
    fstream process_file_;
    process_file_.open(string("/home/tutu/process.txt"), ios::app);

    const int N = accs.cols();
    for (size_t i = 0; i < N; i++)
    {
       uav_positions = OptimizeTrajectory_lbfgs_forCable(accs.col(i), positions.col(i), cable_coef);
       uav_positions_total.push_back(uav_positions);
       process_file_ << i << setprecision(2) << uav_positions << "\n";
    }

    Eigen::Matrix3d uav_ini_pos, uav_fin_pos;
    Eigen::MatrixXd uav_iner_pos;
    uav_iner_pos.resize(3, N-2);
    
    for (size_t j = 0; j < 4; j++)
    {
      uav_ini_pos << uav_positions_total[0].col(j), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
      uav_fin_pos << uav_positions_total[N-1].col(j), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
      for (size_t i = 1; i < N-1; i++)
      {
        uav_iner_pos.col(i-1) = uav_positions_total[i].col(j);
      }
      if(j==0){
        jerkOpt_uav1.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav1.generate(uav_iner_pos, durations);
      }
      if(j==1){
        jerkOpt_uav2.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav2.generate(uav_iner_pos, durations);
      }
      if(j==2){
        jerkOpt_uav3.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav3.generate(uav_iner_pos, durations);
      }
      if(j==3){
        jerkOpt_uav4.reset(uav_ini_pos, uav_fin_pos, N-1);
        jerkOpt_uav4.generate(uav_iner_pos, durations);
      }
    }
    
    return true;
  }

  Eigen::Matrix<double, 3, 4> PolyTrajOptimizer::OptimizeTrajectory_lbfgs_forCable(Eigen::Vector3d acc, Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> &cable_coef)
  {
    //calculate the position of cable points
    load_position = position;

    points_positions.clear();
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d point_position = position + Transformb2e * cable_load_->cable_points[i];
      points_positions.push_back(point_position);
    }
    
    Eigen::Vector3d gra_vector, acc_b;
    gra_vector << 0.0, 0.0, -9.8;
    acc_b << acc[0], -acc[1], -acc[2];  //the angle between earth and body coordinates is [pi, 0, 0]
    FM_init << load_mass_ * (acc_b + gra_vector), 0.0, 0.0, 0.0;

    double final_cost;
    double q[6];
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_iterations = 60; // 200

    int result = lbfgs::lbfgs_optimize(
        6,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback_forCable,
        NULL,
        PolyTrajOptimizer::earlyExitCallback_forCable,
        this,
        &lbfgs_params);
    
    cable_coef << q[0], q[1], q[2], q[3], q[4], q[5];
    
    Eigen::VectorXd FM_each = cable_load_->G_inv*FM_init + cable_load_->G_null_space*cable_coef;

    Eigen::Matrix<double, 3, 4> uav_positions;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FM_each.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      uav_positions.col(i) = points_positions[i] + Transformb2e * cable_length_ * qi;
    }

    return uav_positions;

  }

  bool PolyTrajOptimizer::checkCollision(void)
  {
    /* check the safety of trajectory */
    double T_end;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    
    int N = traj.getPieceNum();
    int k = cps_num_prePiece_ * N + 1;
    int idx = k / 3 * 2; //取前面2/3
    int piece_of_idx = floor((idx - 1) / cps_num_prePiece_);
    Eigen::VectorXd durations = traj.getDurations();
    T_end = durations.head(piece_of_idx).sum() 
            + durations(piece_of_idx) * (idx - piece_of_idx * cps_num_prePiece_) / cps_num_prePiece_;

    bool occ = false;
    double dt = 0.01;
    int i_end = floor(T_end/dt);
    double t = 0.0;
    collision_check_time_end_ = T_end;

    for (int i=0; i<i_end; i++){
      Eigen::Vector3d pos = traj.getPos(t);
      if(grid_map_->getInflateOccupancy(pos) == 1){ //检查是否和地图中的障碍物碰撞
        occ = true;
        break;
      }
      t += dt;
    }
    return occ;
  }

  double PolyTrajOptimizer::costFunctionCallback_forCable(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    Eigen::Map<const Eigen::VectorXd> cable_coef(x, 6);
    Eigen::Map<Eigen::VectorXd> grad_cable_coef(grad, 6);

    Eigen::MatrixXd space = opt->cable_load_->G_null_space;
    Eigen::Matrix<double, 12, 6> G_inv = opt->cable_load_->G_inv;
    Eigen::Matrix<double, 6, 1> FM = opt->FM_init;

    Eigen::VectorXd FM_each = G_inv*FM + space*cable_coef;
    
    //feasibility
    Eigen::VectorXd cost_feasibility(13), grad_feasibility(6); 
    opt->addFeasibilityForCable(FM_each, grad_feasibility, cost_feasibility);
    //uav collision
    Eigen::VectorXd cost_collision(4), grad_collision(6);
    opt->addCollisionForUAV(FM_each, grad_collision, cost_collision);
    //swarm
    Eigen::VectorXd cost_swarm(6), grad_swarm(6);
    opt->addSwarmForCable(FM_each, grad_swarm, cost_swarm);
    // cable collision
    Eigen::VectorXd cost_collision_cable(4), grad_collision_cable(6);
    opt->addCollisionForCable(FM_each, grad_collision_cable, cost_collision_cable);

    grad_cable_coef = grad_feasibility + grad_collision + grad_swarm + grad_collision_cable;
    return cost_feasibility.sum() + cost_collision.sum() + cost_swarm.sum() + cost_collision_cable.sum();
  }

  int PolyTrajOptimizer::earlyExitCallback_forCable(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  void PolyTrajOptimizer::addCollisionForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost)
  {
    cost.setZero(); 
    grad.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;
    
    double dist, dist_err;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);

      size_t N = 20;
      double step = cable_length_ / N;
      double cost_temp;
      for (size_t j = 1; j < N; j++)
      {
        double dist_j = step * j;
        Eigen::Vector3d cable_point =  points_positions[i] + Transformb2e * dist_j * qi;
        grid_map_->evaluateEDT(cable_point, dist);
        dist_err = cable_clearance_ - dist;
        if(dist_err > 0)
        {
          cost_temp = weight_cable_collision_ * pow(dist_err, 3);
          Eigen::Vector3d dist_grad;
          grid_map_->evaluateFirstGrad(cable_point, dist_grad);
          grad += weight_cable_collision_ * 3 * pow(dist_err,2) * (-1) * Gi.transpose() * ((Eigen::Matrix3d::Identity()-qi*qi.transpose()) / FMi.norm()).transpose() * (dist_j*Transformb2e).transpose() * dist_grad;
          cost(i) += cost_temp; 
        }
      }
    }
  }


  void PolyTrajOptimizer::addSwarmForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost)
  {
    cost.setZero();
    grad.setZero();
    
    Eigen::MatrixXd space = cable_load_->G_null_space;
    
    int index = 0;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::Vector3d uav_position_i = points_positions[i] + Transformb2e * cable_length_ * qi;
      Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);
      for (size_t j = i+1; j < 4; j++)
      {
        Eigen::Vector3d FMj = FMeach.block<3,1>(3*j,0);
        Eigen::Vector3d qj = FMj / FMj.norm();
        Eigen::Vector3d uav_position_j = points_positions[j] + Transformb2e * cable_length_ * qj;
        Eigen::MatrixXd Gj = space.block<3,6>(3*j,0);
        
        Eigen::Vector3d dist_vector = uav_position_j - uav_position_i;
        double dist = dist_vector.norm();
        double dist_err = pow(uav_swarm_clearance_,2) - pow(dist,2);
        if (dist_err > 0)
        {
          cost(index) = weight_uav_swarm_ * pow(dist_err,3);
          
          grad += weight_uav_swarm_ * 3 * pow(dist_err, 2) * ( Gj.transpose() * ((Eigen::Matrix3d::Identity()-qj*qj.transpose()) / FMj.norm()).transpose() - Gi.transpose() * ((Eigen::Matrix3d::Identity()-qi*qi.transpose()) / FMi.norm()).transpose() ) * (cable_length_*Transformb2e).transpose() * (-2) * dist_vector;
        }
        index++;
      }
    }
  }

  void PolyTrajOptimizer::addCollisionForUAV(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost)
  {
    cost.setZero(); 
    grad.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;
    
    double dist, dist_err;
    for (size_t i = 0; i < 4; i++)
    {
      Eigen::Vector3d FMi = FMeach.block<3,1>(3*i,0);
      Eigen::Vector3d qi = FMi / FMi.norm();
      Eigen::Vector3d uav_position = points_positions[i] + Transformb2e * cable_length_ * qi;
      grid_map_->evaluateEDT(uav_position, dist);
      dist_err = uav_obs_clearance_ - dist;
      if(dist_err>0)
      {
        cost(i) = weight_uav_obs_ * pow(dist_err,3);
  
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateFirstGrad(uav_position, dist_grad);
        Eigen::MatrixXd Gi = space.block<3,6>(3*i,0);
        grad += weight_uav_obs_ * 3 * pow(dist_err,2) * (-1) * Gi.transpose() * ((Eigen::Matrix3d::Identity()-qi*qi.transpose()) / FMi.norm()).transpose() * (cable_length_*Transformb2e).transpose() * dist_grad;
      }
    }
  }

  void PolyTrajOptimizer::addFeasibilityForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost)
  {
    cost.setZero(); 
    grad.setZero();

    Eigen::MatrixXd space = cable_load_->G_null_space;

    double mg_first_six = -load_mass_ * 9.8 / 6.0;

    // FMeach(0) < 0, FMeach(1) < 0, FMeach(2) < 0
    if(FMeach(0) >= 0)
    {
      cost(0) = pow(FMeach(0),2);
      grad += 2 * FMeach(0) * space.row(0).transpose();
    }
    if(FMeach(1) >= 0)
    {
      cost(1) = pow(FMeach(1),2);
      grad += 2 * FMeach(1) * space.row(1).transpose();
    }
    if((FMeach(2) - mg_first_six) >= 0)
    {
      cost(2) = pow((FMeach(2) - mg_first_six),2);
      grad += 2 * (FMeach(2) - mg_first_six) * space.row(2).transpose();
    }

    // FMeach(3) < 0, FMeach(4) > 0, FMeach(5) < 0
    if(FMeach(3) >= 0)
    {
      cost(3) = pow(FMeach(3),2);
      grad += 2 * FMeach(3) * space.row(3).transpose();
    }
    if(FMeach(4) <= 0)
    {
      cost(4) = pow(FMeach(4),2);
      grad += 2 * FMeach(4) * space.row(4).transpose();
    }
    if((FMeach(5) - mg_first_six) >= 0)
    {
      cost(5) = pow((FMeach(5)-mg_first_six),2);
      grad += 2 * (FMeach(5) - mg_first_six) * space.row(5).transpose();
    }

    // FMeach(6) > 0, FMeach(7) > 0, FMeach(8) < 0
    if(FMeach(6) <= 0)
    {
      cost(6) = pow(FMeach(6),2);
      grad += 2 * FMeach(6) * space.row(6).transpose();
    }
    if(FMeach(7) <= 0)
    {
      cost(7) = pow(FMeach(7),2);
      grad += 2 * FMeach(7) * space.row(7).transpose();
    }
    if((FMeach(8) - mg_first_six) >= 0)
    {
      cost(8) = pow((FMeach(8) - mg_first_six),2);
      grad += 2 * (FMeach(8) - mg_first_six) * space.row(8).transpose();
    }

    // FMeach(9) > 0, FMeach(10) < 0, FMeach(11) < 0
    if(FMeach(9) <= 0)
    {
      cost(9) = pow(FMeach(9),2);
      grad += 2 * FMeach(9) * space.row(9).transpose();
    }
    if(FMeach(10) >= 0)
    {
      cost(10) = pow(FMeach(10),2);
      grad += 2 * FMeach(10) * space.row(10).transpose();
    }
    if((FMeach(11) - mg_first_six) >= 0)
    {
      cost(11) = pow((FMeach(11)-mg_first_six),2);
      grad += 2 * (FMeach(11)-mg_first_six) * space.row(11).transpose();
    }

    cost = cost * weight_FM_feasibility_;  //乘上系数
    grad = grad * weight_FM_feasibility_;

    // minimum sum of FMeach
    // double FM_norm1 = FMeach.block<3,1>(0,0);
    // double FM_norm2 = FMeach.block<3,1>(3,0);
    // double FM_norm3 = FMeach.block<3,1>(6,0);
    // double FM_norm4 = FMeach.block<3,1>(9,0);

    cost(12) = pow(FMeach.norm(),2) * weight_FM_norm_ ;
    grad += 2 * space.transpose() * FMeach * weight_FM_norm_;

  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback_forLoad(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(6);

    opt->jerkOpt_.generate(P, T);

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT1(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
     int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0; //global time

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = cableOpt_1.get_b().block<6, 3>(i * 6, 0);
      step = cableOpt_1.get_T1()(i) / K;
      s1 = 0.0; //local time
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        pos = c.transpose() * beta0;

        Eigen::Vector3d coef_last_three = cableOpt_2.getTraj().getPos(t_now_ + t - cableOpt_2.start_world_time);
        Eigen::Vector3d acc = jerkOpt_.getTraj().getAcc(t_now_ + t -jerkOpt_.start_world_time);
        Eigen::Vector3d load_position = jerkOpt_.getTraj().getPos(t_now_ + t -jerkOpt_.start_world_time);

        points_positions.clear();
        for (size_t i = 0; i < 4; i++)
        {
          Eigen::Vector3d point_position = load_position + Transformb2e * cable_load_->cable_points[i];
          points_positions.push_back(point_position);
        }
     
        Eigen::Matrix<double, 6, 1> FM0, coef;
        Eigen::Vector3d gra_vector, acc_b;
        gra_vector << 0.0, 0.0, -9.8;
        acc_b << acc[0], -acc[1], -acc[2];  //the angle between earth and body coordinates is [pi, 0, 0]
        FM0 << load_mass_ * (acc_b + gra_vector), 0.0, 0.0, 0.0;

        coef << pos, coef_last_three;
        // cout << "t_now_" << t_now_ << "t" << t << "start_time" << cableOpt_2.start_world_time << endl;
        // cout << "coef" << coef.transpose() << endl;
        // cout << "acc" << acc_b.transpose() << endl;

        Eigen::Matrix<double, 12, 1> FM_each = cable_load_->G_inv * FM0 + cable_load_->G_null_space * coef; // compute FM

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        Eigen::VectorXd grad_first_3_grad;
        // feasibility
        Eigen::VectorXd cost_feasibility(13), grad_feasibility(6); 
        addFeasibilityForCable(FM_each, grad_feasibility, cost_feasibility);
        grad_first_3_grad = grad_feasibility.block<3,1>(0,0);
        Eigen::Matrix<double, 6, 3> gradFeasibilityC = beta0 * grad_first_3_grad.transpose();
        cableOpt_1.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradFeasibilityC;
        costs(0) += omg * step * cost_feasibility.sum();

        // uav obstacle
        Eigen::VectorXd cost_collision(4), grad_collision(6);
        addCollisionForUAV(FM_each, grad_collision, cost_collision);
        grad_first_3_grad = grad_collision.block<3,1>(0,0);
        Eigen::Matrix<double, 6, 3> gradCollisionC = beta0 * grad_first_3_grad.transpose();
        cableOpt_1.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradCollisionC;
        costs(1) += omg * step * cost_collision.sum();

        //swarm
        Eigen::VectorXd cost_swarm(6), grad_swarm(6);
        addSwarmForCable(FM_each, grad_swarm, cost_swarm);
        grad_first_3_grad = grad_swarm.block<3,1>(0,0);
        Eigen::Matrix<double, 6, 3> gradSwarmC = beta0 * grad_first_3_grad.transpose();
        cableOpt_1.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradSwarmC;
        costs(2) += omg * step * cost_swarm.sum();

        // cable collision
        Eigen::VectorXd cost_collision_cable(4), grad_collision_cable(6);
        addCollisionForCable(FM_each, grad_collision_cable, cost_collision_cable);
        grad_first_3_grad = grad_collision_cable.block<3,1>(0,0);
        Eigen::Matrix<double, 6, 3> gradCollisionCableC = beta0 * grad_first_3_grad.transpose();
        cableOpt_1.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradCollisionCableC;
        costs(3) += omg * step * cost_collision_cable.sum();

        s1 += step;
        if(j!=K)
          t = t + step; 
      }
    }
  }


  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT2(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
     int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0; //global time

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = cableOpt_2.get_b().block<6, 3>(i * 6, 0);
      step = cableOpt_2.get_T1()(i) / K;
      s1 = 0.0; //local time
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        pos = c.transpose() * beta0;

        Eigen::Vector3d coef_first_three = cableOpt_1.getTraj().getPos(t_now_ + t - cableOpt_1.start_world_time);
        Eigen::Vector3d acc = jerkOpt_.getTraj().getAcc(t_now_ + t -jerkOpt_.start_world_time);

        Eigen::Vector3d load_position = jerkOpt_.getTraj().getPos(t_now_ + t -jerkOpt_.start_world_time);

        points_positions.clear();
        for (size_t i = 0; i < 4; i++)
        {
          Eigen::Vector3d point_position = load_position + Transformb2e * cable_load_->cable_points[i];
          points_positions.push_back(point_position);
        }
        
        Eigen::Matrix<double, 6, 1> FM0, coef;
        Eigen::Vector3d gra_vector, acc_b;
        gra_vector << 0.0, 0.0, -9.8;
        acc_b << acc[0], -acc[1], -acc[2];  //the angle between earth and body coordinates is [pi, 0, 0]
        FM0 << load_mass_ * (acc_b + gra_vector), 0.0, 0.0, 0.0;

        coef << coef_first_three, pos;
        Eigen::Matrix<double, 12, 1> FM_each = cable_load_->G_inv * FM0 + cable_load_->G_null_space * coef; // compute FM
        // cout << "t_now" << t_now_ << endl;
        // cout << "t" << t << endl;
        // cout << "start_world_time" << jerkOpt_.start_world_time << endl;
        // cout << "shijiancha" << t_now_ + t -jerkOpt_.start_world_time << endl;
        // cout << "jerkOpt_duration" << jerkOpt_.getTraj().getTotalDuration() << endl;
        // cout << "durations_jerkOpt" << jerkOpt_.get_T1() << endl;
        // cout << "Cable_jerkOpt" << cableOpt_1.get_T1() << endl;
        // cout << "acc" << acc_b << endl;
        // cout << "FM0" << FM0 << endl;
        // cout << "FM_each" << FM_each << endl;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        Eigen::VectorXd grad_first_3_grad;
        // feasibility
        Eigen::VectorXd cost_feasibility(13), grad_feasibility(6); 
        addFeasibilityForCable(FM_each, grad_feasibility, cost_feasibility);
        grad_first_3_grad = grad_feasibility.block<3,1>(3,0);
        Eigen::Matrix<double, 6, 3> gradFeasibilityC = beta0 * grad_first_3_grad.transpose();
        cableOpt_2.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradFeasibilityC;
        costs(0) += omg * step * cost_feasibility.sum();

        // uav obstacle
        Eigen::VectorXd cost_collision(4), grad_collision(6);
        addCollisionForUAV(FM_each, grad_collision, cost_collision);
        grad_first_3_grad = grad_collision.block<3,1>(3,0);
        Eigen::Matrix<double, 6, 3> gradCollisionC = beta0 * grad_first_3_grad.transpose();
        cableOpt_2.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradCollisionC;
        costs(1) += omg * step * cost_collision.sum();

        //swarm
        Eigen::VectorXd cost_swarm(6), grad_swarm(6);
        addSwarmForCable(FM_each, grad_swarm, cost_swarm);
        grad_first_3_grad = grad_swarm.block<3,1>(3,0);
        Eigen::Matrix<double, 6, 3> gradSwarmC = beta0 * grad_first_3_grad.transpose();
        cableOpt_2.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradSwarmC;
        costs(2) += omg * step * cost_swarm.sum();

        // cable collision
        Eigen::VectorXd cost_collision_cable(4), grad_collision_cable(6);
        addCollisionForCable(FM_each, grad_collision_cable, cost_collision_cable);
        grad_first_3_grad = grad_collision_cable.block<3,1>(3,0);
        Eigen::Matrix<double, 6, 3> gradCollisionCableC = beta0 * grad_first_3_grad.transpose();
        cableOpt_2.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradCollisionCableC;
        costs(3) += omg * step * cost_collision_cable.sum();

        s1 += step;
        if(j!=K)
          t = t + step; 
      }
    }
  }


  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {

    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0; //global time
    // Eigen::MatrixXd constrain_pts(3, N * K + 1);
  
    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0; //local time
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;
        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }
  
        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(4) += omg * step * costv;
        }
        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(4) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }
    costs(5) += var;

  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // use esdf
    double dist;
    grid_map_->evaluateEDT(p, dist);
    double dist_err = obs_clearance_ - dist;
    if (dist_err > 0)
    {
      ret = true;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateFirstGrad(p, dist_grad);

      costp = wei_obs_ * pow(dist_err, 3);
      gradp = -wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           vector<Eigen::Vector3d> &simple_path,
                                           Eigen::MatrixXd &ctl_points,
                                           poly_traj::MinJerkOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0); //初始位置
    Eigen::Vector3d end_pos = finState.col(0); //结束位置

    /* astar search and get the simple path*/
    simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos, load_dist_);

    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }

    frontendMJ.reset(iniState, finState, piece_num);

    /* generate init traj*/
    double des_vel = max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0;
    do
    {
      for (size_t i = 1; i <= piece_num; ++i)
      {
        time_vec(i - 1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                                   : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
      }
      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.5;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ && debug_num < 1);

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
  }
  
  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);

    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0); 
    nh.param("optimization/load_dist", load_dist_, 0.5);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/load_mass", load_mass_, 1.0); // 吊挂物质量

    nh.param("cable/cable_length", cable_length_, 2.0);
    nh.param("cable/uav_obs_clearance", uav_obs_clearance_, 2.0 ); //无人机距离障碍物的最小距离
    nh.param("cable/uav_swarm_clearance", uav_swarm_clearance_, 2.0); //无人机间的最小距离
    nh.param("cable/cable_clearance", cable_clearance_, 2.0); //无人机间的最小距离
    nh.param("cable/weight_uav_obs", weight_uav_obs_, -1.0);
    nh.param("cable/weight_uav_swarm", weight_uav_swarm_, -1.0);
    nh.param("cable/weight_FM_feasibility", weight_FM_feasibility_, -1.0);
    nh.param("cable/weight_FM_norm", weight_FM_norm_, -1.0);
    nh.param("cable/weight_cable_collision", weight_cable_collision_, -1.0);

    // set the formation type
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map, const cable_load::Ptr &CLoad)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(800, 200, 40));

    cable_load_ = CLoad;
    // Eigen::MatrixXd space = cable_load_->G_null_space;
    // space.resize(12,6);
    // Eigen::Matrix<double, 12, 6> G_inv = cable_load_->G_inv;
    
    // Eigen::Matrix<double, 6, 1> grav_vector;
    // grav_vector << 0.0, 0.0, -9.8, 0.0, 0.0, 0.0;
    // double weight = load_mass_ * 9.8, alpha = M_PI/6, beta = M_PI/4;
    // Eigen::Matrix<double, 12, 1> FM_each;
    // double temp1 = weight/4.0 * tan(alpha)*cos(beta);
    // double temp2 = weight/4.0 * tan(alpha)*sin(beta);
    // double temp3 = weight/4.0;
    // FM_each << -temp1, -temp2, -temp3, -temp1, temp2, -temp3, temp1, temp2, -temp3, temp1, -temp2, -temp3; 
    // FM_each = FM_each - G_inv * load_mass_ * grav_vector;

    // cout << space << endl;
    // cout << FM_each << endl;
    
    // cout << space * (space.transpose() * space).ldlt().solve(space.transpose() * FM_each) << endl; 
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::setDroneId(const int drone_id)
  {
    drone_id_ = drone_id;}

}