using namespace std;
#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <iostream>


ros::Publisher pos_cmd_pub0, pos_cmd_pub1, pos_cmd_pub2, pos_cmd_pub3, pos_cmd_pub4;

quadrotor_msgs::PositionCommand cmd0, cmd1, cmd2, cmd3, cmd4;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_0 = false, receive_traj_1 = false, receive_traj_2 = false, receive_traj_3 = false, receive_traj_4 = false;
boost::shared_ptr<poly_traj::Trajectory> traj_0, traj_1, traj_2, traj_3, traj_4;
double traj_duration_0, traj_duration_1, traj_duration_2, traj_duration_3, traj_duration_4;
ros::Time start_time_0, start_time_1, start_time_2, start_time_3, start_time_4;
int traj_id_0, traj_id_1, traj_id_2, traj_id_3, traj_id_4;
ros::Time time_last0, time_last1, time_last2, time_last3, time_last4;

// yaw control
double last_yaw_0 = 0.0, last_yaw_1 = 0.0, last_yaw_2 = 0.0, last_yaw_3 = 0.0, last_yaw_4 = 0.0;
double last_yaw_dot_0 = 0.0, last_yaw_dot_1 = 0.0, last_yaw_dot_2 = 0.0, last_yaw_dot_3 = 0.0, last_yaw_dot_4 = 0.0;

double time_forward_;

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_0.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_0 = msg->start_time;
  traj_duration_0 = traj_0->getTotalDuration();
  traj_id_0 = msg->traj_id;

  time_last0 = ros::Time::now();
  receive_traj_0 = true;
}

void polyTrajCallback1(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_1.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_1 = msg->start_time;
  traj_duration_1 = traj_1->getTotalDuration();
  traj_id_1 = msg->traj_id;

  time_last1 = ros::Time::now();
  receive_traj_1 = true;
}

void polyTrajCallback2(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_2.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_2 = msg->start_time;
  traj_duration_2 = traj_2->getTotalDuration();
  traj_id_2 = msg->traj_id;

  time_last2 = ros::Time::now();
  receive_traj_2 = true;
}

void polyTrajCallback3(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_3.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_3 = msg->start_time;
  traj_duration_3 = traj_3->getTotalDuration();
  traj_id_3 = msg->traj_id;

  time_last3 = ros::Time::now();
  receive_traj_3 = true;
}

void polyTrajCallback4(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_4.reset(new poly_traj::Trajectory(dura, cMats));
  start_time_4 = msg->start_time;
  traj_duration_4 = traj_4->getTotalDuration();
  traj_id_4 = msg->traj_id;

  time_last4 = ros::Time::now();
  receive_traj_4 = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, boost::shared_ptr<poly_traj::Trajectory> traj_, double traj_duration_, double &last_yaw_, double &last_yaw_dot_)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void pubCmd(bool receive_traj_, quadrotor_msgs::PositionCommand cmd, ros::Publisher pos_cmd_pub, ros::Time &time_last, ros::Time start_time_, boost::shared_ptr<poly_traj::Trajectory> traj_, double traj_duration_, int traj_id_, double &last_yaw_, double &last_yaw_dot_, int index)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();


  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    /*** calculate yaw ***/
    // yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last, traj_, traj_duration_, last_yaw_, last_yaw_dot_);
    yaw_yawdot.first = 0;
    yaw_yawdot.second = 0;
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    // calculate yaw 
    // yaw_yawdot.first = last_yaw_;
    yaw_yawdot.first = 0;
    yaw_yawdot.second = 0;
  }
  else
  {
    // std::cout << "[Traj server]: invalid time." << std::endl;
  }
  time_last = time_now;
  
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  // cout << "t_cur " << t_cur << " pos " << pos.transpose() << " start_time " << start_time_ << " duration " << traj_duration_<< endl;
  
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~"); //发布的话题名为节点名/定义的话题名

  ros::Subscriber poly_traj_sub0 = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  pos_cmd_pub0 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Subscriber poly_traj_sub1 = nh.subscribe("planning/trajectory1", 10, polyTrajCallback1);
  pos_cmd_pub1 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd1", 50);

  ros::Subscriber poly_traj_sub2 = nh.subscribe("planning/trajectory2", 10, polyTrajCallback2);
  pos_cmd_pub2 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd2", 50);

  ros::Subscriber poly_traj_sub3 = nh.subscribe("planning/trajectory3", 10, polyTrajCallback3);
  pos_cmd_pub3 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd3", 50);

  ros::Subscriber poly_traj_sub4 = nh.subscribe("planning/trajectory4", 10, polyTrajCallback4);
  pos_cmd_pub4 = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd4", 50);
  /* control parameter */
  cmd0.kx[0] = pos_gain[0];
  cmd0.kx[1] = pos_gain[1];
  cmd0.kx[2] = pos_gain[2];

  cmd0.kv[0] = vel_gain[0];
  cmd0.kv[1] = vel_gain[1];
  cmd0.kv[2] = vel_gain[2];
  cmd1 = cmd0; cmd2 = cmd0; cmd3 = cmd0; cmd4 = cmd0;

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  ROS_WARN("[Traj server]: ready.");
  
  ros::Rate rate(100);
  bool status = ros::ok();
  while(status)
  {
    pubCmd(receive_traj_0, cmd0, pos_cmd_pub0, time_last0, start_time_0, traj_0, traj_duration_0, traj_id_0, last_yaw_0, last_yaw_dot_0, 0);
    pubCmd(receive_traj_1, cmd1, pos_cmd_pub1, time_last1, start_time_1, traj_1, traj_duration_1, traj_id_1, last_yaw_1, last_yaw_dot_1, 1);
    pubCmd(receive_traj_2, cmd2, pos_cmd_pub2, time_last2, start_time_2, traj_2, traj_duration_2, traj_id_2, last_yaw_2, last_yaw_dot_2, 2);
    pubCmd(receive_traj_3, cmd3, pos_cmd_pub3, time_last3, start_time_3, traj_3, traj_duration_3, traj_id_3, last_yaw_3, last_yaw_dot_3, 3);
    pubCmd(receive_traj_4, cmd4, pos_cmd_pub4, time_last4, start_time_4, traj_4, traj_duration_4, traj_id_4, last_yaw_4, last_yaw_dot_4, 4);
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
  return 0;
}