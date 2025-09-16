

/**
 * to_init_pose.cpp  ──  Elfin InitPoseController (smooth position move)
 * ------------------------------------------------
 * ● 接口       : hardware_interface::PositionJointInterface
 * ● 可调参数   :
 *     joints        (string[6])   - 关节名（必填）
 *     target_deg    (double[6])   - 目标角度，单位 °，默认 {0,0,90,0,90,0}
 *     move_time     (double)      - 总运动时间 [s]；若缺省，则用 max_vel 推算
 *     max_vel_deg   (double[6])   - 各轴最大速度 [°/s]，默认 45
 *
 * 编译/加载同普通 controller plugin
 */

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <vector>
#include <string>
#include <cmath>

class InitPoseController
    : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  /* ----------------- init() ------------------------------------------------ */
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh) override
  {
    /* 1. joints ------------------------------------------------------------ */
    if (!nh.getParam("joints", joint_names_) || joint_names_.empty())
    {
      ROS_ERROR("InitPoseController: param 'joints' missing or empty.");
      return false;
    }
    n_joints_ = joint_names_.size();

    /* 2. joint handles ----------------------------------------------------- */
    try
    {
      for (const auto &name : joint_names_)
        joint_handles_.push_back(hw->getHandle(name));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM("InitPoseController: " << e.what());
      return false;
    }

    /* 3. target position (deg→rad) ---------------------------------------- */
    std::vector<double> target_deg;
    if (!nh.getParam("target_deg", target_deg) || target_deg.size() != n_joints_)
      target_deg = {13.63, 0, 90, 90, -40, 0}; // 默认
    for (double d : target_deg)
      target_pos_.push_back(deg2rad(d));

    /* 4. max velocity ------------------------------------------------------ */
    std::vector<double> max_vel_deg;
    if (!nh.getParam("max_vel_deg", max_vel_deg) || max_vel_deg.size() != n_joints_)
      max_vel_deg = std::vector<double>(n_joints_, 45.0); // 45 °/s
    for (double d : max_vel_deg)
      max_vel_.push_back(deg2rad(d));

    /* 5. move_time --------------------------------------------------------- */
    if (nh.getParam("move_time", move_time_) && move_time_ > 0.0)
    {
      // 用户直接给定
      ROS_INFO_STREAM("InitPoseController: move_time = " << move_time_ << " s (param)");
    }
    else
    {
      // 根据最大速度自动估算
      move_time_ = 0.0;
      for (size_t i = 0; i < n_joints_; ++i)
      {
        double dt = std::fabs(target_pos_[i] - /*placeholder*/ 0.0) / max_vel_[i]; // q_start 未知，此处先占位
        if (dt > move_time_)
          move_time_ = dt;
      }
      if (move_time_ < 1.0)
        move_time_ = 1.0; // 至少 1 s
      ROS_INFO_STREAM("InitPoseController: move_time auto-compute = " << move_time_ << " s");
    }

    return true;
  }

  /* ----------------- starting() ------------------------------------------ */
  void starting(const ros::Time &time) override
  {
    start_time_ = time;

    // 记录起始角度 & 重新估算 move_time_ 如有必要
    start_pos_.resize(n_joints_);
    double longest_dt = 0.0;
    for (size_t i = 0; i < n_joints_; ++i)
    {
      start_pos_[i] = joint_handles_[i].getPosition();
      double dt = std::fabs(target_pos_[i] - start_pos_[i]) / max_vel_[i];
      if (dt > longest_dt)
        longest_dt = dt;
    }
    if (!fixed_time_)
      move_time_ = std::max(move_time_, longest_dt);
    ROS_INFO_STREAM("InitPoseController: final move_time = " << move_time_ << " s");

    // 首周期命令当前位置，避免阶跃
    for (size_t i = 0; i < n_joints_; ++i)
      joint_handles_[i].setCommand(start_pos_[i]);
  }

  /* ----------------- update() -------------------------------------------- */
  void update(const ros::Time &time, const ros::Duration & /*period*/) override
  {
    double t = (time - start_time_).toSec();
    double s = (t >= move_time_) ? 1.0
                                 : 0.5 * (1.0 - std::cos(M_PI * t / move_time_)); // 0→1 平滑

    for (size_t i = 0; i < n_joints_; ++i)
    {
      double cmd = start_pos_[i] + s * (target_pos_[i] - start_pos_[i]);
      joint_handles_[i].setCommand(cmd);
    }
  }

  void stopping(const ros::Time &) override {} // 无特殊处理

private:
  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  /* ----------------- members -------------------------------------------- */
  unsigned int n_joints_{0};
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  std::vector<double> target_pos_; // rad
  std::vector<double> start_pos_;  // rad
  std::vector<double> max_vel_;    // rad/s

  ros::Time start_time_;
  double move_time_{10.0}; // s
  bool fixed_time_{false}; // true if user explicitly set move_time_
};

PLUGINLIB_EXPORT_CLASS(InitPoseController, controller_interface::ControllerBase)

// #include <ros/ros.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/position_joint_interface.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <arm_controllers/ControllerJointState.h>
// #include <realtime_tools/realtime_publisher.h>
// #include <urdf/model.h>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <control_toolbox/pid.h>
// #include <angles/angles.h>
// #include <dynamic_reconfigure/server.h>
// #include <arm_controllers/PassivityControllerParamsConfig.h>
// #include <boost/scoped_ptr.hpp>
// #include <vector>
// #include <string>
// #include <iostream>
// #include <iomanip>

// #include <ros/ros.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <vector>
// #include <string>
// #include <cmath>

// class InitPoseController
//     : public controller_interface::Controller<hardware_interface::PositionJointInterface>
// {
// public:
//   bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh) override
//   {
//     /* --- 1. 读取关节列表 --------------------------------------------------- */
//     if (!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR("InitPoseController: param 'joints' not found");
//       return false;
//     }
//     n_joints_ = joint_names_.size();
//     if (n_joints_ == 0)
//     {
//       ROS_ERROR("InitPoseController: List of joint names is empty.");
//       return false;
//     }

//     /* --- 2. 获取关节句柄 ---------------------------------------------------- */
//     for (const auto &name : joint_names_)
//     {
//       try
//       {
//         joint_handles_.push_back(hw->getHandle(name));
//       }
//       catch (const hardware_interface::HardwareInterfaceException &e)
//       {
//         ROS_ERROR_STREAM("InitPoseController: " << e.what());
//         return false;
//       }
//     }
//     /* --- 3. 读取 / 设置目标位姿 ------------------------------------------- */
//     // std::vector<double> targ_deg;
//     // if (nh.getParam("target_deg", targ_deg) && targ_deg.size() == n_joints_)
//     // {
//     //   for (double d : targ_deg)
//     //     target_pos_.push_back(deg2rad(d));
//     // }
//     // else
//     // {
//     //   target_pos_ = std::vector<double>(n_joints_, 0.0); // 默认全部为0
//     //   if (n_joints_ >= 3)
//     //   {
//     //     target_pos_[2] = M_PI_2;
//     //   }
//     //   if (n_joints_ >= 5)
//     //   {
//     //     target_pos_[4] = M_PI_2;
//     //   }
//     // }
//     target_pos_ = {{0.0, 0.0, M_PI_2, 0.0, M_PI_2, 0.0}};
//     ROS_INFO_STREAM("InitPoseController target (rad): ");
//     for (size_t i = 0; i < n_joints_; ++i)
//     {
//       if (i > 0)
//       {
//         ROS_INFO_STREAM(", ");
//       }
//       ROS_INFO_STREAM(target_pos_[i]);
//     }
//     ROS_INFO_STREAM("\n");
//     return true;
//   }

//   /* --- 4. 控制循环 --------------------------------------------------------- */
//   void starting(const ros::Time &) override
//   {
//     // // 刚切换进来时立刻发送一次目标
//     // for (size_t i = 0; i < n_joints_; ++i)
//     //   joint_handles_[i].setCommand(target_pos_[i]);
//   }

//   void update(const ros::Time &, const ros::Duration &) override
//   {
//     // 周期性保持同一目标位姿
//     for (size_t i = 0; i < n_joints_; ++i)
//       joint_handles_[i].setCommand(target_pos_[i]);
//   }

//   void stopping(const ros::Time &) override {} // 无需特殊处理

// private:
//   // static double deg2rad(double deg) { return deg * M_PI / 180.0; }

//   unsigned int n_joints_;
//   std::vector<std::string> joint_names_;
//   std::vector<hardware_interface::JointHandle> joint_handles_;
//   std::array<double, 6> target_pos_;
// };

// /**
//  * to_init_pose.cpp  ──  Elfin-3 位置控制器
//  * ------------------------------------------------
//  * ● 接口：hardware_interface::PositionJointInterface
//  * ● 参数：
//  *     joints      (string[6]) - 要控制的关节名
//  *     target_deg  (double[6]) - 目标角度，单位 °，可选；默认 {0,0,90,0,90,0}
//  *
//  * 编译：
//  *   add_library(to_init_pose_controller src/to_init_pose.cpp)
//  *   target_link_libraries(to_init_pose_controller ${catkin_LIBRARIES})
//  *   pluginlib_export_class<InitPoseController, controller_interface::ControllerBase>(to_init_pose_controller)
//  */

// #include <ros/ros.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <vector>
// #include <string>
// #include <cmath>

// class InitPoseController
//   : public controller_interface::Controller<hardware_interface::PositionJointInterface>
// {
// public:
//   bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override
//   {
//     /* --- 1. 读取关节列表 --------------------------------------------------- */
//     if (!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR("InitPoseController: param 'joints' not found");
//       return false;
//     }
//     if (joint_names_.size() != 6)
//     {
//       ROS_ERROR("InitPoseController: expect 6 joints, got %zu", joint_names_.size());
//       return false;
//     }

//     /* --- 2. 获取关节句柄 ---------------------------------------------------- */
//     for (const auto& name : joint_names_)
//     {
//       try
//       {
//         joint_handles_.push_back(hw->getHandle(name));
//       }
//       catch (const hardware_interface::HardwareInterfaceException& e)
//       {
//         ROS_ERROR_STREAM("InitPoseController: " << e.what());
//         return false;
//       }
//     }
//     /* --- 3. 读取 / 设置目标位姿 ------------------------------------------- */
//     std::vector<double> targ_deg;
//     if (nh.getParam("target_deg", targ_deg) && targ_deg.size() == 6)
//     {
//       for (double d : targ_deg)
//         target_pos_.push_back(deg2rad(d));
//     }
//     else
//     {
//       target_pos_ = {0.0, 0.0, M_PI_2, 0.0, M_PI_2, 0.0};  // 默认 {0,0,90,0,90,0}°
//     }

//     ROS_INFO_STREAM("InitPoseController target (rad): "
//                     << target_pos_[0] << ", " << target_pos_[1] << ", " << target_pos_[2]
//                     << ", " << target_pos_[3] << ", " << target_pos_[4] << ", " << target_pos_[5]);
//     return true;
//   }

//   /* --- 4. 控制循环 --------------------------------------------------------- */
//   void starting(const ros::Time&) override
//   {
//     // 刚切换进来时立刻发送一次目标
//     for (size_t i = 0; i < 6; ++i)
//       joint_handles_[i].setCommand(target_pos_[i]);
//   }

//   void update(const ros::Time&, const ros::Duration&) override
//   {
//     // 周期性保持同一目标位姿
//     for (size_t i = 0; i < 6; ++i)
//       joint_handles_[i].setCommand(target_pos_[i]);
//   }

//   void stopping(const ros::Time&) override {}  // 无需特殊处理

// private:
//   static double deg2rad(double deg) { return deg * M_PI / 180.0; }

//   std::vector<std::string>                       joint_names_;
//   std::vector<hardware_interface::JointHandle>   joint_handles_;
//   std::vector<double>                            target_pos_;
// };

// PLUGINLIB_EXPORT_CLASS(InitPoseController, controller_interface::ControllerBase)
