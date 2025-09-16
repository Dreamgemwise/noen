

/**
 * elfin3_vel_controller.cpp
 * 同名文件、同名类，插件 type 仍是 elfin_ctrl/JointGroupVelCtrl
 * 唯一变化：模板接口改为 PosVelJointInterface
 */
#include <controller_interface/controller.h>
// #include <hardware_interface/posvel_joint_interface.h>      // ← 换头文件
#include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/joint_command_interface.h>      // 通用 JointHandle 等
// #include <posvel_joint_interface/posvel_joint_interface.h>  // ★ elfin 自带 PosVel
#include <hardware_interface/posvel_command_interface.h> // :contentReference[oaicite:3]{index=3}

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>

namespace elfin_ctrl
{
  class JointGroupVelCtrl : public controller_interface::Controller<
                                hardware_interface::PosVelJointInterface> // ← 1) 接口改
  {
    std::vector<hardware_interface::PosVelJointHandle> joints_; // ← 2) 句柄类型
    double cmd_vel_[6]{};                                       // 每轴速度
    double cmd_pos_[6]{};                                       // 要发送的位置
    ros::Subscriber sub_;
    ros::Time last_cmd_time_;
    ros::Duration cmd_timeout_{1.0};

    bool init(hardware_interface::PosVelJointInterface *hw,
              ros::NodeHandle &nh) override
    {
      std::vector<std::string> joint_names;
      if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
      {
        ROS_ERROR("JointGroupVelCtrl: need 6 joint names");
        return false;
      }
      for (const auto &n : joint_names)
        joints_.push_back(hw->getHandle(n));

      sub_ = nh.subscribe("command", 1, &JointGroupVelCtrl::cmdCb, this);
      last_cmd_time_ = ros::Time::now();
      return true;
    }

    void starting(const ros::Time &) override
    {
      // 读当前真实关节角作初值
      for (size_t i = 0; i < 6; ++i)
        cmd_pos_[i] = joints_[i].getPosition();

      // 固定关节1 0.5 rad/s
      cmd_vel_[0] = 0.5;
      std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
    }

    void update(const ros::Time &, const ros::Duration &period) override
    {
      // 超时则清零其余 5 轴速度
      if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
        std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);

      // 速度→位置积分
      double dt = period.toSec();
      for (size_t i = 0; i < 6; ++i)
      {
        cmd_pos_[i] += cmd_vel_[i] * dt;
        joints_[i].setCommand(cmd_pos_[i], cmd_vel_[i]); // ← 3) 一次下发 (pos, vel)
      }
    }

    void cmdCb(const sensor_msgs::JointStateConstPtr &msg)
    {
      if (msg->velocity.size() >= 6)
      {
        std::copy(msg->velocity.begin() + 1,
                  msg->velocity.begin() + 6,
                  cmd_vel_ + 1); // 保留关节1固定速度
        last_cmd_time_ = ros::Time::now();
      }
    }
  };
} // namespace elfin_ctrl

PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupVelCtrl, // 名字不变
                       controller_interface::ControllerBase)












































// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <vector>
// #include <control_toolbox/pid.h>
// #include <math.h>

// namespace elfin_ctrl
// {
// class JointGroupTrqCtrl : public controller_interface::Controller<
//                               hardware_interface::EffortJointInterface>
// {
//   std::vector<hardware_interface::JointHandle> joints_;
//   double cmd_trq_[6]{};     // 每轴力矩
//   ros::Subscriber sub_;
//   ros::Time   last_cmd_time_;
//   ros::Time   start_time_;  // 控制器启动时间
//   ros::Duration cmd_timeout_{2.0}; // 命令超时时间
//   control_toolbox::Pid pid_controller_; // PID控制器

//   bool init(hardware_interface::EffortJointInterface* hw,
//             ros::NodeHandle& nh) override
//   {
//     std::vector<std::string> joint_names;
//     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
//     {
//       ROS_ERROR("JointGroupTrqCtrl: need 6 joint names");
//       return false;
//     }
//     for (const auto& n : joint_names)
//       joints_.push_back(hw->getHandle(n));

//     sub_ = nh.subscribe("command", 1, &JointGroupTrqCtrl::cmdCb, this);
//     last_cmd_time_ = ros::Time::now();

//     // 初始化PID控制器
//     if (!pid_controller_.init(ros::NodeHandle(nh, "joint1_pid"))) {
//       ROS_ERROR("Failed to initialize PID controller for joint 1");
//       return false;
//     }

//     // 动态调整PID参数
//     pid_controller_.setGains(1.0, 0.1, 0.01, 10.0, -10.0, false); // 示例参数，可根据实际情况调整

//     ROS_INFO("JointGroupTrqCtrl: Controller initialized - Joint 1 will move with 2*sin(t) Nm torque");

//     return true;
//   }

//   void starting(const ros::Time& time) override
//   {
//     start_time_ = time;  // 记录启动时间
//     // 固定关节1力矩，可根据实际情况调整
//     cmd_trq_[0] = 0.0;  // 初始力矩为0
//     std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//     pid_controller_.reset();
//   }

//   void update(const ros::Time& time, const ros::Duration& period) override
//   {
//     // 计算从启动开始的时间
//     double elapsed_time = (time - start_time_).toSec();

//     // 关节1使用2*sin(t) Nm的力矩
//     cmd_trq_[0] = 2.0 * sin(elapsed_time);

//     // 超时则清零其余 5 轴力矩
//     if (time - last_cmd_time_ > cmd_timeout_)
//     {
//       std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//       ROS_WARN("Command timeout, setting torques of joints 2-6 to zero.");
//     }

//     // 使用PID控制器计算关节1的力矩
//     double current_effort = joints_[0].getEffort();
//     double error = cmd_trq_[0] - current_effort;
//     double pid_output = pid_controller_.computeCommand(error, period);
//     joints_[0].setCommand(pid_output);

//     // 增加日志输出，方便调试
//     ROS_INFO("Joint 1: Target torque = %.2f Nm, Current torque = %.2f Nm, Error = %.2f, PID output = %.2f",
//              cmd_trq_[0], current_effort, error, pid_output);

//     // 设置其余关节的力矩
//     for (size_t i = 1; i < 6; ++i)
//     {
//       joints_[i].setCommand(cmd_trq_[i]);
//     }
//   }

//   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     if (msg->effort.size() >= 6)
//     {
//       std::copy(msg->effort.begin() + 1,
//                 msg->effort.begin() + 6,
//                 cmd_trq_ + 1);  // 保留关节1的正弦波力矩
//       last_cmd_time_ = ros::Time::now();
//       ROS_INFO("Received new torque command for joints 2-6.");
//     }
//     else
//     {
//       ROS_WARN("Received invalid torque command message. Expected at least 6 efforts.");
//     }
//   }
// };
// } // namespace elfin_ctrl

// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
//                        controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>

// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <vector>
// #include <control_toolbox/pid.h>

// namespace elfin_ctrl
// {
// class JointGroupTrqCtrl : public controller_interface::Controller<
//                               hardware_interface::EffortJointInterface>
// {
//   std::vector<hardware_interface::JointHandle> joints_;
//   double cmd_trq_[6]{};     // 每轴力矩
//   ros::Subscriber sub_;
//   ros::Time   last_cmd_time_;
//   ros::Duration cmd_timeout_{2.0}; // 适当延长命令超时时间
//   control_toolbox::Pid pid_controller_; // PID控制器

//   bool init(hardware_interface::EffortJointInterface* hw,
//             ros::NodeHandle& nh) override
//   {
//     std::vector<std::string> joint_names;
//     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
//     {
//       ROS_ERROR("JointGroupTrqCtrl: need 6 joint names");
//       return false;
//     }
//     for (const auto& n : joint_names)
//       joints_.push_back(hw->getHandle(n));

//     sub_ = nh.subscribe("command", 1, &JointGroupTrqCtrl::cmdCb, this);
//     last_cmd_time_ = ros::Time::now();

//     // 初始化PID控制器
//     if (!pid_controller_.init(ros::NodeHandle(nh, "joint1_pid"))) {
//       ROS_ERROR("Failed to initialize PID controller for joint 1");
//       return false;
//     }

//     // 动态调整PID参数，使用6个参数的setGains函数
//     pid_controller_.setGains(1.0, 0.1, 0.01, 10.0, -10.0, false); // 示例参数，可根据实际情况调整

//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     // 固定关节1力矩，可根据实际情况调整
//     cmd_trq_[0] = 2.0;
//     std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//     pid_controller_.reset();
//   }

//   void update(const ros::Time& time, const ros::Duration& period) override
//   {
//     // 超时则清零其余 5 轴力矩
//     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
//     {
//       std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//       ROS_WARN("Command timeout, setting torques of joints 2-6 to zero.");
//     }

//     // 使用PID控制器计算关节1的力矩
//     double current_effort = joints_[0].getEffort();
//     double error = cmd_trq_[0] - current_effort;
//     double pid_output = pid_controller_.computeCommand(error, period);
//     joints_[0].setCommand(pid_output);

//     // 增加日志输出，方便调试
//     ROS_INFO("Joint 1: Current effort = %.2f, Error = %.2f, PID output = %.2f", current_effort, error, pid_output);

//     // 设置其余关节的力矩
//     for (size_t i = 1; i < 6; ++i)
//     {
//       joints_[i].setCommand(cmd_trq_[i]);
//     }
//   }

//   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     if (msg->effort.size() >= 6)
//     {
//       std::copy(msg->effort.begin() + 1,
//                 msg->effort.begin() + 6,
//                 cmd_trq_ + 1);  // 保留关节1固定力矩
//       last_cmd_time_ = ros::Time::now();
//       ROS_INFO("Received new torque command for joints 2-6.");
//     }
//     else
//     {
//       ROS_WARN("Received invalid torque command message. Expected at least 6 efforts.");
//     }
//   }
// };
// } // namespace elfin_ctrl

// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
//                        controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>

// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <vector>
// #include <control_toolbox/pid.h>

// namespace elfin_ctrl
// {
// class JointGroupTrqCtrl : public controller_interface::Controller<
//                               hardware_interface::EffortJointInterface>
// {
//   std::vector<hardware_interface::JointHandle> joints_;
//   double cmd_trq_[6]{};     // 每轴力矩
//   ros::Subscriber sub_;
//   ros::Time   last_cmd_time_;
//   ros::Duration cmd_timeout_{2.0}; // 适当延长命令超时时间
//   control_toolbox::Pid pid_controller_; // PID控制器

//   bool init(hardware_interface::EffortJointInterface* hw,
//             ros::NodeHandle& nh) override
//   {
//     std::vector<std::string> joint_names;
//     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
//     {
//       ROS_ERROR("JointGroupTrqCtrl: need 6 joint names");
//       return false;
//     }
//     for (const auto& n : joint_names)
//       joints_.push_back(hw->getHandle(n));

//     sub_ = nh.subscribe("command", 1, &JointGroupTrqCtrl::cmdCb, this);
//     last_cmd_time_ = ros::Time::now();

//     // 初始化PID控制器
//     if (!pid_controller_.init(ros::NodeHandle(nh, "joint1_pid"))) {
//       ROS_ERROR("Failed to initialize PID controller for joint 1");
//       return false;
//     }

//     // 动态调整PID参数
//     pid_controller_.setGains(1.0, 0.1, 0.01); // 示例参数，可根据实际情况调整

//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     // 固定关节1力矩，可根据实际情况调整
//     cmd_trq_[0] = 1.0;
//     std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//     pid_controller_.reset();
//   }

//   void update(const ros::Time& time, const ros::Duration& period) override
//   {
//     // 超时则清零其余 5 轴力矩
//     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
//     {
//       std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//       ROS_WARN("Command timeout, setting torques of joints 2-6 to zero.");
//     }

//     // 使用PID控制器计算关节1的力矩
//     double current_effort = joints_[0].getEffort();
//     double error = cmd_trq_[0] - current_effort;
//     double pid_output = pid_controller_.computeCommand(error, period);
//     joints_[0].setCommand(pid_output);

//     // 增加日志输出，方便调试
//     ROS_INFO("Joint 1: Current effort = %.2f, Error = %.2f, PID output = %.2f", current_effort, error, pid_output);

//     // 设置其余关节的力矩
//     for (size_t i = 1; i < 6; ++i)
//     {
//       joints_[i].setCommand(cmd_trq_[i]);
//     }
//   }

//   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     if (msg->effort.size() >= 6)
//     {
//       std::copy(msg->effort.begin() + 1,
//                 msg->effort.begin() + 6,
//                 cmd_trq_ + 1);  // 保留关节1固定力矩
//       last_cmd_time_ = ros::Time::now();
//       ROS_INFO("Received new torque command for joints 2-6.");
//     }
//     else
//     {
//       ROS_WARN("Received invalid torque command message. Expected at least 6 efforts.");
//     }
//   }
// };
// } // namespace elfin_ctrl

// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
//                        controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>

// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <vector>
// #include <control_toolbox/pid.h>

// namespace elfin_ctrl
// {
// class JointGroupTrqCtrl : public controller_interface::Controller<
//                               hardware_interface::EffortJointInterface>
// {
//   std::vector<hardware_interface::JointHandle> joints_;
//   double cmd_trq_[6]{};     // 每轴力矩
//   ros::Subscriber sub_;
//   ros::Time   last_cmd_time_;
//   ros::Duration cmd_timeout_{1.0};
//   control_toolbox::Pid pid_controller_; // PID控制器

//   bool init(hardware_interface::EffortJointInterface* hw,
//             ros::NodeHandle& nh) override
//   {
//     std::vector<std::string> joint_names;
//     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
//     {
//       ROS_ERROR("JointGroupTrqCtrl: need 6 joint names");
//       return false;
//     }
//     for (const auto& n : joint_names)
//       joints_.push_back(hw->getHandle(n));

//     sub_ = nh.subscribe("command", 1, &JointGroupTrqCtrl::cmdCb, this);
//     last_cmd_time_ = ros::Time::now();

//     // 初始化PID控制器
//     if (!pid_controller_.init(ros::NodeHandle(nh, "joint1_pid"))) {
//       ROS_ERROR("Failed to initialize PID controller for joint 1");
//       return false;
//     }

//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     // 固定关节1力矩，可根据实际情况调整
//     cmd_trq_[0] = 1.0;
//     std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);
//     pid_controller_.reset();
//   }

//   void update(const ros::Time& time, const ros::Duration& period) override
//   {
//     // 超时则清零其余 5 轴力矩
//     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
//       std::fill(cmd_trq_ + 1, cmd_trq_ + 6, 0.0);

//     // 使用PID控制器计算关节1的力矩
//     double current_effort = joints_[0].getEffort();
//     double error = cmd_trq_[0] - current_effort;
//     double pid_output = pid_controller_.computeCommand(error, period);
//     joints_[0].setCommand(pid_output);

//     // 设置其余关节的力矩
//     for (size_t i = 1; i < 6; ++i)
//     {
//       joints_[i].setCommand(cmd_trq_[i]);
//     }
//   }

//   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     if (msg->effort.size() >= 6)
//     {
//       std::copy(msg->effort.begin() + 1,
//                 msg->effort.begin() + 6,
//                 cmd_trq_ + 1);  // 保留关节1固定力矩
//       last_cmd_time_ = ros::Time::now();
//     }
//   }
// };
// } // namespace elfin_ctrl

// // PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
// //                        controller_interface::ControllerBase)
// // PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
// //                        controller_interface::ControllerBase)
// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupTrqCtrl,
//                        controller_interface::ControllerBase)

// /**
//  * elfin3_vel_controller.cpp
//  * 同名文件、同名类，插件 type 仍是 elfin_ctrl/JointGroupVelCtrl
//  * 唯一变化：模板接口改为 PosVelJointInterface
//  */
// #include <controller_interface/controller.h>
// // #include <hardware_interface/posvel_joint_interface.h>      // ← 换头文件
// #include <hardware_interface/joint_command_interface.h>
// // #include <hardware_interface/joint_command_interface.h>      // 通用 JointHandle 等
// // #include <posvel_joint_interface/posvel_joint_interface.h>  // ★ elfin 自带 PosVel
// #include <hardware_interface/posvel_command_interface.h>   // :contentReference[oaicite:3]{index=3}

// #include <pluginlib/class_list_macros.hpp>
// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <vector>

// namespace elfin_ctrl
// {
// class JointGroupVelCtrl : public controller_interface::Controller<
//                               hardware_interface::PosVelJointInterface>   // ← 1) 接口改
// {
//   std::vector<hardware_interface::PosVelJointHandle> joints_;             // ← 2) 句柄类型
//   double cmd_vel_[6]{};     // 每轴速度
//   double cmd_pos_[6]{};     // 要发送的位置
//   ros::Subscriber sub_;
//   ros::Time   last_cmd_time_;
//   ros::Duration cmd_timeout_{1.0};

//   bool init(hardware_interface::PosVelJointInterface* hw,
//             ros::NodeHandle& nh) override
//   {
//     std::vector<std::string> joint_names;
//     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
//     {
//       ROS_ERROR("JointGroupVelCtrl: need 6 joint names");
//       return false;
//     }
//     for (const auto& n : joint_names)
//       joints_.push_back(hw->getHandle(n));

//     sub_ = nh.subscribe("command", 1, &JointGroupVelCtrl::cmdCb, this);
//     last_cmd_time_ = ros::Time::now();
//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     // 读当前真实关节角作初值
//     for (size_t i = 0; i < 6; ++i)
//       cmd_pos_[i] = joints_[i].getPosition();

//     // 固定关节1 0.5 rad/s
//     cmd_vel_[0] = 0.5;
//     std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
//   }

//   void update(const ros::Time&, const ros::Duration& period) override
//   {
//     // 超时则清零其余 5 轴速度
//     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
//       std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);

//     // 速度→位置积分
//     double dt = period.toSec();
//     for (size_t i = 0; i < 6; ++i)
//     {
//       cmd_pos_[i] += cmd_vel_[i] * dt;
//       joints_[i].setCommand(cmd_pos_[i], cmd_vel_[i]); // ← 3) 一次下发 (pos, vel)
//     }
//   }

//   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
//   {
//     if (msg->velocity.size() >= 6)
//     {
//       std::copy(msg->velocity.begin() + 1,
//                 msg->velocity.begin() + 6,
//                 cmd_vel_ + 1);  // 保留关节1固定速度
//       last_cmd_time_ = ros::Time::now();
//     }
//   }
// };
// } // namespace elfin_ctrl

// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupVelCtrl,  // 名字不变
//                        controller_interface::ControllerBase)

// // #include <controller_interface/controller.h>
// // #include <hardware_interface/posvel_joint_interface.h>   // ① 头文件换成 PosVel
// // #include <pluginlib/class_list_macros.hpp>
// // #include <sensor_msgs/JointState.h>
// // #include <ros/ros.h>
// // #include <vector>

// // namespace elfin_ctrl
// // {
// // class JointGroupPosVelCtrl : public controller_interface::Controller<
// //                                 hardware_interface::PosVelJointInterface>  // ② 模板接口
// // {
// //   std::vector<hardware_interface::PosVelJointHandle> joints_;
// //   double cmd_vel_[6]{};        // 速度指令
// //   double cmd_pos_[6]{};        // 要发的位置
// //   ros::Subscriber sub_;
// //   ros::Duration  cmd_timeout_{1.0};
// //   ros::Time      last_cmd_time_;

// //   bool init(hardware_interface::PosVelJointInterface* hw,
// //             ros::NodeHandle& nh) override
// //   {
// //     std::vector<std::string> joint_names;
// //     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
// //     {
// //       ROS_ERROR("Need exactly 6 joint names");
// //       return false;
// //     }
// //     for (auto& n : joint_names)
// //       joints_.push_back(hw->getHandle(n));      // ③ 句柄类型变了

// //     sub_ = nh.subscribe("command", 1, &JointGroupPosVelCtrl::cmdCb, this);
// //     last_cmd_time_ = ros::Time::now();
// //     return true;
// //   }

// //   void starting(const ros::Time&) override
// //   {
// //     for (size_t i = 0; i < 6; ++i)
// //       cmd_pos_[i] = joints_[i].getPosition();   // 当前姿态作为起点

// //     cmd_vel_[0] = 0.5;                          // 固定关节1 0.5 rad/s
// //     std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
// //   }

// //   void update(const ros::Time&, const ros::Duration& period) override
// //   {
// //     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
// //       std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);   // 超时清零其余轴

// //     // ④ 速度→位置积分
// //     for (size_t i = 0; i < 6; ++i)
// //       cmd_pos_[i] += cmd_vel_[i] * period.toSec();

// //     // ⑤ 一次下发 pos ＋ vel
// //     for (size_t i = 0; i < 6; ++i)
// //       joints_[i].setCommand(cmd_pos_[i], cmd_vel_[i]);
// //   }

// //   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
// //   {
// //     if (msg->velocity.size() >= 6)
// //     {
// //       std::copy(msg->velocity.begin() + 1,
// //                 msg->velocity.begin() + 6,
// //                 cmd_vel_ + 1);
// //       last_cmd_time_ = ros::Time::now();
// //     }
// //   }
// // };
// // } // namespace elfin_ctrl

// // PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupPosVelCtrl,
// //                        controller_interface::ControllerBase)

// // // #include <controller_interface/controller.h>
// // // #include <hardware_interface/joint_command_interface.h>
// // // #include <pluginlib/class_list_macros.hpp>
// // // #include <sensor_msgs/JointState.h>
// // // #include <ros/ros.h>
// // // #include <iostream>
// // // #include <vector>

// // // namespace elfin_ctrl
// // // {

// // //   class JointGroupVelCtrl : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
// // // // class JointGroupVelCtrl : public controller_interface::Controller<
// // // //         hardware_interface::PosVelJointInterface>

// // //   {
// // //     std::vector<hardware_interface::JointHandle> joints_;
// // //     double cmd_vel_[6]{};       // 存储关节速度命令
// // //     ros::Subscriber sub_;       // 速度命令订阅者
// // //     ros::Time last_cmd_time_;   // 上次接收命令的时间
// // //     ros::Duration cmd_timeout_; // 命令超时时间

// // //     bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &nh) override
// // //     {
// // //       std::vector<std::string> joint_names;

// // //       // 获取关节名称参数
// // //       if (!nh.getParam("joints", joint_names)) {
// // //         ROS_ERROR("JointGroupVelCtrl: Could not find 'joints' parameter");
// // //         return false;
// // //       }

// // //       // 获取关节句柄
// // //       for (auto &name : joint_names) {
// // //         try {
// // //           joints_.push_back(hw->getHandle(name));
// // //         } catch (const hardware_interface::HardwareInterfaceException& e) {
// // //           ROS_ERROR_STREAM("JointGroupVelCtrl: Failed to get joint handle for " << name << ": " << e.what());
// // //           return false;
// // //         }
// // //       }

// // //       // 检查关节数量
// // //       if (joints_.size() != 6) {
// // //         ROS_ERROR_STREAM("JointGroupVelCtrl: Expected 6 joints, got " << joints_.size());
// // //         return false;
// // //       }

// // //       // 初始化参数
// // //       cmd_timeout_ = ros::Duration(1.0);  // 命令超时时间

// // //       // 初始化订阅者和时间戳
// // //       sub_ = nh.subscribe("command", 1, &JointGroupVelCtrl::cmdCb, this);
// // //       last_cmd_time_ = ros::Time::now();

// // //       ROS_INFO("JointGroupVelCtrl: Controller initialized - Joint 1 will rotate at 0.5 rad/s");

// // //       return true;
// // //     }

// // //     void update(const ros::Time &time, const ros::Duration &period) override
// // //     {
// // //       // 固定设置关节1速度为0.5rad/s
// // //       cmd_vel_[0] = 0.5;

// // //       // 检查命令是否超时，超时则重置其他关节速度为0
// // //       if (time - last_cmd_time_ > cmd_timeout_) {
// // //         for (size_t i = 1; i < 6; ++i) {
// // //           cmd_vel_[i] = 0.0;
// // //         }
// // //         ROS_WARN_THROTTLE(1.0, "JointGroupVelCtrl: Command timeout, setting velocities to zero (except joint 1)");
// // //       }

// // //       // 设置关节速度命令
// // //       for (size_t i = 0; i < joints_.size(); ++i) {
// // //         joints_[i].setCommand(cmd_vel_[i]);
// // //         std::cout << "Joint " << i+1 << " velocity: " << cmd_vel_[i] << " rad/s\t";
// // //       }
// // //       std::cout << std::endl;
// // //     }

// // //     void starting(const ros::Time &time) override
// // //     {
// // //       last_cmd_time_ = time;
// // //       ROS_INFO("JointGroupVelCtrl: Controller started");

// // //       // 启动时初始化关节速度
// // //       cmd_vel_[0] = 0.5;  // 关节1固定速度
// // //       for (size_t i = 1; i < 6; ++i) {
// // //         cmd_vel_[i] = 0.0;
// // //       }
// // //     }

// // //     void stopping(const ros::Time &time) override
// // //     {
// // //       // 停止时将所有关节速度设为0
// // //       for (auto& joint : joints_) {
// // //         joint.setCommand(0.0);
// // //       }
// // //       ROS_INFO("JointGroupVelCtrl: Controller stopped, velocities set to zero");
// // //     }

// // //     void cmdCb(const sensor_msgs::JointStateConstPtr &msg)
// // //     {
// // //       // 检查消息长度
// // //       if (msg->velocity.size() >= 6) {
// // //         // 复制接收到的速度命令，但保留关节1的固定速度
// // //         std::copy(msg->velocity.begin() + 1, msg->velocity.begin() + 6, cmd_vel_ + 1);
// // //         last_cmd_time_ = ros::Time::now();
// // //         ROS_DEBUG("JointGroupVelCtrl: Received new velocity command (excluding joint 1)");
// // //       } else {
// // //         ROS_WARN_STREAM("JointGroupVelCtrl: Received " << msg->velocity.size()
// // //                       << " velocities, expected at least 6");
// // //       }
// // //     }
// // //   };
// // // }

// // // PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupVelCtrl, controller_interface::ControllerBase)

// // // // #include <controller_interface/controller.h>
// // // // #include <hardware_interface/joint_command_interface.h>   // 不变
// // // // #include <pluginlib/class_list_macros.hpp>
// // // // #include <sensor_msgs/JointState.h>
// // // // #include <ros/ros.h>
// // // // #include <vector>

// // // // namespace elfin_ctrl
// // // // {
// // // // class JointGroupPosCtrl : public controller_interface::Controller<
// // // //                              hardware_interface::PositionJointInterface>   // ① ⬅ 接口换了
// // // // {
// // // //   std::vector<hardware_interface::JointHandle> joints_;
// // // //   double cmd_vel_[6]{};              // 保留“速度指令”概念
// // // //   double cmd_pos_[6]{};              // 实际发送的位置
// // // //   ros::Subscriber sub_;
// // // //   ros::Duration cmd_timeout_{1.0};
// // // //   ros::Time last_cmd_time_;

// // // //   bool init(hardware_interface::PositionJointInterface* hw,
// // // //             ros::NodeHandle& nh) override
// // // //   {
// // // //     std::vector<std::string> joint_names;
// // // //     if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
// // // //     {
// // // //       ROS_ERROR("Need exactly 6 joint names");
// // // //       return false;
// // // //     }

// // // //     for (auto& n : joint_names)
// // // //       joints_.push_back(hw->getHandle(n));

// // // //     sub_ = nh.subscribe("command", 1, &JointGroupPosCtrl::cmdCb, this);
// // // //     last_cmd_time_ = ros::Time::now();
// // // //     return true;
// // // //   }

// // // //   void starting(const ros::Time& /*time*/) override
// // // //   {
// // // //     // 读取当前真实关节角作为起点
// // // //     for (size_t i = 0; i < 6; ++i)
// // // //       cmd_pos_[i] = joints_[i].getPosition();
// // // //     // 固定让关节1以 0.5 rad/s 正向转
// // // //     cmd_vel_[0] = 0.5;
// // // //     std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
// // // //   }

// // // //   void update(const ros::Time& /*time*/,
// // // //               const ros::Duration& period) override
// // // //   {
// // // //     // 若超时则清零其余 5 轴速度
// // // //     if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
// // // //       std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);

// // // //     // ② ⬅ 速度 → 位置（积分）
// // // //     for (size_t i = 0; i < 6; ++i)
// // // //       cmd_pos_[i] += cmd_vel_[i] * period.toSec();

// // // //     // 输出到硬件
// // // //     for (size_t i = 0; i < 6; ++i)
// // // //       joints_[i].setCommand(cmd_pos_[i]);
// // // //   }

// // // //   void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
// // // //   {
// // // //     if (msg->velocity.size() >= 6)
// // // //     {
// // // //       std::copy(msg->velocity.begin() + 1,
// // // //                 msg->velocity.begin() + 6,
// // // //                 cmd_vel_ + 1);              // 仍旧用 velocity topic
// // // //       last_cmd_time_ = ros::Time::now();
// // // //     }
// // // //   }
// // // // };
// // // // } // namespace elfin_ctrl

// // // // PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupPosCtrl,
// // // //                        controller_interface::ControllerBase)
