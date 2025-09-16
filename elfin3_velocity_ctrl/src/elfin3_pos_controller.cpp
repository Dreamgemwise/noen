// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <iostream>
// #include <vector>

// namespace elfin_ctrl
// {

//   class JointGroupVelCtrl : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
// // class JointGroupVelCtrl : public controller_interface::Controller<
// //         hardware_interface::PosVelJointInterface>

//   {
//     std::vector<hardware_interface::JointHandle> joints_;
//     double cmd_vel_[6]{};       // 存储关节速度命令
//     ros::Subscriber sub_;       // 速度命令订阅者
//     ros::Time last_cmd_time_;   // 上次接收命令的时间
//     ros::Duration cmd_timeout_; // 命令超时时间

//     bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &nh) override
//     {
//       std::vector<std::string> joint_names;
      
//       // 获取关节名称参数
//       if (!nh.getParam("joints", joint_names)) {
//         ROS_ERROR("JointGroupVelCtrl: Could not find 'joints' parameter");
//         return false;
//       }
      
//       // 获取关节句柄
//       for (auto &name : joint_names) {
//         try {
//           joints_.push_back(hw->getHandle(name));
//         } catch (const hardware_interface::HardwareInterfaceException& e) {
//           ROS_ERROR_STREAM("JointGroupVelCtrl: Failed to get joint handle for " << name << ": " << e.what());
//           return false;
//         }
//       }
      
//       // 检查关节数量
//       if (joints_.size() != 6) {
//         ROS_ERROR_STREAM("JointGroupVelCtrl: Expected 6 joints, got " << joints_.size());
//         return false;
//       }
      
//       // 初始化参数
//       cmd_timeout_ = ros::Duration(1.0);  // 命令超时时间
      
//       // 初始化订阅者和时间戳
//       sub_ = nh.subscribe("command", 1, &JointGroupVelCtrl::cmdCb, this);
//       last_cmd_time_ = ros::Time::now();
      
//       ROS_INFO("JointGroupVelCtrl: Controller initialized - Joint 1 will rotate at 0.5 rad/s");
      
//       return true;
//     }

//     void update(const ros::Time &time, const ros::Duration &period) override
//     {
//       // 固定设置关节1速度为0.5rad/s
//       cmd_vel_[0] = 0.5;
      
//       // 检查命令是否超时，超时则重置其他关节速度为0
//       if (time - last_cmd_time_ > cmd_timeout_) {
//         for (size_t i = 1; i < 6; ++i) {
//           cmd_vel_[i] = 0.0;
//         }
//         ROS_WARN_THROTTLE(1.0, "JointGroupVelCtrl: Command timeout, setting velocities to zero (except joint 1)");
//       }
      
//       // 设置关节速度命令
//       for (size_t i = 0; i < joints_.size(); ++i) {
//         joints_[i].setCommand(cmd_vel_[i]);
//         std::cout << "Joint " << i+1 << " velocity: " << cmd_vel_[i] << " rad/s\t";
//       }
//       std::cout << std::endl;
//     }

//     void starting(const ros::Time &time) override
//     {
//       last_cmd_time_ = time;
//       ROS_INFO("JointGroupVelCtrl: Controller started");
      
//       // 启动时初始化关节速度
//       cmd_vel_[0] = 0.5;  // 关节1固定速度
//       for (size_t i = 1; i < 6; ++i) {
//         cmd_vel_[i] = 0.0;
//       }
//     }

//     void stopping(const ros::Time &time) override
//     {
//       // 停止时将所有关节速度设为0
//       for (auto& joint : joints_) {
//         joint.setCommand(0.0);
//       }
//       ROS_INFO("JointGroupVelCtrl: Controller stopped, velocities set to zero");
//     }

//     void cmdCb(const sensor_msgs::JointStateConstPtr &msg)
//     {
//       // 检查消息长度
//       if (msg->velocity.size() >= 6) {
//         // 复制接收到的速度命令，但保留关节1的固定速度
//         std::copy(msg->velocity.begin() + 1, msg->velocity.begin() + 6, cmd_vel_ + 1);
//         last_cmd_time_ = ros::Time::now();
//         ROS_DEBUG("JointGroupVelCtrl: Received new velocity command (excluding joint 1)");
//       } else {
//         ROS_WARN_STREAM("JointGroupVelCtrl: Received " << msg->velocity.size() 
//                       << " velocities, expected at least 6");
//       }
//     }
//   };
// }

// PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupVelCtrl, controller_interface::ControllerBase)















// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <sensor_msgs/JointState.h>
// #include <ros/ros.h>
// #include <iostream>
// #include <vector>




#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>   // 不变
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

namespace elfin_ctrl
{
class JointGroupPosCtrl : public controller_interface::Controller<
                             hardware_interface::PositionJointInterface>   // ① ⬅ 接口换了
{
  std::vector<hardware_interface::JointHandle> joints_;
  double cmd_vel_[6]{};              // 保留“速度指令”概念
  double cmd_pos_[6]{};              // 实际发送的位置
  ros::Subscriber sub_;
  ros::Duration cmd_timeout_{1.0};
  ros::Time last_cmd_time_;

  bool init(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& nh) override
  {
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
    {
      ROS_ERROR("Need exactly 6 joint names");
      return false;
    }

    for (auto& n : joint_names)
      joints_.push_back(hw->getHandle(n));

    sub_ = nh.subscribe("command", 1, &JointGroupPosCtrl::cmdCb, this);
    last_cmd_time_ = ros::Time::now();
    return true;
  }

  void starting(const ros::Time& /*time*/) override
  {
    // 读取当前真实关节角作为起点
    for (size_t i = 0; i < 6; ++i)
      cmd_pos_[i] = joints_[i].getPosition();
    // 固定让关节1以 0.5 rad/s 正向转
    cmd_vel_[0] = 0.5;
    std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
  }

  void update(const ros::Time& /*time*/,
              const ros::Duration& period) override
  {
    // 若超时则清零其余 5 轴速度
    if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
      std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);

    // ② ⬅ 速度 → 位置（积分）
    for (size_t i = 0; i < 6; ++i)
      cmd_pos_[i] += cmd_vel_[i] * period.toSec();

    // 输出到硬件
    for (size_t i = 0; i < 6; ++i)
      joints_[i].setCommand(cmd_pos_[i]);
  }

  void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->velocity.size() >= 6)
    {
      std::copy(msg->velocity.begin() + 1,
                msg->velocity.begin() + 6,
                cmd_vel_ + 1);              // 仍旧用 velocity topic
      last_cmd_time_ = ros::Time::now();
    }
  }
};
} // namespace elfin_ctrl

PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupPosCtrl,
                       controller_interface::ControllerBase)








