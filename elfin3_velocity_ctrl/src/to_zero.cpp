#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>
#include <control_toolbox/pid.h>

namespace elfin_ctrl
{
class JointGroupPosCtrl : public controller_interface::Controller<
                              hardware_interface::PositionJointInterface>
{
  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<double> target_positions_;  // 目标位置（默认零点）
  ros::Subscriber sub_;
  ros::Time last_cmd_time_;
  ros::Duration cmd_timeout_{1.0};  // 命令超时时间
  std::vector<control_toolbox::Pid> pid_controllers_;  // 每个关节的PID控制器

  bool init(hardware_interface::PositionJointInterface* hw,
            ros::NodeHandle& nh) override
  {
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
    {
      ROS_ERROR("JointGroupPosCtrl: need 6 joint names");
      return false;
    }

    // 初始化关节句柄
    joints_.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      try
      {
        joints_[i] = hw->getHandle(joint_names[i]);
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Could not find joint " << joint_names[i] << ": " << e.what());
        return false;
      }
    }

    // 初始化目标位置为零点
    target_positions_.assign(joint_names.size(), 0.0);
    
    // 从参数服务器加载目标位置（如果有）
    std::vector<double> param_targets;
    if (nh.getParam("target_positions", param_targets) && param_targets.size() == joint_names.size())
    {
      target_positions_ = param_targets;
      ROS_INFO("Loaded target positions from parameter server");
    }
    else
    {
      ROS_INFO("Using default zero positions as targets");
    }

    // 订阅位置命令
    sub_ = nh.subscribe("command", 1, &JointGroupPosCtrl::cmdCb, this);
    last_cmd_time_ = ros::Time::now();

    // 初始化每个关节的PID控制器
    pid_controllers_.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      std::string pid_ns = "joint" + std::to_string(i+1) + "_pid";
      if (!pid_controllers_[i].init(ros::NodeHandle(nh, pid_ns)))
      {
        ROS_ERROR_STREAM("Failed to initialize PID controller for joint " << joint_names[i]);
        return false;
      }
    }

    return true;
  }

  void starting(const ros::Time& time) override
  {
    // 记录起始位置
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      pid_controllers_[i].reset();
    }
    ROS_INFO("JointGroupPosCtrl started. Moving to target positions.");
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    // 超时则保持当前位置
    bool timeout = (ros::Time::now() - last_cmd_time_ > cmd_timeout_);
    
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      double current_pos = joints_[i].getPosition();
      double error = timeout ? 0.0 : (target_positions_[i] - current_pos);
      
      // 使用PID控制器计算位置命令
      double command = pid_controllers_[i].computeCommand(error, period);
      joints_[i].setCommand(command);
    }
  }

  void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->position.size() >= joints_.size())
    {
      // 更新目标位置
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        target_positions_[i] = msg->position[i];
      }
      last_cmd_time_ = ros::Time::now();
      ROS_INFO("Received new target positions");
    }
    else
    {
      ROS_WARN_STREAM("Received command with insufficient positions. Expected " 
                     << joints_.size() << ", got " << msg->position.size());
    }
  }
};
} // namespace elfin_ctrl

PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupPosCtrl,
                       controller_interface::ControllerBase)
