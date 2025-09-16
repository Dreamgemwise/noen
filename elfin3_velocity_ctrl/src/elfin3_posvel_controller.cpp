#include <controller_interface/controller.h>
#include <hardware_interface/posvel_joint_interface.h>   // ① 头文件换成 PosVel
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>

namespace elfin_ctrl
{
class JointGroupPosVelCtrl : public controller_interface::Controller<
                                hardware_interface::PosVelJointInterface>  // ② 模板接口
{
  std::vector<hardware_interface::PosVelJointHandle> joints_;
  double cmd_vel_[6]{};        // 速度指令
  double cmd_pos_[6]{};        // 要发的位置
  ros::Subscriber sub_;
  ros::Duration  cmd_timeout_{1.0};
  ros::Time      last_cmd_time_;

  bool init(hardware_interface::PosVelJointInterface* hw,
            ros::NodeHandle& nh) override
  {
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
    {
      ROS_ERROR("Need exactly 6 joint names");
      return false;
    }
    for (auto& n : joint_names)
      joints_.push_back(hw->getHandle(n));      // ③ 句柄类型变了

    sub_ = nh.subscribe("command", 1, &JointGroupPosVelCtrl::cmdCb, this);
    last_cmd_time_ = ros::Time::now();
    return true;
  }

  void starting(const ros::Time&) override
  {
    for (size_t i = 0; i < 6; ++i)
      cmd_pos_[i] = joints_[i].getPosition();   // 当前姿态作为起点

    cmd_vel_[0] = 0.5;                          // 固定关节1 0.5 rad/s
    std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);
  }

  void update(const ros::Time&, const ros::Duration& period) override
  {
    if (ros::Time::now() - last_cmd_time_ > cmd_timeout_)
      std::fill(cmd_vel_ + 1, cmd_vel_ + 6, 0.0);   // 超时清零其余轴

    // ④ 速度→位置积分
    for (size_t i = 0; i < 6; ++i)
      cmd_pos_[i] += cmd_vel_[i] * period.toSec();

    // ⑤ 一次下发 pos ＋ vel
    for (size_t i = 0; i < 6; ++i)
      joints_[i].setCommand(cmd_pos_[i], cmd_vel_[i]);
  }

  void cmdCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->velocity.size() >= 6)
    {
      std::copy(msg->velocity.begin() + 1,
                msg->velocity.begin() + 6,
                cmd_vel_ + 1);
      last_cmd_time_ = ros::Time::now();
    }
  }
};
} // namespace elfin_ctrl

PLUGINLIB_EXPORT_CLASS(elfin_ctrl::JointGroupPosVelCtrl,
                       controller_interface::ControllerBase)
