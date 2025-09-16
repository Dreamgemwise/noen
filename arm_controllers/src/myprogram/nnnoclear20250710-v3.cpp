

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

// from computed torque clik
#include <boost/lexical_cast.hpp>
//
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/triyi/catkin_ws/src/elfin_robot/elfin_description/urdf/elfin3.urdf"
#endif

// #define SaveDataMax 97
#define num_taskspace 6
#define A 0.1
#define b 2.5
#define f 1
#define t_set 1

#include <string>
#include <iostream>
//

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define JointMax 6
#define SaveDataMax 7

namespace arm_controllers
{

  class AdmittanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    ~AdmittanceController()
    {
      sub_q_cmd_.shutdown();
      sub_forcetorque_sensor_.shutdown();
    }

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
      // List of controlled joints
      if (!n.getParam("joints", joint_names_))
      {
        ROS_ERROR("Could not find joint name");
        return false;
      }
      n_joints = joint_names_.size();

      if (n_joints == 0)
      {
        ROS_ERROR("List of joint names is empty.");
        return false;
      }

      // joint handle
      for (int i = 0; i < n_joints; i++)
      {
        try
        {
          joints_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
          ROS_ERROR_STREAM("Exception thrown: " << e.what());
          return false;
        }
      }

      // 初始化变量
      q = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      dq = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      tau = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};




































      // urdf
      const std::string urdf_filename = PINOCCHIO_MODEL_DIR;
      pinocchio::Model model;
      pinocchio::urdf::buildModel(urdf_filename, model);
      pinocchio::Data data(model);
      Eigen::VectorXd q_m(model.nq), dq_m(model.nv), ddq_m(model.nv), tau_m(model.nq);

      q_m << 0.0, 0.0, q[2], 0.0, q[4], 0.0;
      dq_m << 0.0, 0.0, dq[2], 0.0, dq[4], 0.0;
      ddq_m << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      tau_m << 0.0, 0.0, tau[2], 0.0, tau[4], 0.0;




      sub_q_cmd_ = n.subscribe("command", 1, &AdmittanceController::commandCB, this);

      sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor", 1, &AdmittanceController::updateFTsensor, this);

 
    } // end of initialize


    void starting(const ros::Time &time)
    {
      // get joint positions
      for (size_t i = 0; i < n_joints; i++)
      {
        ROS_INFO("JOINT %d", (int)i);
        q_(i) = joints_[i].getPosition();
        q_init_(i) = q_(i);
        qdot_(i) = joints_[i].getVelocity();
      }

      t = 0.0;
      sum_t = 0.0;

      // ROS_INFO("Starting Adaptive Impedance Controller");
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
      if (msg->data.size() != n_joints)
      {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints << ")! Not executing!");
        return;
      }

      for (unsigned int i = 0; i < n_joints; i++)
        q_cmd_(i) = msg->data[i];
    }

    // void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
      geometry_msgs::Wrench f_meas = msg->wrench;

      f_cur_[0] = f_meas.force.x;
      f_cur_[1] = f_meas.force.y;
      f_cur_[2] = f_meas.force.z;
      f_cur_[3] = f_meas.torque.x;
      f_cur_[4] = f_meas.torque.y;
      f_cur_[5] = f_meas.torque.z;
    }


    void update(const ros::Time &time, const ros::Duration &period)
    {

      // 设置参考轨迹
      for (size_t i = 0; i < n_joints; i++)
      {
        joints_[i].setCommand(tau_cmd_(i));
      }
    }

    void stopping(const ros::Time &time) {}

  private:
    std::vector<std::string> joint_names_;
    unsigned int n_joints;
    std::vector<hardware_interface::JointHandle> joints_;

    double t, sum_t, l1, l2;
    std::array<double, 6> q_, dq_;
    std::array<double, 6> tau_;
    std::array<double, 6> f_cur_;

    Eigen::Matrix<double, 6, 1> q_cmd_;

    ros::Subscriber sub_q_cmd_,sub_forcetorque_sensor_;
  };
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdmittanceController, controller_interface::ControllerBase)
