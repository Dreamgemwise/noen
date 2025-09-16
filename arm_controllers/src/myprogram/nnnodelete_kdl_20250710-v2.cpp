
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <cstdlib>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include <Eigen/Dense>
// #include <duration.h>
// #include <xmate_exception.h>
// #include <model.h>
// #include <robot.h>

// #include "ini.h"
// #include "rci_data/command_types.h"
// #include "rci_data/robot_datas.h"
// #include "print_rci.h"
// #include "move.h"
// #include "identify_model.h"

#include <vector>
#include <string>

// #ifndef PINOCCHIO_MODEL_DIR
// #define PINOCCHIO_MODEL_DIR "/home/robot/robot_model/xMatePro3_acc.urdf"
// #endif

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include "arm_controllers/ControllerJointState.h"
#include "arm_controllers/PassivityControllerParamsConfig.h"

#include <iostream>
#include <iomanip> // 引入setw和left所需的头文件

#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_buffer.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI

namespace arm_controllers
{

  class MyTorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
    class GainsHandler
    {
    public:
      struct Gains
      {
        Gains() : alpha_(0.0) {}
        Gains(double alpha) : alpha_(alpha) {}

        double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
      };

      GainsHandler() {}

      bool initDynamicReconfig(const ros::NodeHandle &node)
      {
        ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

        Gains gains;
        if (!node.getParam("alpha", gains.alpha_))
        {
          ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
          return false;
        }

        // Start dynamic reconfigure server
        typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
        param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
        dynamic_reconfig_initialized_ = true;

        setGains(gains);

        // Set Dynamic Reconfigure's gains to Pid's values
        updateDynamicReconfig();

        // Set callback
        param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
        param_reconfig_server_->setCallback(param_reconfig_callback_);

        return true;
      }

      void getGains(double &alpha)
      {
        Gains gains = *gains_buffer_.readFromRT();
        alpha = gains.alpha_;
      }

      Gains getGains()
      {
        return *gains_buffer_.readFromRT();
      }

      void setGains(double alpha)
      {
        Gains gains(alpha);

        setGains(gains);
      }

      void setGains(const Gains &gains)
      {
        gains_buffer_.writeFromNonRT(gains);

        updateDynamicReconfig(gains);
      }

      void updateDynamicReconfig()
      {
        // Make sure dynamic reconfigure is initialized
        if (!dynamic_reconfig_initialized_)
          return;

        // Get starting values
        PassivityControllerParamsConfig config;
        getGains(config.alpha);

        updateDynamicReconfig(config);
      }

      void updateDynamicReconfig(Gains gains)
      {
        // Make sure dynamic reconfigure is initialized
        if (!dynamic_reconfig_initialized_)
          return;

        PassivityControllerParamsConfig config;

        // Convert to dynamic reconfigure format
        config.alpha = gains.alpha_;

        updateDynamicReconfig(config);
      }

      void updateDynamicReconfig(PassivityControllerParamsConfig config)
      {
        // Make sure dynamic reconfigure is initialized
        if (!dynamic_reconfig_initialized_)
          return;

        // Set starting values, using a shared mutex with dynamic reconfig
        param_reconfig_mutex_.lock();
        param_reconfig_server_->updateConfig(config);
        param_reconfig_mutex_.unlock();
      }

      void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
      {
        ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

        // Set the gains
        setGains(config.alpha);
      }

    private:
      // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
      // blocking the realtime update loop
      realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

      // Dynamics reconfigure
      bool dynamic_reconfig_initialized_;
      typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
      boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
      DynamicReconfigServer::CallbackType param_reconfig_callback_;

      boost::recursive_mutex param_reconfig_mutex_;
    };

  public:
    ~MyTorqueController()
    {
      command_sub_.shutdown();
    }

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
      loop_count_ = 0;
      // List of controlled joints
      if (!n.getParam("joints", joint_names_))
      {
        ROS_ERROR("Could not find joint name");
        return false;
      }
      n_joints_ = joint_names_.size();

      if (n_joints_ == 0)
      {
        ROS_ERROR("List of joint names is empty.");
        return false;
      }

      // urdf
      urdf::Model urdf;
      if (!urdf.initParam("robot_description"))
      {
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      // joint handle
      for (int i = 0; i < n_joints_; i++)
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

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
          ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
          return false;
        }
        joint_urdfs_.push_back(joint_urdf);
      }

      Eigen::VectorXd q_m(model.nq), dq_m(model.nv), ddq_m(model.nv), tau_m(model.nq);
      q_m << 0.0, 0.0, q[2], M_PI / 2.0, M_PI / 2.0, q[5], 0.0;
      dq_m << 0.0, 0.0, dq[2], 0.0, 0.0, dq[5], 0.0;
      ddq_m << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      tau_m << 0.0, 0.0, tau[2], 0.0, 0.0, tau[5], 0.0;
      // kdl parser
      // if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
      // {
      //   ROS_ERROR("Failed to construct kdl tree");
      //   return false;
      // }

      // kdl chain
      // std::string root_name, tip_name;
      // if (!n.getParam("root_link", root_name))
      // {
      //   ROS_ERROR("Could not find root link name");
      //   return false;
      // }
      // if (!n.getParam("tip_link", tip_name))
      // {
      //   ROS_ERROR("Could not find tip link name");
      //   return false;
      // }
      // if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
      // {
      //   ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      //   ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
      //   ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
      //   ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
      //   ROS_ERROR_STREAM("  The segments are:");

      //   KDL::SegmentMap segment_map = kdl_tree_.getSegments();
      //   KDL::SegmentMap::iterator it;

      //   for (it = segment_map.begin(); it != segment_map.end(); it++)
      //     ROS_ERROR_STREAM("    " << (*it).first);

      //   return false;
      // }

      // gravity_ = KDL::Vector::Zero();
      // gravity_(2) = -9.81;
      // M_.resize(n_joints_);
      // C_.resize(n_joints_);
      // G_.resize(n_joints_);

      // inverse dynamics solver
      // id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

     // command and state
      // tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      // tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
      // q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      // qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      // qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      // q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
      // qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
      // qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
      // qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

      // q_.data = Eigen::VectorXd::Zero(n_joints_);
      // qdot_.data = Eigen::VectorXd::Zero(n_joints_);

      // q_error_.data = Eigen::VectorXd::Zero(n_joints_);
      // q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

      // gains
      if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
      {
        ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
        return false;
      }

      pids_.resize(n_joints_);
      for (size_t i = 0; i < n_joints_; i++)
      {
        if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
        {
          ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
          return false;
        }
      }

      // command subscriber
      commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
      command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &MyTorqueController::commandCB, this);
      ft_sub_ = n.subscribe<geometry_msgs::WrenchStamped>("ft_data_raw", 1, &MyTorqueController::ftCallback, this);

      // start realtime state publisher
      controller_state_pub_.reset(
          new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

      controller_state_pub_->msg_.header.stamp = ros::Time::now();
      for (size_t i = 0; i < n_joints_; i++)
      {
        controller_state_pub_->msg_.name.push_back(joint_names_[i]);
        controller_state_pub_->msg_.command.push_back(0.0);
        controller_state_pub_->msg_.command_dot.push_back(0.0);
        controller_state_pub_->msg_.state.push_back(0.0);
        controller_state_pub_->msg_.state_dot.push_back(0.0);
        controller_state_pub_->msg_.error.push_back(0.0);
        controller_state_pub_->msg_.error_dot.push_back(0.0);
        controller_state_pub_->msg_.effort_command.push_back(0.0);
        controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
        controller_state_pub_->msg_.effort_feedback.push_back(0.0);
      }
      ROS_INFO("EffortJointInterface registered successfully!");
      return true;
    }

    void starting(const ros::Time &time)
    {
      // get joint positions
      for (size_t i = 0; i < n_joints_; i++)
      {
        q_(i) = joints_[i].getPosition();
        qdot_(i) = joints_[i].getVelocity();

        // 记录初始位置作为目标位置
        q_cmd_(i) = q_(i);
      }

      ROS_INFO("Starting Torque Controller");
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
      if (msg->data.size() != n_joints_)
      {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
      }
      ROS_INFO("Received command message with %zu elements", msg->data.size()); // 添加日志输出
      commands_buffer_.writeFromNonRT(msg->data);
    }

    void ftCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
      ft_buffer_.writeFromNonRT(msg->wrench);
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {

      /* 读取最新 FT传感器数值 */
      geometry_msgs::Wrench wrench = *ft_buffer_.readFromRT();
      F = {wrench.force.x, wrench.force.y, wrench.force.z,
           wrench.torque.x, wrench.torque.y, wrench.torque.z};

      std::vector<double> &commands = *commands_buffer_.readFromRT();
      double dt = period.toSec();
      double q_cmd_old;

      // get joint states
      static double t = 0;
      // double angle = 45 * KDL::deg2rad;
      for (size_t i = 0; i < n_joints_; i++)
      {
        q_cmd_(i) = angle * sin(M_PI / 2 * t);
        // q_cmd_(i) = commands[i];

        enforceJointLimits(q_cmd_(i), i);
        q_(i) = joints_[i].getPosition();
        qdot_(i) = joints_[i].getVelocity();

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
          angles::shortest_angular_distance_with_limits(
              q_(i),
              q_cmd_(i),
              joint_urdfs_[i]->limits->lower,
              joint_urdfs_[i]->limits->upper,
              q_error_(i));
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
        }
        else // prismatic
        {
          q_error_(i) = q_cmd_(i) - q_(i);
        }

        qdot_cmd_(i) = angle * M_PI / 2 * cos(M_PI / 2 * t);              // (q_cmd_(i) - q_cmd_old_(i)) / period.toSec();;
        qddot_cmd_(i) = -angle * M_PI * M_PI / 2 / 2 * sin(M_PI / 2 * t); // (qdot_cmd_(i) - qdot_cmd_old_(i)) / period.toSec();

        q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

        q_cmd_old_(i) = q_cmd_(i);
        qdot_cmd_old_(i) = qdot_cmd_(i);

        // friction compensation, to do: implement friction observer
        // tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
      }

      t += dt;

      // compute dynamics term
      id_solver_->JntToMass(q_, M_);
      id_solver_->JntToCoriolis(q_, qdot_, C_);
      id_solver_->JntToGravity(q_, G_);

      // torque command
      GainsHandler::Gains gains = gains_handler_.getGains();
      qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
      // qddot_ref_.data = qddot_cmd_.data + gains.alpha_*q_error_dot_.data;
      qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

      tau_cmd_.data = M_.data * qddot_ref_.data + C_.data + G_.data + tau_fric_.data; // not exact passivity-based controller equation, since we cannot get coriolis matrix independently from KDL library.

      // 给其他轴施加虚拟墙。
      // 实现步骤：
      // 获取其他关节的目标关节角度q07x1
      // q_drag
      // 根据公式:tau1=-ke((q-q0)-delta),|q-q0|>delta计算虚拟墙的力tau1
      // 不同的轴分开：轴0到轴6
      if (fabs(q[0] - q_drag[0]) > delta)
      {
        tau1[0] = (-1) * be[0] * tanh(B * dq[0]) - ke[0] * tanh(K * ((q[0] - q_drag[0]) - delta));
      } // 轴0
      if (fabs(q[1] - q_drag[1]) > delta)
      {
        tau1[1] = (-1) * be[1] * tanh(B * dq[1]) - ke[1] * tanh(K * ((q[1] - q_drag[1]) - delta));
      } // 轴1
      if (fabs(q[2] - q_drag[2]) > delta)
      {
        tau1[2] = (-1) * be[2] * tanh(B * dq[2]) - ke[2] * tanh(K * ((q[2] - q_drag[2]) - delta));
      } // 轴2
      if (fabs(q[3] - q_drag[3]) > delta)
      {
        tau1[3] = (-1) * be[3] * tanh(B * dq[3]) - ke[3] * tanh(K * ((q[3] - q_drag[3]) - delta));
      } // 轴3
      if (fabs(q[4] - q_drag[4]) > delta)
      {
        tau1[4] = (-1) * be[4] * tanh(B * dq[4]) - ke[4] * tanh(K * ((q[4] - q_drag[4]) - delta));
      } // 轴4
      if (fabs(q[5] - q_drag[5]) > delta)
      {
        tau1[5] = (-1) * be[5] * tanh(B * dq[5]) - ke[5] * tanh(K * ((q[5] - q_drag[5]) - delta));
      } // 轴5

      tau[0] = tau1[0];
      tau[1] = tau1[1];
      tau[2] = tau1[2];
      tau[3] = tau1[3];
      tau[4] = tau1[4];
      tau[5] = tau1[5];

      tau[2] = C(0, 0) * dq[2] + C(0, 1) * dq[4] + M(0, 0) * ((-1.0) * k1(0) * Phi1(0) + g(0) + (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) * ddqd(0) - lambda2(0) * pow(fabs(e(0)), 1.0 / 3.0) * sign(e(0))) / (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) + M(0, 1) * ((-1.0) * k1(1) * Phi1(1) + g(1) + (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) * ddqd(1) - lambda2(1) * pow(fabs(e(1)), 1.0 / 3.0) * sign(e(1))) / (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) + sign(s(0)) * a_hat(0);
      tau[4] = C(1, 0) * dq[2] + C(1, 1) * dq[4] + M(1, 0) * ((-1.0) * k1(0) * Phi1(0) + g(0) + (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) * ddqd(0) - lambda2(0) * pow(fabs(e(0)), 1.0 / 3.0) * sign(e(0))) / (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) + M(1, 1) * ((-1.0) * k1(1) * Phi1(1) + g(1) + (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) * ddqd(1) - lambda2(1) * pow(fabs(e(1)), 1.0 / 3.0) * sign(e(1))) / (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) + sign(s(1)) * a_hat(1);

      for (int i = 0; i < n_joints_; i++)
      {
        controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);
        tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);

        // effort saturation
        if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
          tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

        if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
          tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

        joints_[i].setCommand(tau_cmd_(i));
      }

      // publish
      if (loop_count_ % 10 == 0)
      {
        if (controller_state_pub_->trylock())
        {
          controller_state_pub_->msg_.header.stamp = time;
          for (int i = 0; i < n_joints_; i++)
          {
            controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
            controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
            controller_state_pub_->msg_.state[i] = R2D * q_(i);
            controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
            controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
            controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
            controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
            controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
          }
          controller_state_pub_->unlockAndPublish();
        }
      }
    }

    void stopping(const ros::Time &time) {}

    void enforceJointLimits(double &command, unsigned int index)
    {
      // Check that this joint has applicable limits
      if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
      {
        if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
        {
          command = joint_urdfs_[index]->limits->upper;
        }
        else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
        {
          command = joint_urdfs_[index]->limits->lower;
        }
      }
    }

  private:
    int loop_count_;
    double actual_torque;

    // joint handles
    unsigned int n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // kdl
    // KDL::Tree kdl_tree_;
    // KDL::Chain kdl_chain_;
    // boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
    // KDL::JntSpaceInertiaMatrix M_;
    // KDL::JntArray C_;
    // KDL::JntArray G_; // gravity torque vector
    // KDL::Vector gravity_;

    // cmd, state
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
    realtime_tools::RealtimeBuffer<geometry_msgs::Wrench> ft_buffer_;
    // KDL::JntArray tau_cmd_, tau_fric_;
    // KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
    // KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
    // KDL::JntArray q_, qdot_;
    // KDL::JntArray q_error_, q_error_dot_;

    // gain
    GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
    std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

    // topic
    ros::Subscriber command_sub_;
    ros::Subscriber ft_sub_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            arm_controllers::ControllerJointState>>
        controller_state_pub_;

    double t = 0.0, T = 0.001, count = 0.0, delta = M_PI / 1800, K = 0.10, B = 0.20;
    double xr = 0.0, yr = 0.0, q0 = 0.0, a0 = 0.0, temp = 0.0, alpha = 0.1, F_dx = -10.0, F_dy = 5.0, F_d = 5.0;
    std::array<double, 16> pos;
    std::array<double, 7> q = {{0.0, 0.0, -0.9, 0.0, 0.0, 0.5, 0.0}};
    std::array<double, 7> dq = {{0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
    std::array<double, 7> ddq = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> dq_last = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> F = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> F_force = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> F1 = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> F2 = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> F_last = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> tau = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> tau1 = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> ke = {{2000.0, 2000.0, 0.0, 2000.0, 2000.0, 2000.0, 2000.0}};
    std::array<double, 7> be = {{100.0, 100.0, 0.0, 100.0, 100.0, 100.0, 100.0}};
    Eigen::Matrix<double, 2, 1> dqr, dqr_last;
    Eigen::Matrix<double, 2, 1> ddqr;
    Eigen::Matrix<double, 2, 2> J, J_INV;
    Eigen::Matrix<double, 2, 1> F_Joint;
    Eigen::Matrix<double, 2, 1> Fr;
    Eigen::Matrix<double, 2, 1> Fr_Joint;
    Eigen::Matrix<double, 2, 1> dqd;
    Eigen::Matrix<double, 2, 2> Md_INV; // 惯性系数的逆
    Eigen::Matrix<double, 2, 1> ddqd, qd;
    qd << -0.8159,
        0.6772;
    Eigen::Matrix<double, 2, 1> e, de;
    Eigen::Matrix<double, 2, 2> M, M_INV; // 惯性矩阵
    Eigen::Matrix<double, 2, 2> C;        // 科氏矩阵
    Eigen::Matrix<double, 2, 1> dPart_of_Integral, Part_of_Integral;
    Part_of_Integral << 0.0,
        0.0;
    Eigen::Matrix<double, 2, 1> s;
    Eigen::Matrix<double, 2, 1> Phi1, Phi2;
    Eigen::Matrix<double, 2, 1> dk1, k1;
    k1 << 1.5,
        1.5;
    Eigen::Matrix<double, 2, 1> k2;
    Eigen::Matrix<double, 2, 1> dg, g;
    g << 0.0,
        0.0;
    Eigen::Matrix<double, 2, 1> a_hat, dot_a_hat;
    a_hat << 0.0,
        0.0;
    Eigen::Matrix<double, 2, 1> h; // 自适应抗扰项增益
    h << 25.0,
        25.0;

    // 1、定义变量并赋值
    double l1 = 0.366, l2 = 0.2503; // 机器人臂长;
    // 速度场参数
    double VD = 0.02 * M_PI; // 速度场 参考轨迹速度
    double xc = 0.4;         // 速度场参考轨迹圆心横坐标
    double yc = -0.3;        // 速度长参考轨迹圆心纵坐标
    double r0 = 0.1;         // 速度场参考轨迹圆半径
    double epsilon1 = 0.00075;
    double c0 = 250;
    double k0 = 0.35;
    // 阻抗参数
    Eigen::Matrix<double, 2, 2> Md; // 惯性系数
    Md << 100.0, 0.0,
        0.0, 100.0;
    Eigen::Matrix<double, 2, 1> Bd; // 阻尼系数，不要太小，越小对外界力的响应越敏感
    Bd << 2.0,
        2.0;
    // 虚拟墙参数
    Eigen::Matrix<double, 2, 1> Br; // 阻尼
    Br << 0.0,
        0.0;
    Eigen::Matrix<double, 2, 1> Kr; // 刚度
    Kr << 0.0,
        0.0;
    double Vw = 0.18; // 速度虚拟墙
    double Pw = 0.11; // 位置虚拟墙
    double b = 500.0; // 阻尼调节系数
    double k = 500.0; // 刚度调节系数

    // 滑膜参数
    Eigen::Matrix<double, 2, 1> lambda1; // 比例项系数
    lambda1 << 1.0,
        0.5;
    Eigen::Matrix<double, 2, 1> lambda2; // 积分项系数,不能太大，0.1左右，太大超调会增大
    lambda2 << 0.1,
        0.1;
    Eigen::Matrix<double, 2, 1> lambda3; // 比例项系数终端滑膜项
    lambda3 << 0.5,
        0.5;
    // 超扭曲参数
    Eigen::Matrix<double, 2, 1> km; // dk1切换边界
    km << 1.0,
        1.0;
    Eigen::Matrix<double, 2, 1> w1; // dk1增益1
    w1 << 5.0,
        5.0;
    Eigen::Matrix<double, 2, 1> gamma1; // dk1增益2
    gamma1 << 5.0,
        5.0;
    Eigen::Matrix<double, 2, 1> miu; // 边界层
    miu << 0.03,
        0.03;
    Eigen::Matrix<double, 2, 1> eta; // 当k1小于边界km时，按eta快速增长
    eta << 2.0,
        2.0;
    Eigen::Matrix<double, 2, 1> k3; // fi1的比例项增益
    k3 << 2.0,
        2.0;
    Eigen::Matrix<double, 2, 1> epsilon; // k2增益
    epsilon << 0.25,
        0.25;
    Eigen::Matrix<double, 2, 1> beta0; // k2大小调节项
    beta0 << 4.0,
        4.0;
    // 控制律限制
    double min_val = -15.0; // 下限
    double max_val = 15.0;  // 上限
  };

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::MyMyTorqueController, controller_interface::ControllerBase)
