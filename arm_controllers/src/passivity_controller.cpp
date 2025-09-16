

// passivity_controller.cpp
// =============================================================
// PassivityTorqueController — publish *pure Nm* effort commands.
// -------------------------------------------------------------
// ● The Elfin hardware_interface already converts Nm → cnt using
//   axis_torque_factors, so *do not* multiply inside the controller.
// ● Subscribes   /elfin_effort_controller/command_nm
//     std_msgs/Float64MultiArray, length == 6, units = **Nm**.
// ● Joints list comes from the controller YAML (`joints:`) or, if
//   absent, parameter ~/joints.
// ● Works at any update_rate; recommend 500 Hz in YAML.
// -------------------------------------------------------------

// #include <ros/ros.h>
// #include <controller_interface/multi_interface_controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <realtime_tools/realtime_buffer.h>
// #include <std_msgs/Float64MultiArray.h>

// namespace passivity_controller
// {
// class PassivityTorqueController :
//     public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
// {
// public:
//   bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override
//   {
//     // ----- joint list -----
//     if(!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR_STREAM("[PassivityCtrl] parameter 'joints' not found");
//       return false;
//     }
//     if(joint_names_.size() != 6)
//     {
//       ROS_ERROR_STREAM("[PassivityCtrl] need 6 joints, got " << joint_names_.size());
//       return false;
//     }

//     // get effort joint handles
//     auto* effort_iface = hw->get<hardware_interface::EffortJointInterface>();
//     if(!effort_iface)
//     {
//       ROS_ERROR("[PassivityCtrl] EffortJointInterface not found");
//       return false;
//     }
//     for(const auto& name : joint_names_)
//       joint_handles_.push_back(effort_iface->getHandle(name));

//     // read torque factors just for logging (conversion done in HW layer)
//     std::vector<double> tf_tmp;
//     if(ros::param::get("/elfin/axis_torque_factors", tf_tmp) && tf_tmp.size()==6)
//       ROS_INFO_STREAM("[PassivityCtrl] axis_torque_factors: " << tf_tmp[0] << ", " << tf_tmp[1]
//                       << ", " << tf_tmp[2] << ", " << tf_tmp[3] << ", " << tf_tmp[4]
//                       << ", " << tf_tmp[5]);
//     else
//       ROS_WARN("[PassivityCtrl] axis_torque_factors not found or size!=6 (ok, HW interface handles default)");

//     // subscribe command topic
//     sub_ = nh.subscribe("/elfin_effort_controller/command_nm", 1,
//                         &PassivityTorqueController::commandCB, this);
//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     std::array<double,6> zero{};
//     cmd_buffer_.writeFromNonRT(zero);
//   }

//   void update(const ros::Time&, const ros::Duration&) override
//   {
//     // read latest command Nm array
//     const std::array<double,6>& cmd = *cmd_buffer_.readFromRT();
//     for(size_t i=0;i<6;++i)
//       joint_handles_[i].setCommand(cmd[i]);   // unit Nm — HW layer converts
//   }

// private:
//   void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
//   {
//     if(msg->data.size()!=6)
//     {
//       ROS_ERROR_THROTTLE(5.0, "[PassivityCtrl] command size %zu !=6", msg->data.size());
//       return;
//     }
//     std::array<double,6> cmd;
//     std::copy_n(msg->data.begin(), 6, cmd.begin());
//     cmd_buffer_.writeFromNonRT(cmd);
//   }

//   // members --------------------------------------------------
//   std::vector<std::string> joint_names_;
//   std::vector<hardware_interface::JointHandle> joint_handles_;
//   realtime_tools::RealtimeBuffer<std::array<double,6>> cmd_buffer_;
//   ros::Subscriber sub_;
// };
// } // namespace passivity_controller

// // === pluginlib export ===
// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(passivity_controller::PassivityTorqueController,
//                        controller_interface::ControllerBase)

// // passivity_controller.cpp  ---  Elfin‑3 torque (effort) controller
// // -------------------------------------------------------------------
// // ▸ Loads axis‑torque factors from the ROS param server (array<int> length 6)
// // ▸ Subscribes   /elfin_effort_controller/command_nm   (std_msgs/Float64MultiArray)
// //      data[0 – 5]  = desired joint torques in **Nm** for joint1 … joint6
// // ▸ Converts Nm → cnt and sends to EtherCAT via ros_control EffortJointInterface
// // ▸ Designed for update_rate 500 Hz (period 0.002 s) but works at any rate
// // --------------------------------------------------------------------

// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <realtime_tools/realtime_buffer.h>

// class PassivityTorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
// {
// public:
//   bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh) override
//   {
//     //-------------------- 1.  joint list ---------------------------
//     if (!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR("[PassivityCtrl] param 'joints' missing");
//       return false;
//     }
//     if (joint_names_.size() != 6)
//     {
//       ROS_ERROR("[PassivityCtrl] need exactly 6 joints, got %zu", joint_names_.size());
//       return false;
//     }
//     joints_.resize(6);
//     for (size_t i = 0; i < 6; ++i)
//     {
//       joints_[i] = hw->getHandle(joint_names_[i]);
//     }

//     //-------------------- 2.  torque factors -----------------------
//     if (!nh.getParam("/elfin/axis_torque_factors", torque_factors_))
//     {
//       ROS_ERROR("[PassivityCtrl] failed to get /elfin/axis_torque_factors");
//       return false;
//     }
//     if (torque_factors_.size() != 6)
//     {
//       ROS_ERROR("[PassivityCtrl] axis_torque_factors must length 6, got %zu", torque_factors_.size());
//       return false;
//     }

//     //-------------------- 3.  RT‑safe command buffer ---------------
//     std::vector<double> zero(6, 0.0);
//     cmd_buffer_.writeFromNonRT(zero);

//     sub_ = nh.subscribe("/elfin_effort_controller/command_nm", 1, &PassivityTorqueController::commandCB, this);
//     ROS_INFO("[PassivityCtrl] initialised. Listening /elfin_effort_controller/command_nm (Nm)");
//     return true;
//   }

//   void starting(const ros::Time &) override
//   {
//     // zero effort on start
//     std::vector<double> zero(6, 0.0);
//     cmd_buffer_.writeFromNonRT(zero);
//   }

//   void update(const ros::Time &, const ros::Duration &period) override
//   {
//     // 1. get desired torques (Nm)
//     const std::vector<double> &cmd_nm = *cmd_buffer_.readFromRT();

//     // 2. convert & send cnt
//     for (size_t i = 0; i < 6; ++i)
//     {
//       double cnt = cmd_nm[i] * static_cast<double>(torque_factors_[i]);
//       joints_[i].setCommand(cnt);
//     }
//   }

// private:
//   //-------------------- helpers -----------------------------------
//   void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//   {
//     if (msg->data.size() != 6)
//     {
//       ROS_WARN_THROTTLE(2.0, "[PassivityCtrl] command length %zu, need 6", msg->data.size());
//       return;
//     }
//     cmd_buffer_.writeFromNonRT(msg->data);
//   }

//   //-------------------- members -----------------------------------
//   std::vector<std::string> joint_names_;
//   std::vector<hardware_interface::JointHandle> joints_;
//   std::vector<double> torque_factors_;  // factors can be float or int; keep as double
//   realtime_tools::RealtimeBuffer<std::vector<double>> cmd_buffer_;
//   ros::Subscriber sub_;
// };

// PLUGINLIB_EXPORT_CLASS(PassivityTorqueController, controller_interface::ControllerBase)

// // passivity_controller.cpp  ---  Elfin‑3 torque (effort) controller
// // -------------------------------------------------------------------
// // ▸ Loads axis‑torque factors from the ROS param server (array<int> length 6)
// // ▸ Subscribes   /elfin_effort_controller/command_nm   (std_msgs/Float64MultiArray)
// //      data[0 – 5]  = desired joint torques in **Nm** for joint1 … joint6
// // ▸ Converts Nm → cnt and sends to EtherCAT via ros_control EffortJointInterface
// // ▸ Designed for update_rate 500 Hz (period 0.002 s) but works at any rate
// // --------------------------------------------------------------------

// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <realtime_tools/realtime_buffer.h>

// class PassivityTorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
// {
// public:
//   bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh) override
//   {
//     //-------------------- 1.  joint list ---------------------------
//     if (!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR("[PassivityCtrl] param 'joints' missing");
//       return false;
//     }
//     if (joint_names_.size() != 6)
//     {
//       ROS_ERROR("[PassivityCtrl] need exactly 6 joints, got %zu", joint_names_.size());
//       return false;
//     }
//     joints_.resize(6);
//     for (size_t i = 0; i < 6; ++i)
//     {
//       joints_[i] = hw->getHandle(joint_names_[i]);
//     }

//     //-------------------- 2.  torque factors -----------------------
//     if (!nh.getParam("/elfin/axis_torque_factors", torque_factors_))
//     {
//       ROS_ERROR("[PassivityCtrl] failed to get /elfin/axis_torque_factors");
//       return false;
//     }
//     if (torque_factors_.size() != 6)
//     {
//       ROS_ERROR("[PassivityCtrl] axis_torque_factors must length 6, got %zu", torque_factors_.size());
//       return false;
//     }

//     //-------------------- 3.  RT‑safe command buffer ---------------
//     std::vector<double> zero(6, 0.0);
//     cmd_buffer_.writeFromNonRT(zero);

//     sub_ = nh.subscribe("/elfin_effort_controller/command_nm", 1, &PassivityTorqueController::commandCB, this);
//     ROS_INFO("[PassivityCtrl] initialised. Listening /elfin_effort_controller/command_nm (Nm)");
//     return true;
//   }

//   void starting(const ros::Time &) override
//   {
//     // zero effort on start
//     std::vector<double> zero(6, 0.0);
//     cmd_buffer_.writeFromNonRT(zero);
//   }

//   void update(const ros::Time &, const ros::Duration &period) override
//   {
//     // 1. get desired torques (Nm)
//     const std::vector<double> &cmd_nm = *cmd_buffer_.readFromRT();

//     // 2. convert & send cnt
//     for (size_t i = 0; i < 6; ++i)
//     {
//       double cnt = cmd_nm[i] * static_cast<double>(torque_factors_[i]);
//       joints_[i].setCommand(cnt);
//     }
//   }

// private:
//   //-------------------- helpers -----------------------------------
//   void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//   {
//     if (msg->data.size() != 6)
//     {
//       ROS_WARN_THROTTLE(2.0, "[PassivityCtrl] command length %zu, need 6", msg->data.size());
//       return;
//     }
//     cmd_buffer_.writeFromNonRT(msg->data);
//   }

//   //-------------------- members -----------------------------------
//   std::vector<std::string> joint_names_;
//   std::vector<hardware_interface::JointHandle> joints_;
//   std::vector<int> torque_factors_;
//   realtime_tools::RealtimeBuffer<std::vector<double>> cmd_buffer_;
//   ros::Subscriber sub_;
// };

// PLUGINLIB_EXPORT_CLASS(PassivityTorqueController, controller_interface::ControllerBase)

// // passivity_controller.cpp — Elfin‑3 torque controller (CST)
// // -----------------------------------------------------------
// // * Publishes effort (torque) commands in **cnt** directly to EtherCAT PDO
// // * Subscribes to std_msgs/Float64MultiArray on  `/elfin_effort_controller/command_nm`
// //   whose data[0..5] are desired torques **in Nm** for joint1 … joint6.
// // * Converts Nm → cnt using axis_torque_factors loaded from
// //   parameter `/elfin/axis_torque_factors`.
// // * Requires that the robot is already in CST mode (0x6060 = 0x0A) and that
// //   `effort_controllers/JointGroupEffortController` is **NOT** running at the same
// //   time (only one controller may claim the same joints).
// //
// // Build:
// //   add_library(passivity_controller src/passivity_controller.cpp)
// //   target_link_libraries(passivity_controller ${catkin_LIBRARIES})
// //   pluginlib_export_class<PassivityController, controller_interface::ControllerBase>(passivity_controller)

// #include <mutex>
// #include <vector>
// #include <algorithm>

// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>

// class PassivityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
// {
// public:
//   bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override
//   {
//     // 1. Joint list ----------------------------------------------------------
//     if (!nh.getParam("joints", joint_names_))
//     {
//       ROS_ERROR_STREAM("[PassivityController] param 'joints' not found");
//       return false;
//     }
//     if (joint_names_.size() != 6)
//     {
//       ROS_ERROR_STREAM("[PassivityController] expecting exactly 6 joints, got " << joint_names_.size());
//       return false;
//     }

//     // 2. Hardware handles ----------------------------------------------------
//     for (const auto& j : joint_names_)
//     {
//       try
//       {
//         joint_handles_.push_back(hw->getHandle(j));
//       }
//       catch (const hardware_interface::HardwareInterfaceException& e)
//       {
//         ROS_ERROR_STREAM("[PassivityController] joint handle error: " << e.what());
//         return false;
//       }
//     }

//     // 3. Torque factors ------------------------------------------------------
//     if (!ros::param::get("/elfin/axis_torque_factors", axis_torque_factors_))
//     {
//       ROS_ERROR_STREAM("[PassivityController] '/elfin/axis_torque_factors' param not found");
//       return false;
//     }
//     if (axis_torque_factors_.size() != 6)
//     {
//       ROS_ERROR_STREAM("[PassivityController] expecting 6 torque factors, got " << axis_torque_factors_.size());
//       return false;
//     }

//     // Pre‑allocate command buffers ------------------------------------------
//     desired_nm_.assign(6, 0.0);
//     desired_cnt_.assign(6, 0.0);

//     // 4. Subscribe command topic --------------------------------------------
//     sub_ = nh.subscribe("command_nm", 1, &PassivityController::commandCB, this);

//     ROS_INFO_STREAM("[PassivityController] initialised for joints " << joint_names_[0] << " … " << joint_names_[5]);
//     return true;
//   }

//   void starting(const ros::Time&) override
//   {
//     std::lock_guard<std::mutex> lock(mtx_);
//     std::fill(desired_nm_.begin(), desired_nm_.end(), 0.0);
//   }

//   void update(const ros::Time&, const ros::Duration&) override
//   {
//     std::lock_guard<std::mutex> lock(mtx_);
//     // Nm → cnt conversion and write to hardware
//     for (size_t i = 0; i < 6; ++i)
//     {
//       desired_cnt_[i] = desired_nm_[i] * static_cast<double>(axis_torque_factors_[i]);
//       joint_handles_[i].setCommand(desired_cnt_[i]);
//     }
//   }

//   void stopping(const ros::Time&) override
//   {
//     // Reset commands to zero
//     for (auto& h : joint_handles_)
//       h.setCommand(0.0);
//   }

// private:
//   void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
//   {
//     if (msg->data.size() < 6)
//     {
//       ROS_WARN_THROTTLE(2.0, "[PassivityController] command length < 6, ignoring");
//       return;
//     }
//     std::lock_guard<std::mutex> lock(mtx_);
//     for (size_t i = 0; i < 6; ++i)
//       desired_nm_[i] = msg->data[i];
//   }

//   // -------------------------------------------------------------------------
//   std::vector<std::string> joint_names_;
//   std::vector<hardware_interface::JointHandle> joint_handles_;

//   std::vector<int> axis_torque_factors_;           // ±cnt per Nm
//   std::vector<double> desired_nm_, desired_cnt_;   // command buffers

//   ros::Subscriber sub_;
//   std::mutex mtx_;
// };

// // Plugin registration --------------------------------------------------------
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(PassivityController, controller_interface::ControllerBase)

// ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(
//                       "/elfin_effort_controller/command", 1);
// std_msgs::Float64MultiArray cmd;
// cmd.layout.dim.clear();
// cmd.data.resize(6);

// ros::Rate loop(500); // 500 Hz
// double t = 0.0;
// while (ros::ok()) {
//     cmd.data[0] = 0.5 * sin(t); // 轴1
//     for (int i=1;i<6;++i) cmd.data[i] = 0; // 其余轴严格写 0
//     pub.publish(cmd);
//     t += 0.002 * M_PI; // ≈0.25 Hz
//     loop.sleep();
// }

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include "arm_controllers/ControllerJointState.h"
#include "arm_controllers/PassivityControllerParamsConfig.h"

#include <iostream>
#include <iomanip> // 引入setw和left所需的头文件

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI

namespace arm_controllers
{

  class TorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
    ~TorqueController()
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

      // kdl parser
      if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
      {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
      }

      // kdl chain
      std::string root_name, tip_name;
      if (!n.getParam("root_link", root_name))
      {
        ROS_ERROR("Could not find root link name");
        return false;
      }
      if (!n.getParam("tip_link", tip_name))
      {
        ROS_ERROR("Could not find tip link name");
        return false;
      }
      if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
      {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for (it = segment_map.begin(); it != segment_map.end(); it++)
          ROS_ERROR_STREAM("    " << (*it).first);

        return false;
      }

      gravity_ = KDL::Vector::Zero();
      gravity_(2) = -9.81;
      M_.resize(n_joints_);
      C_.resize(n_joints_);
      G_.resize(n_joints_);

      // inverse dynamics solver
      id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

      // command and state
      tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
      q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
      q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
      qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
      qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
      qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

      q_.data = Eigen::VectorXd::Zero(n_joints_);
      qdot_.data = Eigen::VectorXd::Zero(n_joints_);

      q_error_.data = Eigen::VectorXd::Zero(n_joints_);
      q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

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
      command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &TorqueController::commandCB, this);

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

    // void update(const ros::Time &time, const ros::Duration &period)
    // {
    //     std::vector<double> &commands = *commands_buffer_.readFromRT();
    //     std::cout<<commands[0];
    //     double dt = period.toSec();

    //     // 移除从终端读取力矩指令的代码

    //     // get joint states
    //     for (size_t i = 0; i < n_joints_; i++)
    //     {
    //         q_(i) = joints_[i].getPosition();
    //         qdot_(i) = joints_[i].getVelocity();

    //         // 所有关节目标位置设为初始位置（保持不动）
    //         qdot_cmd_(i) = 0.0;
    //         qddot_cmd_(i) = 0.0;

    //         // Compute position error
    //         if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
    //         {
    //             angles::shortest_angular_distance_with_limits(
    //                 q_(i),
    //                 q_cmd_(i),
    //                 joint_urdfs_[i]->limits->lower,
    //                 joint_urdfs_[i]->limits->upper,
    //                 q_error_(i));
    //         }
    //         else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
    //         {
    //             q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
    //         }
    //         else // prismatic
    //         {
    //             q_error_(i) = q_cmd_(i) - q_(i);
    //         }

    //         q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

    //         // friction compensation
    //         tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
    //     }

    //     // compute dynamics term
    //     id_solver_->JntToMass(q_, M_);
    //     id_solver_->JntToCoriolis(q_, qdot_, C_);
    //     id_solver_->JntToGravity(q_, G_);

    //     // torque command
    //     GainsHandler::Gains gains = gains_handler_.getGains();
    //     qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
    //     qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

    //     // 直接设置力矩指令：关节1为1Nm，其他关节为0Nm
    //     for (size_t i = 0; i < n_joints_; i++)
    //     {
    //         // if (i == 0) {  // 关节1的索引通常为0
    //         //     tau_cmd_(i) = 1.0;  // 设置关节1的力矩为1Nm
    //         // } else {
    //         //     tau_cmd_(i) = 0.0;  // 其他关节力矩为0Nm
    //         // }
    //         tau_cmd_(i) = commands[i];
    //     }

    //     // 计算最大力矩值的宽度（假设为2位小数）
    //     const int value_width = 13; // 包含小数点和小数位
    //     for (int i = 0; i < n_joints_; i++)
    //     {
    //         controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

    //         // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
    //         // if (i != 2)
    //         // {
    //         //     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
    //         // }

    //         // effort saturation
    //         if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
    //             tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

    //         if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
    //             tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

    //         joints_[i].setCommand(tau_cmd_(i));
    //         actual_torque = joints_[i].getEffort();
    //         // 只输出力矩值，使用固定宽度对齐
    //         std::cout << std::right
    //                   << std::setw(value_width)
    //                   << std::fixed << std::setprecision(2) << tau_cmd_(i)
    //                   << std::right
    //                   << std::setw(value_width)
    //                   << std::fixed << std::setprecision(2) << actual_torque;
    //     }
    //     std::cout << std::endl; // 最后换行

    //     // publish
    //     if (loop_count_ % 10 == 0)
    //     {
    //         if (controller_state_pub_->trylock())
    //         {
    //             controller_state_pub_->msg_.header.stamp = time;
    //             for (int i = 0; i < n_joints_; i++)
    //             {
    //                 controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
    //                 controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
    //                 controller_state_pub_->msg_.state[i] = R2D * q_(i);
    //                 controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
    //                 controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
    //                 controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
    //                 controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
    //                 controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
    //             }
    //             controller_state_pub_->unlockAndPublish();
    //         }
    //     }
    //     loop_count_++;
    // }

    // void update(const ros::Time &time, const ros::Duration &period)
    // {
    //     std::vector<double> &commands = *commands_buffer_.readFromRT();
    //     double dt = period.toSec();

    //     // get joint states
    //     for (size_t i = 0; i < n_joints_; i++)
    //     {
    //         q_(i) = joints_[i].getPosition();
    //         qdot_(i) = joints_[i].getVelocity();

    //         // 所有关节目标位置设为初始位置（保持不动）
    //         qdot_cmd_(i) = 0.0;
    //         qddot_cmd_(i) = 0.0;

    //         // Compute position error
    //         if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
    //         {
    //             angles::shortest_angular_distance_with_limits(
    //                 q_(i),
    //                 q_cmd_(i),
    //                 joint_urdfs_[i]->limits->lower,
    //                 joint_urdfs_[i]->limits->upper,
    //                 q_error_(i));
    //         }
    //         else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
    //         {
    //             q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
    //         }
    //         else // prismatic
    //         {
    //             q_error_(i) = q_cmd_(i) - q_(i);
    //         }

    //         q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

    //         // friction compensation
    //         tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
    //     }

    //     // compute dynamics term
    //     id_solver_->JntToMass(q_, M_);
    //     id_solver_->JntToCoriolis(q_, qdot_, C_);
    //     id_solver_->JntToGravity(q_, G_);

    //     // torque command
    //     GainsHandler::Gains gains = gains_handler_.getGains();
    //     qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
    //     qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

    //     // 设置力矩指令：关节1为1*sin(t) Nm，其他关节为0 Nm
    //     double t = time.toSec();
    //     for (size_t i = 0; i < n_joints_; i++)
    //     {
    //         if (i == 0)
    //         {                                    // 关节1的索引通常为0
    //             tau_cmd_(i) = 8.0 * std::sin(t); // 设置关节1的力矩为1*sin(t) Nm
    //         }
    //         else
    //         {
    //             tau_cmd_(i) = 0.0; // 其他关节力矩为0 Nm
    //         }
    //     }

    //     // 计算最大力矩值的宽度（假设为2位小数）
    //     const int value_width = 13; // 包含小数点和小数位
    //     for (int i = 0; i < n_joints_; i++)
    //     {
    //         controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

    //         // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
    //         // if (i != 2)
    //         // {
    //         //     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
    //         // }

    //         // effort saturation
    //         if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
    //             tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

    //         if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
    //             tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

    //         joints_[i].setCommand(tau_cmd_(i));
    //         // tau_cmd_(i) = commands[i];
    //         actual_torque = joints_[i].getEffort();
    //         // 只输出力矩值，使用固定宽度对齐
    //         std::cout << std::right
    //                   << std::setw(value_width)
    //                   << std::fixed << std::setprecision(2) << tau_cmd_(i)
    //                   << std::right
    //                   << std::setw(value_width)
    //                   << std::fixed << std::setprecision(2) << actual_torque;
    //     }
    //     std::cout << std::endl; // 最后换行

    //     // publish
    //     if (loop_count_ % 10 == 0)
    //     {
    //         if (controller_state_pub_->trylock())
    //         {
    //             controller_state_pub_->msg_.header.stamp = time;
    //             for (int i = 0; i < n_joints_; i++)
    //             {
    //                 controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
    //                 controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
    //                 controller_state_pub_->msg_.state[i] = R2D * q_(i);
    //                 controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
    //                 controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
    //                 controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
    //                 controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
    //                 controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
    //             }
    //             controller_state_pub_->unlockAndPublish();
    //         }
    //     }
    //     loop_count_++;
    // }
    void update(const ros::Time &time, const ros::Duration &period)
    {
      std::vector<double> &commands = *commands_buffer_.readFromRT();
      double dt = period.toSec();
      double q_cmd_old;

      // get joint states
      static double t = 0;
      double angle = 45 * KDL::deg2rad;
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
        tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
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
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
    KDL::JntSpaceInertiaMatrix M_;
    KDL::JntArray C_;
    KDL::JntArray G_; // gravity torque vector
    KDL::Vector gravity_;

    // cmd, state
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
    KDL::JntArray tau_cmd_, tau_fric_;
    KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
    KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
    KDL::JntArray q_, qdot_;
    KDL::JntArray q_error_, q_error_dot_;

    // gain
    GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
    std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

    // topic
    ros::Subscriber command_sub_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            arm_controllers::ControllerJointState>>
        controller_state_pub_;
  };

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::TorqueController, controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <control_toolbox/pid.h>
// #include <realtime_tools/realtime_buffer.h>

// #include <pluginlib/class_list_macros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <angles/angles.h>

// #include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include <boost/scoped_ptr.hpp>

// #include "arm_controllers/ControllerJointState.h"
// #include "arm_controllers/PassivityControllerParamsConfig.h"

// #include <iostream>
// #include <iomanip> // 引入setw和left所需的头文件

// #define PI 3.141592
// #define D2R PI / 180.0
// #define R2D 180.0 / PI

// namespace arm_controllers
// {

//     class TorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
//     {
//         class GainsHandler
//         {
//         public:
//             struct Gains
//             {
//                 Gains() : alpha_(0.0) {}
//                 Gains(double alpha) : alpha_(alpha) {}

//                 double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
//             };

//             GainsHandler() {}

//             bool initDynamicReconfig(const ros::NodeHandle &node)
//             {
//                 ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

//                 Gains gains;
//                 if (!node.getParam("alpha", gains.alpha_))
//                 {
//                     ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
//                     return false;
//                 }

//                 // Start dynamic reconfigure server
//                 typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//                 param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
//                 dynamic_reconfig_initialized_ = true;

//                 setGains(gains);

//                 // Set Dynamic Reconfigure's gains to Pid's values
//                 updateDynamicReconfig();

//                 // Set callback
//                 param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
//                 param_reconfig_server_->setCallback(param_reconfig_callback_);

//                 return true;
//             }

//             void getGains(double &alpha)
//             {
//                 Gains gains = *gains_buffer_.readFromRT();
//                 alpha = gains.alpha_;
//             }

//             Gains getGains()
//             {
//                 return *gains_buffer_.readFromRT();
//             }

//             void setGains(double alpha)
//             {
//                 Gains gains(alpha);

//                 setGains(gains);
//             }

//             void setGains(const Gains &gains)
//             {
//                 gains_buffer_.writeFromNonRT(gains);

//                 updateDynamicReconfig(gains);
//             }

//             void updateDynamicReconfig()
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Get starting values
//                 PassivityControllerParamsConfig config;
//                 getGains(config.alpha);

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(Gains gains)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 PassivityControllerParamsConfig config;

//                 // Convert to dynamic reconfigure format
//                 config.alpha = gains.alpha_;

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(PassivityControllerParamsConfig config)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Set starting values, using a shared mutex with dynamic reconfig
//                 param_reconfig_mutex_.lock();
//                 param_reconfig_server_->updateConfig(config);
//                 param_reconfig_mutex_.unlock();
//             }

//             void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
//             {
//                 ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

//                 // Set the gains
//                 setGains(config.alpha);
//             }

//         private:
//             // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
//             // blocking the realtime update loop
//             realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

//             // Dynamics reconfigure
//             bool dynamic_reconfig_initialized_;
//             typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//             boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
//             DynamicReconfigServer::CallbackType param_reconfig_callback_;

//             boost::recursive_mutex param_reconfig_mutex_;
//         };

//     public:
//         ~TorqueController()
//         {
//             command_sub_.shutdown();
//         }

//         bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
//         {
//             loop_count_ = 0;
//             // List of controlled joints
//             if (!n.getParam("joints", joint_names_))
//             {
//                 ROS_ERROR("Could not find joint name");
//                 return false;
//             }
//             n_joints_ = joint_names_.size();

//             if (n_joints_ == 0)
//             {
//                 ROS_ERROR("List of joint names is empty.");
//                 return false;
//             }

//             // urdf
//             urdf::Model urdf;
//             if (!urdf.initParam("robot_description"))
//             {
//                 ROS_ERROR("Failed to parse urdf file");
//                 return false;
//             }

//             // joint handle
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 try
//                 {
//                     joints_.push_back(hw->getHandle(joint_names_[i]));
//                 }
//                 catch (const hardware_interface::HardwareInterfaceException &e)
//                 {
//                     ROS_ERROR_STREAM("Exception thrown: " << e.what());
//                     return false;
//                 }

//                 urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
//                 if (!joint_urdf)
//                 {
//                     ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
//                     return false;
//                 }
//                 joint_urdfs_.push_back(joint_urdf);
//             }

//             // kdl parser
//             if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
//             {
//                 ROS_ERROR("Failed to construct kdl tree");
//                 return false;
//             }

//             // kdl chain
//             std::string root_name, tip_name;
//             if (!n.getParam("root_link", root_name))
//             {
//                 ROS_ERROR("Could not find root link name");
//                 return false;
//             }
//             if (!n.getParam("tip_link", tip_name))
//             {
//                 ROS_ERROR("Could not find tip link name");
//                 return false;
//             }
//             if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
//             {
//                 ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
//                 ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
//                 ROS_ERROR_STREAM("  The segments are:");

//                 KDL::SegmentMap segment_map = kdl_tree_.getSegments();
//                 KDL::SegmentMap::iterator it;

//                 for (it = segment_map.begin(); it != segment_map.end(); it++)
//                     ROS_ERROR_STREAM("    " << (*it).first);

//                 return false;
//             }

//             gravity_ = KDL::Vector::Zero();
//             gravity_(2) = -9.81;
//             M_.resize(n_joints_);
//             C_.resize(n_joints_);
//             G_.resize(n_joints_);

//             // inverse dynamics solver
//             id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

//             // command and state
//             tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_error_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

//             // gains
//             if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
//             {
//                 ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
//                 return false;
//             }

//             pids_.resize(n_joints_);
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
//                 {
//                     ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
//                     return false;
//                 }
//             }

//             // command subscriber
//             commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
//             command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &TorqueController::commandCB, this);

//             // start realtime state publisher
//             controller_state_pub_.reset(
//                 new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

//             controller_state_pub_->msg_.header.stamp = ros::Time::now();
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.name.push_back(joint_names_[i]);
//                 controller_state_pub_->msg_.command.push_back(0.0);
//                 controller_state_pub_->msg_.command_dot.push_back(0.0);
//                 controller_state_pub_->msg_.state.push_back(0.0);
//                 controller_state_pub_->msg_.state_dot.push_back(0.0);
//                 controller_state_pub_->msg_.error.push_back(0.0);
//                 controller_state_pub_->msg_.error_dot.push_back(0.0);
//                 controller_state_pub_->msg_.effort_command.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedback.push_back(0.0);
//             }

//             return true;
//         }

//         void starting(const ros::Time &time)
//         {
//             // get joint positions
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 记录初始位置作为目标位置
//                 q_cmd_(i) = q_(i);
//             }

//             ROS_INFO("Starting Torque Controller");
//         }

//         void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//         {
//             if (msg->data.size() != n_joints_)
//             {
//                 ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
//                 return;
//             }
//             commands_buffer_.writeFromNonRT(msg->data);
//         }

//         void update(const ros::Time &time, const ros::Duration &period)
//         {
//             std::vector<double> &commands = *commands_buffer_.readFromRT();
//             double dt = period.toSec();

//             // 从终端读取力矩指令
//             std::cout << "Please enter torque commands for " << n_joints_ << " joints (separated by spaces): ";
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 std::cin >> commands[i];
//             }

//             // get joint states
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 所有关节目标位置设为初始位置（保持不动）
//                 qdot_cmd_(i) = 0.0;
//                 qddot_cmd_(i) = 0.0;

//                 // Compute position error
//                 if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
//                 {
//                     angles::shortest_angular_distance_with_limits(
//                         q_(i),
//                         q_cmd_(i),
//                         joint_urdfs_[i]->limits->lower,
//                         joint_urdfs_[i]->limits->upper,
//                         q_error_(i));
//                 }
//                 else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
//                 {
//                     q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
//                 }
//                 else // prismatic
//                 {
//                     q_error_(i) = q_cmd_(i) - q_(i);
//                 }

//                 q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

//                 // friction compensation
//                 tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
//             }

//             // compute dynamics term
//             id_solver_->JntToMass(q_, M_);
//             id_solver_->JntToCoriolis(q_, qdot_, C_);
//             id_solver_->JntToGravity(q_, G_);

//             // torque command
//             GainsHandler::Gains gains = gains_handler_.getGains();
//             qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
//             qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

//             // 使用终端输入的力矩值直接控制关节
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 tau_cmd_(i) = commands[i];
//             }

//             // 计算最大力矩值的宽度（假设为2位小数）
//             const int value_width = 13; // 包含小数点和小数位
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

//                 // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
//                 if (i != 2)
//                 {
//                     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
//                 }

//                 // effort saturation
//                 if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

//                 if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

//                 joints_[i].setCommand(tau_cmd_(i));
//                 actual_torque = joints_[i].getEffort();
//                 // 只输出力矩值，使用固定宽度对齐
//                 std::cout << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << tau_cmd_(i)
//                           << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << actual_torque;
//             }
//             std::cout << std::endl; // 最后换行

//             // publish
//             if (loop_count_ % 10 == 0)
//             {
//                 if (controller_state_pub_->trylock())
//                 {
//                     controller_state_pub_->msg_.header.stamp = time;
//                     for (int i = 0; i < n_joints_; i++)
//                     {
//                         controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
//                         controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
//                         controller_state_pub_->msg_.state[i] = R2D * q_(i);
//                         controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
//                         controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
//                         controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
//                         controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
//                         controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
//                     }
//                     controller_state_pub_->unlockAndPublish();
//                 }
//             }
//             loop_count_++;
//         }

//         void stopping(const ros::Time &time) {}

//         void enforceJointLimits(double &command, unsigned int index)
//         {
//             // Check that this joint has applicable limits
//             if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
//             {
//                 if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
//                 {
//                     command = joint_urdfs_[index]->limits->upper;
//                 }
//                 else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
//                 {
//                     command = joint_urdfs_[index]->limits->lower;
//                 }
//             }
//         }

//     private:
//         int loop_count_;
//         double actual_torque;
//         // joint handles
//         unsigned int n_joints_;
//         std::vector<std::string> joint_names_;
//         std::vector<hardware_interface::JointHandle> joints_;
//         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

//         // kdl
//         KDL::Tree kdl_tree_;
//         KDL::Chain kdl_chain_;
//         boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
//         KDL::JntSpaceInertiaMatrix M_;
//         KDL::JntArray C_;
//         KDL::JntArray G_; // gravity torque vector
//         KDL::Vector gravity_;

//         // cmd, state
//         realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
//         KDL::JntArray tau_cmd_, tau_fric_;
//         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
//         KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
//         KDL::JntArray q_, qdot_;
//         KDL::JntArray q_error_, q_error_dot_;

//         // gain
//         GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
//         std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

//         // topic
//         ros::Subscriber command_sub_;
//         boost::scoped_ptr<
//             realtime_tools::RealtimePublisher<
//                 arm_controllers::ControllerJointState>>
//             controller_state_pub_;
//     };

// }

// PLUGINLIB_EXPORT_CLASS(arm_controllers::TorqueController, controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <control_toolbox/pid.h>
// #include <realtime_tools/realtime_buffer.h>

// #include <pluginlib/class_list_macros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <angles/angles.h>

// #include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include <boost/scoped_ptr.hpp>

// #include "arm_controllers/ControllerJointState.h"
// #include "arm_controllers/PassivityControllerParamsConfig.h"

// #include <iostream>
// #include <iomanip> // 引入setw和left所需的头文件

// #define PI 3.141592
// #define D2R PI / 180.0
// #define R2D 180.0 / PI

// namespace arm_controllers
// {

//     class TorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
//     {
//         class GainsHandler
//         {
//         public:
//             struct Gains
//             {
//                 Gains() : alpha_(0.0) {}
//                 Gains(double alpha) : alpha_(alpha) {}

//                 double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
//             };

//             GainsHandler() {}

//             bool initDynamicReconfig(const ros::NodeHandle &node)
//             {
//                 ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

//                 Gains gains;
//                 if (!node.getParam("alpha", gains.alpha_))
//                 {
//                     ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
//                     return false;
//                 }

//                 // Start dynamic reconfigure server
//                 typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//                 param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
//                 dynamic_reconfig_initialized_ = true;

//                 setGains(gains);

//                 // Set Dynamic Reconfigure's gains to Pid's values
//                 updateDynamicReconfig();

//                 // Set callback
//                 param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
//                 param_reconfig_server_->setCallback(param_reconfig_callback_);

//                 return true;
//             }

//             void getGains(double &alpha)
//             {
//                 Gains gains = *gains_buffer_.readFromRT();
//                 alpha = gains.alpha_;
//             }

//             Gains getGains()
//             {
//                 return *gains_buffer_.readFromRT();
//             }

//             void setGains(double alpha)
//             {
//                 Gains gains(alpha);

//                 setGains(gains);
//             }

//             void setGains(const Gains &gains)
//             {
//                 gains_buffer_.writeFromNonRT(gains);

//                 updateDynamicReconfig(gains);
//             }

//             void updateDynamicReconfig()
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Get starting values
//                 PassivityControllerParamsConfig config;
//                 getGains(config.alpha);

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(Gains gains)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 PassivityControllerParamsConfig config;

//                 // Convert to dynamic reconfigure format
//                 config.alpha = gains.alpha_;

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(PassivityControllerParamsConfig config)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Set starting values, using a shared mutex with dynamic reconfig
//                 param_reconfig_mutex_.lock();
//                 param_reconfig_server_->updateConfig(config);
//                 param_reconfig_mutex_.unlock();
//             }

//             void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
//             {
//                 ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

//                 // Set the gains
//                 setGains(config.alpha);
//             }

//         private:
//             // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
//             // blocking the realtime update loop
//             realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

//             // Dynamics reconfigure
//             bool dynamic_reconfig_initialized_;
//             typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//             boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
//             DynamicReconfigServer::CallbackType param_reconfig_callback_;

//             boost::recursive_mutex param_reconfig_mutex_;
//         };

//     public:
//         ~TorqueController()
//         {
//             command_sub_.shutdown();
//         }

//         bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
//         {
//             loop_count_ = 0;
//             // List of controlled joints
//             if (!n.getParam("joints", joint_names_))
//             {
//                 ROS_ERROR("Could not find joint name");
//                 return false;
//             }
//             n_joints_ = joint_names_.size();

//             if (n_joints_ == 0)
//             {
//                 ROS_ERROR("List of joint names is empty.");
//                 return false;
//             }

//             // urdf
//             urdf::Model urdf;
//             if (!urdf.initParam("robot_description"))
//             {
//                 ROS_ERROR("Failed to parse urdf file");
//                 return false;
//             }

//             // joint handle
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 try
//                 {
//                     joints_.push_back(hw->getHandle(joint_names_[i]));
//                 }
//                 catch (const hardware_interface::HardwareInterfaceException &e)
//                 {
//                     ROS_ERROR_STREAM("Exception thrown: " << e.what());
//                     return false;
//                 }

//                 urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
//                 if (!joint_urdf)
//                 {
//                     ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
//                     return false;
//                 }
//                 joint_urdfs_.push_back(joint_urdf);
//             }

//             // kdl parser
//             if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
//             {
//                 ROS_ERROR("Failed to construct kdl tree");
//                 return false;
//             }

//             // kdl chain
//             std::string root_name, tip_name;
//             if (!n.getParam("root_link", root_name))
//             {
//                 ROS_ERROR("Could not find root link name");
//                 return false;
//             }
//             if (!n.getParam("tip_link", tip_name))
//             {
//                 ROS_ERROR("Could not find tip link name");
//                 return false;
//             }
//             if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
//             {
//                 ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
//                 ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
//                 ROS_ERROR_STREAM("  The segments are:");

//                 KDL::SegmentMap segment_map = kdl_tree_.getSegments();
//                 KDL::SegmentMap::iterator it;

//                 for (it = segment_map.begin(); it != segment_map.end(); it++)
//                     ROS_ERROR_STREAM("    " << (*it).first);

//                 return false;
//             }

//             gravity_ = KDL::Vector::Zero();
//             gravity_(2) = -9.81;
//             M_.resize(n_joints_);
//             C_.resize(n_joints_);
//             G_.resize(n_joints_);

//             // inverse dynamics solver
//             id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

//             // command and state
//             tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_error_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

//             // gains
//             if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
//             {
//                 ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
//                 return false;
//             }

//             pids_.resize(n_joints_);
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
//                 {
//                     ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
//                     return false;
//                 }
//             }

//             // command subscriber
//             commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
//             command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &TorqueController::commandCB, this);

//             // start realtime state publisher
//             controller_state_pub_.reset(
//                 new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

//             controller_state_pub_->msg_.header.stamp = ros::Time::now();
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.name.push_back(joint_names_[i]);
//                 controller_state_pub_->msg_.command.push_back(0.0);
//                 controller_state_pub_->msg_.command_dot.push_back(0.0);
//                 controller_state_pub_->msg_.state.push_back(0.0);
//                 controller_state_pub_->msg_.state_dot.push_back(0.0);
//                 controller_state_pub_->msg_.error.push_back(0.0);
//                 controller_state_pub_->msg_.error_dot.push_back(0.0);
//                 controller_state_pub_->msg_.effort_command.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedback.push_back(0.0);
//             }

//             return true;
//         }

//         void starting(const ros::Time &time)
//         {
//             // get joint positions
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 记录初始位置作为目标位置
//                 q_cmd_(i) = q_(i);
//             }

//             ROS_INFO("Starting Torque Controller");
//         }

//         void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//         {
//             if (msg->data.size() != n_joints_)
//             {
//                 ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
//                 return;
//             }
//             commands_buffer_.writeFromNonRT(msg->data);
//         }

//         void update(const ros::Time &time, const ros::Duration &period)
//         {
//             std::vector<double> &commands = *commands_buffer_.readFromRT();
//             double dt = period.toSec();

//             // get joint states
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 所有关节目标位置设为初始位置（保持不动）
//                 qdot_cmd_(i) = 0.0;
//                 qddot_cmd_(i) = 0.0;

//                 // Compute position error
//                 if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
//                 {
//                     angles::shortest_angular_distance_with_limits(
//                         q_(i),
//                         q_cmd_(i),
//                         joint_urdfs_[i]->limits->lower,
//                         joint_urdfs_[i]->limits->upper,
//                         q_error_(i));
//                 }
//                 else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
//                 {
//                     q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
//                 }
//                 else // prismatic
//                 {
//                     q_error_(i) = q_cmd_(i) - q_(i);
//                 }

//                 q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

//                 // friction compensation
//                 tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
//             }

//             // compute dynamics term
//             id_solver_->JntToMass(q_, M_);
//             id_solver_->JntToCoriolis(q_, qdot_, C_);
//             id_solver_->JntToGravity(q_, G_);

//             // torque command
//             GainsHandler::Gains gains = gains_handler_.getGains();
//             qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
//             qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

//             // 使用命令消息中的力矩值直接控制关节
//             std::cout << n_joints_;
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 tau_cmd_(i) = commands[i];
//             }

//             // 计算最大力矩值的宽度（假设为2位小数）
//             const int value_width = 13; // 包含小数点和小数位
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

//                 // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
//                 if (i != 2)
//                 {
//                     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
//                 }

//                 // effort saturation
//                 if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

//                 if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

//                 joints_[i].setCommand(tau_cmd_(i));
//                 actual_torque = joints_[i].getEffort();
//                 // 只输出力矩值，使用固定宽度对齐
//                 std::cout << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << tau_cmd_(i)
//                           << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << actual_torque;
//             }
//             // // 可选：读取当前实际力矩值（如果硬件支持反馈）
//             //             double actual_torque = joints_.getEffort();
//             //             ROS_INFO_STREAM("Actual torque feedback: " << actual_torque << " Nm");
//             std::cout << std::endl; // 最后换行

//             // publish
//             if (loop_count_ % 10 == 0)
//             {
//                 if (controller_state_pub_->trylock())
//                 {
//                     controller_state_pub_->msg_.header.stamp = time;
//                     for (int i = 0; i < n_joints_; i++)
//                     {
//                         controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
//                         controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
//                         controller_state_pub_->msg_.state[i] = R2D * q_(i);
//                         controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
//                         controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
//                         controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
//                         controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
//                         controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
//                     }
//                     controller_state_pub_->unlockAndPublish();
//                 }
//             }
//             loop_count_++;
//         }

//         void stopping(const ros::Time &time) {}

//         void enforceJointLimits(double &command, unsigned int index)
//         {
//             // Check that this joint has applicable limits
//             if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
//             {
//                 if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
//                 {
//                     command = joint_urdfs_[index]->limits->upper;
//                 }
//                 else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
//                 {
//                     command = joint_urdfs_[index]->limits->lower;
//                 }
//             }
//         }

//     private:
//         int loop_count_;
//         double actual_torque;
//         // joint handles
//         unsigned int n_joints_;
//         std::vector<std::string> joint_names_;
//         std::vector<hardware_interface::JointHandle> joints_;
//         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

//         // kdl
//         KDL::Tree kdl_tree_;
//         KDL::Chain kdl_chain_;
//         boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
//         KDL::JntSpaceInertiaMatrix M_;
//         KDL::JntArray C_;
//         KDL::JntArray G_; // gravity torque vector
//         KDL::Vector gravity_;

//         // cmd, state
//         realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
//         KDL::JntArray tau_cmd_, tau_fric_;
//         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
//         KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
//         KDL::JntArray q_, qdot_;
//         KDL::JntArray q_error_, q_error_dot_;

//         // gain
//         GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
//         std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

//         // topic
//         ros::Subscriber command_sub_;
//         boost::scoped_ptr<
//             realtime_tools::RealtimePublisher<
//                 arm_controllers::ControllerJointState>>
//             controller_state_pub_;
//     };

// }

// PLUGINLIB_EXPORT_CLASS(arm_controllers::TorqueController, controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <control_toolbox/pid.h>
// #include <realtime_tools/realtime_buffer.h>

// #include <pluginlib/class_list_macros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <angles/angles.h>

// #include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include <boost/scoped_ptr.hpp>

// #include "arm_controllers/ControllerJointState.h"
// #include "arm_controllers/PassivityControllerParamsConfig.h"

// #include <iostream>
// #include <iomanip> // 引入setw和left所需的头文件

// #define PI 3.141592
// #define D2R PI / 180.0
// #define R2D 180.0 / PI

// namespace arm_controllers
// {

//     class TorqueController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
//     {
//         class GainsHandler
//         {
//         public:
//             struct Gains
//             {
//                 Gains() : alpha_(0.0) {}
//                 Gains(double alpha) : alpha_(alpha) {}

//                 double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
//             };

//             GainsHandler() {}

//             bool initDynamicReconfig(const ros::NodeHandle &node)
//             {
//                 ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

//                 Gains gains;
//                 if (!node.getParam("alpha", gains.alpha_))
//                 {
//                     ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
//                     return false;
//                 }

//                 // Start dynamic reconfigure server
//                 typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//                 param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
//                 dynamic_reconfig_initialized_ = true;

//                 setGains(gains);

//                 // Set Dynamic Reconfigure's gains to Pid's values
//                 updateDynamicReconfig();

//                 // Set callback
//                 param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
//                 param_reconfig_server_->setCallback(param_reconfig_callback_);

//                 return true;
//             }

//             void getGains(double &alpha)
//             {
//                 Gains gains = *gains_buffer_.readFromRT();
//                 alpha = gains.alpha_;
//             }

//             Gains getGains()
//             {
//                 return *gains_buffer_.readFromRT();
//             }

//             void setGains(double alpha)
//             {
//                 Gains gains(alpha);

//                 setGains(gains);
//             }

//             void setGains(const Gains &gains)
//             {
//                 gains_buffer_.writeFromNonRT(gains);

//                 updateDynamicReconfig(gains);
//             }

//             void updateDynamicReconfig()
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Get starting values
//                 PassivityControllerParamsConfig config;
//                 getGains(config.alpha);

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(Gains gains)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 PassivityControllerParamsConfig config;

//                 // Convert to dynamic reconfigure format
//                 config.alpha = gains.alpha_;

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(PassivityControllerParamsConfig config)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Set starting values, using a shared mutex with dynamic reconfig
//                 param_reconfig_mutex_.lock();
//                 param_reconfig_server_->updateConfig(config);
//                 param_reconfig_mutex_.unlock();
//             }

//             void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
//             {
//                 ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

//                 // Set the gains
//                 setGains(config.alpha);
//             }

//         private:
//             // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
//             // blocking the realtime update loop
//             realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

//             // Dynamics reconfigure
//             bool dynamic_reconfig_initialized_;
//             typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//             boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
//             DynamicReconfigServer::CallbackType param_reconfig_callback_;

//             boost::recursive_mutex param_reconfig_mutex_;
//         };

//     public:
//         ~TorqueController()
//         {
//             command_sub_.shutdown();
//         }

//         bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
//         {
//             loop_count_ = 0;
//             // List of controlled joints
//             if (!n.getParam("joints", joint_names_))
//             {
//                 ROS_ERROR("Could not find joint name");
//                 return false;
//             }
//             n_joints_ = joint_names_.size();

//             if (n_joints_ == 0)
//             {
//                 ROS_ERROR("List of joint names is empty.");
//                 return false;
//             }

//             // urdf
//             urdf::Model urdf;
//             if (!urdf.initParam("robot_description"))
//             {
//                 ROS_ERROR("Failed to parse urdf file");
//                 return false;
//             }

//             // joint handle
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 try
//                 {
//                     joints_.push_back(hw->getHandle(joint_names_[i]));
//                 }
//                 catch (const hardware_interface::HardwareInterfaceException &e)
//                 {
//                     ROS_ERROR_STREAM("Exception thrown: " << e.what());
//                     return false;
//                 }

//                 urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
//                 if (!joint_urdf)
//                 {
//                     ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
//                     return false;
//                 }
//                 joint_urdfs_.push_back(joint_urdf);
//             }

//             // kdl parser
//             if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
//             {
//                 ROS_ERROR("Failed to construct kdl tree");
//                 return false;
//             }

//             // kdl chain
//             std::string root_name, tip_name;
//             if (!n.getParam("root_link", root_name))
//             {
//                 ROS_ERROR("Could not find root link name");
//                 return false;
//             }
//             if (!n.getParam("tip_link", tip_name))
//             {
//                 ROS_ERROR("Could not find tip link name");
//                 return false;
//             }
//             if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
//             {
//                 ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
//                 ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
//                 ROS_ERROR_STREAM("  The segments are:");

//                 KDL::SegmentMap segment_map = kdl_tree_.getSegments();
//                 KDL::SegmentMap::iterator it;

//                 for (it = segment_map.begin(); it != segment_map.end(); it++)
//                     ROS_ERROR_STREAM("    " << (*it).first);

//                 return false;
//             }

//             gravity_ = KDL::Vector::Zero();
//             gravity_(2) = -9.81;
//             M_.resize(n_joints_);
//             C_.resize(n_joints_);
//             G_.resize(n_joints_);

//             // inverse dynamics solver
//             id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

//             // command and state
//             tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_error_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

//             // gains
//             if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
//             {
//                 ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
//                 return false;
//             }

//             pids_.resize(n_joints_);
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
//                 {
//                     ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
//                     return false;
//                 }
//             }

//             // command subscriber
//             commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
//             command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &TorqueController::commandCB, this);

//             // start realtime state publisher
//             controller_state_pub_.reset(
//                 new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

//             controller_state_pub_->msg_.header.stamp = ros::Time::now();
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.name.push_back(joint_names_[i]);
//                 controller_state_pub_->msg_.command.push_back(0.0);
//                 controller_state_pub_->msg_.command_dot.push_back(0.0);
//                 controller_state_pub_->msg_.state.push_back(0.0);
//                 controller_state_pub_->msg_.state_dot.push_back(0.0);
//                 controller_state_pub_->msg_.error.push_back(0.0);
//                 controller_state_pub_->msg_.error_dot.push_back(0.0);
//                 controller_state_pub_->msg_.effort_command.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedback.push_back(0.0);
//             }

//             return true;
//         }

//         void starting(const ros::Time &time)
//         {
//             // get joint positions
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 记录初始位置作为目标位置
//                 q_cmd_(i) = q_(i);
//             }

//             ROS_INFO("Starting Torque Controller");
//         }

//         void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//         {
//             if (msg->data.size() != n_joints_)
//             {
//                 ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
//                 return;
//             }
//             commands_buffer_.writeFromNonRT(msg->data);
//         }

//         void update(const ros::Time &time, const ros::Duration &period)
//         {
//             std::vector<double> &commands = *commands_buffer_.readFromRT();
//             double dt = period.toSec();

//             // get joint states
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 所有关节目标位置设为初始位置（保持不动）
//                 qdot_cmd_(i) = 0.0;
//                 qddot_cmd_(i) = 0.0;

//                 // Compute position error
//                 if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
//                 {
//                     angles::shortest_angular_distance_with_limits(
//                         q_(i),
//                         q_cmd_(i),
//                         joint_urdfs_[i]->limits->lower,
//                         joint_urdfs_[i]->limits->upper,
//                         q_error_(i));
//                 }
//                 else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
//                 {
//                     q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
//                 }
//                 else // prismatic
//                 {
//                     q_error_(i) = q_cmd_(i) - q_(i);
//                 }

//                 q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

//                 // friction compensation
//                 tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
//             }

//             // compute dynamics term
//             id_solver_->JntToMass(q_, M_);
//             id_solver_->JntToCoriolis(q_, qdot_, C_);
//             id_solver_->JntToGravity(q_, G_);

//             // torque command
//             GainsHandler::Gains gains = gains_handler_.getGains();
//             qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
//             qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

//             tau_cmd_.data = M_.data * qddot_ref_.data + C_.data + G_.data + tau_fric_.data;

//             // 单独设置关节3的力矩为5Nm
//             if (n_joints_ > 2)
//             {
//                 tau_cmd_(2) = 5.0; // 关节3的索引是2（从0开始）
//             }

//             // 计算最大力矩值的宽度（假设为2位小数）
//             const int value_width = 13; // 包含小数点和小数位
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

//                 // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
//                 if (i != 2)
//                 {
//                     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
//                 }

//                 // effort saturation
//                 if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

//                 if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

//                 joints_[i].setCommand(tau_cmd_(i));
//                 actual_torque = joints_[i].getEffort();
//                 // 只输出力矩值，使用固定宽度对齐
//                 std::cout << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << tau_cmd_(i)
//                           << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << actual_torque;
//             }
//             // // 可选：读取当前实际力矩值（如果硬件支持反馈）
//             //             double actual_torque = joints_.getEffort();
//             //             ROS_INFO_STREAM("Actual torque feedback: " << actual_torque << " Nm");
//             std::cout << std::endl; // 最后换行

//             // publish
//             if (loop_count_ % 10 == 0)
//             {
//                 if (controller_state_pub_->trylock())
//                 {
//                     controller_state_pub_->msg_.header.stamp = time;
//                     for (int i = 0; i < n_joints_; i++)
//                     {
//                         controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
//                         controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
//                         controller_state_pub_->msg_.state[i] = R2D * q_(i);
//                         controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
//                         controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
//                         controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
//                         controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
//                         controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
//                     }
//                     controller_state_pub_->unlockAndPublish();
//                 }
//             }
//             loop_count_++;
//         }

//         void stopping(const ros::Time &time) {}

//         void enforceJointLimits(double &command, unsigned int index)
//         {
//             // Check that this joint has applicable limits
//             if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
//             {
//                 if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
//                 {
//                     command = joint_urdfs_[index]->limits->upper;
//                 }
//                 else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
//                 {
//                     command = joint_urdfs_[index]->limits->lower;
//                 }
//             }
//         }

//     private:
//         int loop_count_;
//         double actual_torque;
//         // joint handles
//         unsigned int n_joints_;
//         std::vector<std::string> joint_names_;
//         std::vector<hardware_interface::JointHandle> joints_;
//         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

//         // kdl
//         KDL::Tree kdl_tree_;
//         KDL::Chain kdl_chain_;
//         boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
//         KDL::JntSpaceInertiaMatrix M_;
//         KDL::JntArray C_;
//         KDL::JntArray G_; // gravity torque vector
//         KDL::Vector gravity_;

//         // cmd, state
//         realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
//         KDL::JntArray tau_cmd_, tau_fric_;
//         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
//         KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
//         KDL::JntArray q_, qdot_;
//         KDL::JntArray q_error_, q_error_dot_;

//         // gain
//         GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
//         std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

//         // topic
//         ros::Subscriber command_sub_;
//         boost::scoped_ptr<
//             realtime_tools::RealtimePublisher<
//                 arm_controllers::ControllerJointState>>
//             controller_state_pub_;
//     };

// }

// PLUGINLIB_EXPORT_CLASS(arm_controllers::TorqueController, controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <control_toolbox/pid.h>
// #include <realtime_tools/realtime_buffer.h>

// #include <pluginlib/class_list_macros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <angles/angles.h>

// #include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include <boost/scoped_ptr.hpp>

// #include "arm_controllers/ControllerJointState.h"
// #include "arm_controllers/PassivityControllerParamsConfig.h"

// #include <iostream>
// #include <iomanip> // 引入setw和left所需的头文件

// // #include "elfin_hardware_interface/PosVelTrqJointInterface.h" // 引入新的硬件接口头文件

// #include "elfin_hardware_interface/posveltrq_command_interface.h"

// #define PI 3.141592
// #define D2R PI / 180.0
// #define R2D 180.0 / PI

// namespace arm_controllers
// {

//     class PassivityController : public controller_interface::Controller<elfin_hardware_interface::PosVelTrqJointInterface> // 修改控制器基类
//     {
//         class GainsHandler
//         {
//         public:
//             struct Gains
//             {
//                 Gains() : alpha_(0.0) {}
//                 Gains(double alpha) : alpha_(alpha) {}

//                 double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
//             };

//             GainsHandler() {}

//             bool initDynamicReconfig(const ros::NodeHandle &node)
//             {
//                 ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

//                 Gains gains;
//                 if (!node.getParam("alpha", gains.alpha_))
//                 {
//                     ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
//                     return false;
//                 }

//                 // Start dynamic reconfigure server
//                 param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
//                 dynamic_reconfig_initialized_ = true;

//                 setGains(gains);

//                 // Set Dynamic Reconfigure's gains to Pid's values
//                 updateDynamicReconfig();

//                 // Set callback
//                 param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
//                 param_reconfig_server_->setCallback(param_reconfig_callback_);

//                 return true;
//             }

//             void getGains(double &alpha)
//             {
//                 Gains gains = *gains_buffer_.readFromRT();
//                 alpha = gains.alpha_;
//             }

//             Gains getGains()
//             {
//                 return *gains_buffer_.readFromRT();
//             }

//             void setGains(double alpha)
//             {
//                 Gains gains(alpha);

//                 setGains(gains);
//             }

//             void setGains(const Gains &gains)
//             {
//                 gains_buffer_.writeFromNonRT(gains);

//                 updateDynamicReconfig(gains);
//             }

//             void updateDynamicReconfig()
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Get starting values
//                 PassivityControllerParamsConfig config;
//                 getGains(config.alpha);

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(Gains gains)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 PassivityControllerParamsConfig config;

//                 // Convert to dynamic reconfigure format
//                 config.alpha = gains.alpha_;

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(PassivityControllerParamsConfig config)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Set starting values, using a shared mutex with dynamic reconfig
//                 param_reconfig_mutex_.lock();
//                 param_reconfig_server_->updateConfig(config);
//                 param_reconfig_mutex_.unlock();
//             }

//             void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
//             {
//                 ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

//                 // Set the gains
//                 setGains(config.alpha);
//             }

//         private:
//             // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
//             // blocking the realtime update loop
//             realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

//             // Dynamics reconfigure
//             bool dynamic_reconfig_initialized_;
//             typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//             boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
//             DynamicReconfigServer::CallbackType param_reconfig_callback_;

//             boost::recursive_mutex param_reconfig_mutex_;
//         };

//     public:
//         ~PassivityController()
//         {
//             command_sub_.shutdown();
//         }

//         bool init(elfin_hardware_interface::PosVelTrqJointInterface *hw, ros::NodeHandle &n) // 修改硬件接口类型
//         {
//             loop_count_ = 0;
//             // List of controlled joints
//             if (!n.getParam("joints", joint_names_))
//             {
//                 ROS_ERROR("Could not find joint name");
//                 return false;
//             }
//             n_joints_ = joint_names_.size();

//             if (n_joints_ == 0)
//             {
//                 ROS_ERROR("List of joint names is empty.");
//                 return false;
//             }

//             // urdf
//             urdf::Model urdf;
//             if (!urdf.initParam("robot_description"))
//             {
//                 ROS_ERROR("Failed to parse urdf file");
//                 return false;
//             }

//             // joint handle
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 try
//                 {
//                     elfin_hardware_interface::PosVelTrqJointHandle handle = hw->getHandle(joint_names_[i]); // 获取新的关节句柄
//                     joints_.push_back(handle);
//                 }
//                 catch (const hardware_interface::HardwareInterfaceException &e)
//                 {
//                     ROS_ERROR_STREAM("Exception thrown: " << e.what());
//                     return false;
//                 }

//                 urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
//                 if (!joint_urdf)
//                 {
//                     ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
//                     return false;
//                 }
//                 joint_urdfs_.push_back(joint_urdf);
//             }

//             // kdl parser
//             if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
//             {
//                 ROS_ERROR("Failed to construct kdl tree");
//                 return false;
//             }

//             // kdl chain
//             std::string root_name, tip_name;
//             if (!n.getParam("root_link", root_name))
//             {
//                 ROS_ERROR("Could not find root link name");
//                 return false;
//             }
//             if (!n.getParam("tip_link", tip_name))
//             {
//                 ROS_ERROR("Could not find tip link name");
//                 return false;
//             }
//             if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
//             {
//                 ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
//                 ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
//                 ROS_ERROR_STREAM("  The segments are:");

//                 KDL::SegmentMap segment_map = kdl_tree_.getSegments();
//                 KDL::SegmentMap::iterator it;

//                 for (it = segment_map.begin(); it != segment_map.end(); it++)
//                     ROS_ERROR_STREAM("    " << (*it).first);

//                 return false;
//             }

//             gravity_ = KDL::Vector::Zero();
//             gravity_(2) = -9.81;
//             M_.resize(n_joints_);
//             C_.resize(n_joints_);
//             G_.resize(n_joints_);

//             // inverse dynamics solver
//             id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

//             // command and state
//             tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_error_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

//             // gains
//             if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
//             {
//                 ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
//                 return false;
//             }

//             pids_.resize(n_joints_);
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
//                 {
//                     ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
//                     return false;
//                 }
//             }

//             // command subscriber
//             commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
//             command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &PassivityController::commandCB, this);

//             // start realtime state publisher
//             controller_state_pub_.reset(
//                 new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

//             controller_state_pub_->msg_.header.stamp = ros::Time::now();
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.name.push_back(joint_names_[i]);
//                 controller_state_pub_->msg_.command.push_back(0.0);
//                 controller_state_pub_->msg_.command_dot.push_back(0.0);
//                 controller_state_pub_->msg_.state.push_back(0.0);
//                 controller_state_pub_->msg_.state_dot.push_back(0.0);
//                 controller_state_pub_->msg_.error.push_back(0.0);
//                 controller_state_pub_->msg_.error_dot.push_back(0.0);
//                 controller_state_pub_->msg_.effort_command.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedback.push_back(0.0);
//             }

//             return true;
//         }

//         void starting(const ros::Time &time)
//         {
//             // get joint positions
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 记录初始位置作为目标位置
//                 q_cmd_(i) = q_(i);
//             }

//             ROS_INFO("Starting Passivity Based Controller");
//         }

//         void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//         {
//             if (msg->data.size() != n_joints_)
//             {
//                 ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
//                 return;
//             }
//             commands_buffer_.writeFromNonRT(msg->data);
//         }

//         void update(const ros::Time &time, const ros::Duration &period)
//         {
//             std::vector<double> &commands = *commands_buffer_.readFromRT();
//             double dt = period.toSec();

//             // get joint states
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 所有关节目标位置设为初始位置（保持不动）
//                 qdot_cmd_(i) = 0.0;
//                 qddot_cmd_(i) = 0.0;

//                 // Compute position error
//                 if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
//                 {
//                     angles::shortest_angular_distance_with_limits(
//                         q_(i),
//                         q_cmd_(i),
//                         joint_urdfs_[i]->limits->lower,
//                         joint_urdfs_[i]->limits->upper,
//                         q_error_(i));
//                 }
//                 else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
//                 {
//                     q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
//                 }
//                 else // prismatic
//                 {
//                     q_error_(i) = q_cmd_(i) - q_(i);
//                 }

//                 q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

//                 // friction compensation
//                 tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
//             }

//             // compute dynamics term
//             id_solver_->JntToMass(q_, M_);
//             id_solver_->JntToCoriolis(q_, qdot_, C_);
//             id_solver_->JntToGravity(q_, G_);

//             // torque command
//             GainsHandler::Gains gains = gains_handler_.getGains();
//             qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
//             qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

//             tau_cmd_.data = M_.data * qddot_ref_.data + C_.data + G_.data + tau_fric_.data;

//             // 单独设置关节3的力矩为5Nm
//             if (n_joints_ > 2) {
//                 tau_cmd_(2) = 100.0; // 关节3的索引是2（从0开始）
//             }

//             // 计算最大力矩值的宽度（假设为2位小数）
//             const int value_width = 13; // 包含小数点和小数位
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

//                 // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
//                 if (i != 2) {
//                     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
//                 }

//                 // effort saturation
//                 if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

//                 if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

//                 joints_[i].setCommandTorque(tau_cmd_(i)); // 使用新的接口设置力矩命令

//                 // 只输出力矩值，使用固定宽度对齐
//                 std::cout << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << tau_cmd_(i);
//             }

//             std::cout << std::endl; // 最后换行

//             // publish
//             if (loop_count_ % 10 == 0)
//             {
//                 if (controller_state_pub_->trylock())
//                 {
//                     controller_state_pub_->msg_.header.stamp = time;
//                     for (int i = 0; i < n_joints_; i++)
//                     {
//                         controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
//                         controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
//                         controller_state_pub_->msg_.state[i] = R2D * q_(i);
//                         controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
//                         controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
//                         controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
//                         controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
//                         controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
//                     }
//                     controller_state_pub_->unlockAndPublish();
//                 }
//             }
//             loop_count_++;
//         }

//         void stopping(const ros::Time &time) {}

//         void enforceJointLimits(double &command, unsigned int index)
//         {
//             // Check that this joint has applicable limits
//             if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
//             {
//                 if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
//                 {
//                     command = joint_urdfs_[index]->limits->upper;
//                 }
//                 else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
//                 {
//                     command = joint_urdfs_[index]->limits->lower;
//                 }
//             }
//         }

//     private:
//         int loop_count_;

//         // joint handles
//         unsigned int n_joints_;
//         std::vector<std::string> joint_names_;
//         std::vector<elfin_hardware_interface::PosVelTrqJointHandle> joints_; // 修改关节句柄类型
//         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

//         // kdl
//         KDL::Tree kdl_tree_;
//         KDL::Chain kdl_chain_;
//         boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
//         KDL::JntSpaceInertiaMatrix M_;
//         KDL::JntArray C_;
//         KDL::JntArray G_; // gravity torque vector
//         KDL::Vector gravity_;

//         // cmd, state
//         realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
//         KDL::JntArray tau_cmd_, tau_fric_;
//         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
//         KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
//         KDL::JntArray q_, qdot_;
//         KDL::JntArray q_error_, q_error_dot_;

//         // gain
//         GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
//         std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

//         // topic
//         ros::Subscriber command_sub_;
//         boost::scoped_ptr<
//             realtime_tools::RealtimePublisher<
//                 arm_controllers::ControllerJointState>>
//             controller_state_pub_;
//     };

// }

// PLUGINLIB_EXPORT_CLASS(arm_controllers::PassivityController, controller_interface::ControllerBase)

// #include <controller_interface/controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <control_toolbox/pid.h>
// #include <realtime_tools/realtime_buffer.h>

// #include <pluginlib/class_list_macros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <angles/angles.h>

// #include <urdf/model.h>

// #include <kdl/tree.hpp>
// #include <kdl/kdl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include <boost/scoped_ptr.hpp>

// #include "arm_controllers/ControllerJointState.h"
// #include "arm_controllers/PassivityControllerParamsConfig.h"

// #include <iostream>
// #include <iomanip> // 引入setw和left所需的头文件

// #define PI 3.141592
// #define D2R PI / 180.0
// #define R2D 180.0 / PI

// namespace arm_controllers
// {

//     class PassivityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
//     {
//         class GainsHandler
//         {
//         public:
//             struct Gains
//             {
//                 Gains() : alpha_(0.0) {}
//                 Gains(double alpha) : alpha_(alpha) {}

//                 double alpha_; // it's only one gain, but  make it structure for unity with the controller having multiple gains
//             };

//             GainsHandler() {}

//             bool initDynamicReconfig(const ros::NodeHandle &node)
//             {
//                 ROS_INFO("Init dynamic reconfig in namespace %s", node.getNamespace().c_str());

//                 Gains gains;
//                 if (!node.getParam("alpha", gains.alpha_))
//                 {
//                     ROS_ERROR("Could not find gain %s", (node.getNamespace() + "/alpha").c_str());
//                     return false;
//                 }

//                 // Start dynamic reconfigure server
//                 param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
//                 dynamic_reconfig_initialized_ = true;

//                 setGains(gains);

//                 // Set Dynamic Reconfigure's gains to Pid's values
//                 updateDynamicReconfig();

//                 // Set callback
//                 param_reconfig_callback_ = boost::bind(&GainsHandler::dynamicReconfigCallback, this, _1, _2);
//                 param_reconfig_server_->setCallback(param_reconfig_callback_);

//                 return true;
//             }

//             void getGains(double &alpha)
//             {
//                 Gains gains = *gains_buffer_.readFromRT();
//                 alpha = gains.alpha_;
//             }

//             Gains getGains()
//             {
//                 return *gains_buffer_.readFromRT();
//             }

//             void setGains(double alpha)
//             {
//                 Gains gains(alpha);

//                 setGains(gains);
//             }

//             void setGains(const Gains &gains)
//             {
//                 gains_buffer_.writeFromNonRT(gains);

//                 updateDynamicReconfig(gains);
//             }

//             void updateDynamicReconfig()
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Get starting values
//                 PassivityControllerParamsConfig config;
//                 getGains(config.alpha);

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(Gains gains)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 PassivityControllerParamsConfig config;

//                 // Convert to dynamic reconfigure format
//                 config.alpha = gains.alpha_;

//                 updateDynamicReconfig(config);
//             }

//             void updateDynamicReconfig(PassivityControllerParamsConfig config)
//             {
//                 // Make sure dynamic reconfigure is initialized
//                 if (!dynamic_reconfig_initialized_)
//                     return;

//                 // Set starting values, using a shared mutex with dynamic reconfig
//                 param_reconfig_mutex_.lock();
//                 param_reconfig_server_->updateConfig(config);
//                 param_reconfig_mutex_.unlock();
//             }

//             void dynamicReconfigCallback(PassivityControllerParamsConfig &config, uint32_t /*level*/)
//             {
//                 ROS_DEBUG_STREAM_NAMED("passivity gain", "Dynamics reconfigure callback recieved.");

//                 // Set the gains
//                 setGains(config.alpha);
//             }

//         private:
//             // Store the gains in a realtime buffer to allow dynamic reconfigure to update it without
//             // blocking the realtime update loop
//             realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

//             // Dynamics reconfigure
//             bool dynamic_reconfig_initialized_;
//             typedef dynamic_reconfigure::Server<arm_controllers::PassivityControllerParamsConfig> DynamicReconfigServer;
//             boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
//             DynamicReconfigServer::CallbackType param_reconfig_callback_;

//             boost::recursive_mutex param_reconfig_mutex_;
//         };

//     public:
//         ~PassivityController()
//         {
//             command_sub_.shutdown();
//         }

//         bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
//         {
//             loop_count_ = 0;
//             // List of controlled joints
//             if (!n.getParam("joints", joint_names_))
//             {
//                 ROS_ERROR("Could not find joint name");
//                 return false;
//             }
//             n_joints_ = joint_names_.size();

//             if (n_joints_ == 0)
//             {
//                 ROS_ERROR("List of joint names is empty.");
//                 return false;
//             }

//             // urdf
//             urdf::Model urdf;
//             if (!urdf.initParam("robot_description"))
//             {
//                 ROS_ERROR("Failed to parse urdf file");
//                 return false;
//             }

//             // joint handle
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 try
//                 {
//                     joints_.push_back(hw->getHandle(joint_names_[i]));
//                 }
//                 catch (const hardware_interface::HardwareInterfaceException &e)
//                 {
//                     ROS_ERROR_STREAM("Exception thrown: " << e.what());
//                     return false;
//                 }

//                 urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
//                 if (!joint_urdf)
//                 {
//                     ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
//                     return false;
//                 }
//                 joint_urdfs_.push_back(joint_urdf);
//             }

//             // kdl parser
//             if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
//             {
//                 ROS_ERROR("Failed to construct kdl tree");
//                 return false;
//             }

//             // kdl chain
//             std::string root_name, tip_name;
//             if (!n.getParam("root_link", root_name))
//             {
//                 ROS_ERROR("Could not find root link name");
//                 return false;
//             }
//             if (!n.getParam("tip_link", tip_name))
//             {
//                 ROS_ERROR("Could not find tip link name");
//                 return false;
//             }
//             if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
//             {
//                 ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
//                 ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
//                 ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
//                 ROS_ERROR_STREAM("  The segments are:");

//                 KDL::SegmentMap segment_map = kdl_tree_.getSegments();
//                 KDL::SegmentMap::iterator it;

//                 for (it = segment_map.begin(); it != segment_map.end(); it++)
//                     ROS_ERROR_STREAM("    " << (*it).first);

//                 return false;
//             }

//             gravity_ = KDL::Vector::Zero();
//             gravity_(2) = -9.81;
//             M_.resize(n_joints_);
//             C_.resize(n_joints_);
//             G_.resize(n_joints_);

//             // inverse dynamics solver
//             id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

//             // command and state
//             tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             tau_fric_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_ref_.data = Eigen::VectorXd::Zero(n_joints_);
//             qddot_ref_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_.data = Eigen::VectorXd::Zero(n_joints_);
//             qdot_.data = Eigen::VectorXd::Zero(n_joints_);

//             q_error_.data = Eigen::VectorXd::Zero(n_joints_);
//             q_error_dot_.data = Eigen::VectorXd::Zero(n_joints_);

//             // gains
//             if (!gains_handler_.initDynamicReconfig(ros::NodeHandle(n, "gains/")))
//             {
//                 ROS_ERROR_STREAM("Failed to load alpha gain parameter from gains");
//                 return false;
//             }

//             pids_.resize(n_joints_);
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
//                 {
//                     ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
//                     return false;
//                 }
//             }

//             // command subscriber
//             commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
//             command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &PassivityController::commandCB, this);

//             // start realtime state publisher
//             controller_state_pub_.reset(
//                 new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

//             controller_state_pub_->msg_.header.stamp = ros::Time::now();
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.name.push_back(joint_names_[i]);
//                 controller_state_pub_->msg_.command.push_back(0.0);
//                 controller_state_pub_->msg_.command_dot.push_back(0.0);
//                 controller_state_pub_->msg_.state.push_back(0.0);
//                 controller_state_pub_->msg_.state_dot.push_back(0.0);
//                 controller_state_pub_->msg_.error.push_back(0.0);
//                 controller_state_pub_->msg_.error_dot.push_back(0.0);
//                 controller_state_pub_->msg_.effort_command.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
//                 controller_state_pub_->msg_.effort_feedback.push_back(0.0);
//             }

//             return true;
//         }

//         void starting(const ros::Time &time)
//         {
//             // get joint positions
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 记录初始位置作为目标位置
//                 q_cmd_(i) = q_(i);
//             }

//             ROS_INFO("Starting Passivity Based Controller");
//         }

//         void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
//         {
//             if (msg->data.size() != n_joints_)
//             {
//                 ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
//                 return;
//             }
//             commands_buffer_.writeFromNonRT(msg->data);
//         }

//         void update(const ros::Time &time, const ros::Duration &period)
//         {
//             std::vector<double> &commands = *commands_buffer_.readFromRT();
//             double dt = period.toSec();

//             // get joint states
//             for (size_t i = 0; i < n_joints_; i++)
//             {
//                 q_(i) = joints_[i].getPosition();
//                 qdot_(i) = joints_[i].getVelocity();

//                 // 所有关节目标位置设为初始位置（保持不动）
//                 qdot_cmd_(i) = 0.0;
//                 qddot_cmd_(i) = 0.0;

//                 // Compute position error
//                 if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
//                 {
//                     angles::shortest_angular_distance_with_limits(
//                         q_(i),
//                         q_cmd_(i),
//                         joint_urdfs_[i]->limits->lower,
//                         joint_urdfs_[i]->limits->upper,
//                         q_error_(i));
//                 }
//                 else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
//                 {
//                     q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
//                 }
//                 else // prismatic
//                 {
//                     q_error_(i) = q_cmd_(i) - q_(i);
//                 }

//                 q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

//                 // friction compensation
//                 tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
//             }

//             // compute dynamics term
//             id_solver_->JntToMass(q_, M_);
//             id_solver_->JntToCoriolis(q_, qdot_, C_);
//             id_solver_->JntToGravity(q_, G_);

//             // torque command
//             GainsHandler::Gains gains = gains_handler_.getGains();
//             qdot_ref_.data = qdot_cmd_.data + gains.alpha_ * q_error_.data;
//             qddot_ref_.data = qddot_cmd_.data + gains.alpha_ * q_error_dot_.data + q_error_.data;

//             tau_cmd_.data = M_.data * qddot_ref_.data + C_.data + G_.data + tau_fric_.data;

//             // 单独设置关节3的力矩为5Nm
//             if (n_joints_ > 2) {
//                 tau_cmd_(2) = 5.0; // 关节3的索引是2（从0开始）
//             }

//             // 计算最大力矩值的宽度（假设为2位小数）
//             const int value_width = 13; // 包含小数点和小数位
//             for (int i = 0; i < n_joints_; i++)
//             {
//                 controller_state_pub_->msg_.effort_feedforward[i] = tau_cmd_(i);

//                 // 只对关节3以外的轴应用PID控制（关节3使用纯力矩控制）
//                 if (i != 2) {
//                     tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);
//                 }

//                 // effort saturation
//                 if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

//                 if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
//                     tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

//                 joints_[i].setCommand(tau_cmd_(i));

//                 // 只输出力矩值，使用固定宽度对齐
//                 std::cout << std::right
//                           << std::setw(value_width)
//                           << std::fixed << std::setprecision(2) << tau_cmd_(i);
//             }

//             std::cout << std::endl; // 最后换行

//             // publish
//             if (loop_count_ % 10 == 0)
//             {
//                 if (controller_state_pub_->trylock())
//                 {
//                     controller_state_pub_->msg_.header.stamp = time;
//                     for (int i = 0; i < n_joints_; i++)
//                     {
//                         controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);
//                         controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);
//                         controller_state_pub_->msg_.state[i] = R2D * q_(i);
//                         controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);
//                         controller_state_pub_->msg_.error[i] = R2D * q_error_(i);
//                         controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);
//                         controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
//                         controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - controller_state_pub_->msg_.effort_feedforward[i];
//                     }
//                     controller_state_pub_->unlockAndPublish();
//                 }
//             }
//             loop_count_++;
//         }

//         void stopping(const ros::Time &time) {}

//         void enforceJointLimits(double &command, unsigned int index)
//         {
//             // Check that this joint has applicable limits
//             if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
//             {
//                 if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
//                 {
//                     command = joint_urdfs_[index]->limits->upper;
//                 }
//                 else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
//                 {
//                     command = joint_urdfs_[index]->limits->lower;
//                 }
//             }
//         }

//     private:
//         int loop_count_;

//         // joint handles
//         unsigned int n_joints_;
//         std::vector<std::string> joint_names_;
//         std::vector<hardware_interface::JointHandle> joints_;
//         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

//         // kdl
//         KDL::Tree kdl_tree_;
//         KDL::Chain kdl_chain_;
//         boost::scoped_ptr<KDL::ChainDynParam> id_solver_; // inverse dynamics solver
//         KDL::JntSpaceInertiaMatrix M_;
//         KDL::JntArray C_;
//         KDL::JntArray G_; // gravity torque vector
//         KDL::Vector gravity_;

//         // cmd, state
//         realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
//         KDL::JntArray tau_cmd_, tau_fric_;
//         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_, q_cmd_old_, qdot_cmd_old_;
//         KDL::JntArray qdot_ref_, qddot_ref_; // passivity-based control joint reference
//         KDL::JntArray q_, qdot_;
//         KDL::JntArray q_error_, q_error_dot_;

//         // gain
//         GainsHandler gains_handler_;             // alpha gain(dynamic reconfigured)
//         std::vector<control_toolbox::Pid> pids_; // Internal PID controllers in ros-control

//         // topic
//         ros::Subscriber command_sub_;
//         boost::scoped_ptr<
//             realtime_tools::RealtimePublisher<
//                 arm_controllers::ControllerJointState>>
//             controller_state_pub_;
//     };

// }

// PLUGINLIB_EXPORT_CLASS(arm_controllers::PassivityController, controller_interface::ControllerBase)
