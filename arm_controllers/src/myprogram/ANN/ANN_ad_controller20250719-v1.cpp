#include <pinocchio/fwd.hpp>
#include <boost/variant.hpp>
#include <ros/ros.h>

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

#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <cstdlib>

#include <boost/scoped_ptr.hpp>

// from computed torque clik
#include <boost/lexical_cast.hpp>
//
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
// #include <utils/skew_symmetric.h>
#include <vector>
#include <string>
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/triyi/catkin_ws/src/elfin_robot/elfin_description/urdf/elfin3.urdf"
#endif

#include <string>

namespace arm_controllers
{

  class AdmittanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    ~AdmittanceController()
    {
      sub_q_cmd_.shutdown();
      sub_forcetorque_sensor_.shutdown();
      if (data_file_.is_open())
      {
        data_file_.flush();
        data_file_.close();
        std::cout << "Data file closed************************************************" << std::endl;
      }
    }

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
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
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
          ROS_ERROR_STREAM("Exception thrown: " << e.what());
          return false;
        }
      }
      std::cout << "即将对参数进行初始化啦######################################" << std::endl;
      // 初始化变量
      count = 0.0;
      // 机器人臂长;
      l1 = 0.324;
      l2 = 0.184;
      // 速度场参数
      n.param("VD", VD, 0.05);
      n.param("xc", xc, 0.25);
      n.param("yc", yc, 0.25);
      n.param("r0", r0, 0.08);
      // VD = 0.02 * M_PI;
      // xc = 0.28;
      // yc = 0.28;
      // r0 = 0.08;
      epsilon1 = 0.00075;
      c0 = 250;
      k0 = 0.35;
      // 阻抗参数
      if (!n.getParam("Md_", Md_vector))
      {
        ROS_ERROR("Could not find Md_ parameter");
        return false;
      }
      if (Md_vector.size() != 4)
      {
        ROS_ERROR("Md_ parameter should have 2 elements");
        return false;
      }
      Md_ << Md_vector[0], Md_vector[1],
        Md_vector[2], Md_vector[3];

      if (!n.getParam("Bd_", Bd_vector))
      {
        ROS_ERROR("Could not find Bd_ parameter");
        return false;
      }
      if (Bd_vector.size() != 2)
      {
        ROS_ERROR("Bd_ parameter should have 2 elements");
        return false;
      }
      Bd_ << Bd_vector[0], Bd_vector[1];
      // Md_ << 4.0, 0.0,
      //   0.0, 4.0;
      // Bd_ << 8.0,
      //   8.0;

      // 滑膜参数

      if (!n.getParam("lambda1_", lambda1_vector))
      {
        ROS_ERROR("Could not find lambda1_ parameter");
        return false;
      }
      if (lambda1_vector.size() != 2)
      {
        ROS_ERROR("lambda1_ parameter should have 2 elements");
        return false;
      }
      lambda1_ << lambda1_vector[0], lambda1_vector[1];

      if (!n.getParam("lambda2_", lambda2_vector))
      {
        ROS_ERROR("Could not find lambda2_ parameter");
        return false;
      }
      if (lambda2_vector.size() != 2)
      {
        ROS_ERROR("lambda2_ parameter should have 2 elements");
        return false;
      }
      lambda2_ << lambda2_vector[0], lambda2_vector[1];

      if (!n.getParam("lambda3_", lambda3_vector))
      {
        ROS_ERROR("Could not find lambda3_ parameter");
        return false;
      }
      if (lambda3_vector.size() != 2)
      {
        ROS_ERROR("lambda3_ parameter should have 2 elements");
        return false;
      }
      lambda3_ << lambda3_vector[0], lambda3_vector[1];

      if (!n.getParam("fc_", fc_vector))
      {
        ROS_ERROR("Could not find fc_ parameter");
        return false;
      }
      if (fc_vector.size() != 2)
      {
        ROS_ERROR("fc_ parameter should have 2 elements");
        return false;
      }
      fc_ << fc_vector[0], fc_vector[1];

      if (!n.getParam("fv_", fv_vector))
      {
        ROS_ERROR("Could not find fv_ parameter");
        return false;
      }
      if (fv_vector.size() != 2)
      {
        ROS_ERROR("fv_ parameter should have 2 elements");
        return false;
      }
      fv_ << fv_vector[0], fv_vector[1];
      if (!n.getParam("fs_", fs_vector))
      {
        ROS_ERROR("Could not find fs_ parameter");
        return false;
      }
      if (fs_vector.size() != 2)
      {
        ROS_ERROR("fs_ parameter should have 2 elements");
        return false;
      }
      fs_ << fs_vector[0], fs_vector[1];
      if (!n.getParam("kf_", kf_vector))
      {
        ROS_ERROR("Could not find kf_ parameter");
        return false;
      }
      if (kf_vector.size() != 2)
      {
        ROS_ERROR("kf_ parameter should have 2 elements");
        return false;
      }
      kf_ << kf_vector[0], kf_vector[1];
      if (!n.getParam("wf_", wf_vector))
      {
        ROS_ERROR("Could not find wf_ parameter");
        return false;
      }
      if (wf_vector.size() != 2)
      {
        ROS_ERROR("wf_ parameter should have 2 elements");
        return false;
      }
      wf_ << wf_vector[0], wf_vector[1];
      if (!n.getParam("km_", km_vector))
      {
        ROS_ERROR("Could not find km_ parameter");
        return false;
      }
      if (km_vector.size() != 2)
      {
        ROS_ERROR("km_ parameter should have 2 elements");
        return false;
      }
      km_ << km_vector[0], km_vector[1];

      if (!n.getParam("w1_", w1_vector))
      {
        ROS_ERROR("Could not find w1_ parameter");
        return false;
      }
      if (w1_vector.size() != 2)
      {
        ROS_ERROR("w1_ parameter should have 2 elements");
        return false;
      }
      w1_ << w1_vector[0], w1_vector[1];

      if (!n.getParam("gamma1_", gamma1_vector))
      {
        ROS_ERROR("Could not find gamma1_ parameter");
        return false;
      }
      if (gamma1_vector.size() != 2)
      {
        ROS_ERROR("gamma1_ parameter should have 2 elements");
        return false;
      }
      gamma1_ << gamma1_vector[0], gamma1_vector[1];

      if (!n.getParam("miu_", miu_vector))
      {
        ROS_ERROR("Could not find miu_ parameter");
        return false;
      }
      if (miu_vector.size() != 2)
      {
        ROS_ERROR("miu_ parameter should have 2 elements");
        return false;
      }
      miu_ << miu_vector[0], miu_vector[1];

      if (!n.getParam("eta_", eta_vector))
      {
        ROS_ERROR("Could not find eta_ parameter");
        return false;
      }
      if (eta_vector.size() != 2)
      {
        ROS_ERROR("eta_ parameter should have 2 elements");
        return false;
      }
      eta_ << eta_vector[0], eta_vector[1];

      if (!n.getParam("k1_", k1_vector))
      {
        ROS_ERROR("Could not find k1_ parameter");
        return false;
      }
      if (k1_vector.size() != 2)
      {
        ROS_ERROR("k1_ parameter should have 2 elements");
        return false;
      }
      k1_ << k1_vector[0], k1_vector[1];

      if (!n.getParam("ki_", ki_vector)) {
          ROS_ERROR("Could not find ki_ parameter");
          return false;
      }
      if (ki_vector.size() != 2) {
          ROS_ERROR("ki_ parameter should have 2 elements");
          return false;
      }
      ki_ << ki_vector[0], ki_vector[1];

      if (!n.getParam("kp_", kp_vector)) {
          ROS_ERROR("Could not find kp_ parameter");
          return false;
      }
      if (kp_vector.size() != 2) {
          ROS_ERROR("kp_ parameter should have 2 elements");
          return false;
      }
      kp_ << kp_vector[0], kp_vector[1];

      if (!n.getParam("epsilon_N_", epsilon_N_vector))
      {
        ROS_ERROR("Could not find epsilon_N_ parameter");
        return false;
      }
      if (epsilon_N_vector.size() != 2)
      {
        ROS_ERROR("epsilon_N_ parameter should have 2 elements");
        return false;
      }
      epsilon_N_ << epsilon_N_vector[0], epsilon_N_vector[1];

      if (!n.getParam("c1_", c1_vector)) {
          ROS_ERROR("Could not find c1_ parameter");
          return false;
      }
      if (c1_vector.size() != 7) {
          ROS_ERROR("c1_ parameter should have 2 elements");
          return false;
      }
      c1_ << c1_vector[0], c1_vector[1], c1_vector[2], c1_vector[3], c1_vector[4], c1_vector[5], c1_vector[6], c1_vector[0], c1_vector[1], c1_vector[2], c1_vector[3], c1_vector[4], c1_vector[5], c1_vector[6], c1_vector[0], c1_vector[1], c1_vector[2], c1_vector[3], c1_vector[4], c1_vector[5], c1_vector[6], c1_vector[0], c1_vector[1], c1_vector[2], c1_vector[3], c1_vector[4], c1_vector[5], c1_vector[6];

      if (!n.getParam("c2_", c2_vector)) {
          ROS_ERROR("Could not find c2_ parameter");
          return false;
      }
      if (c2_vector.size() != 7) {
          ROS_ERROR("c2_ parameter should have 2 elements");
          return false;
      }
      c2_ << c2_vector[0], c2_vector[1], c2_vector[2], c2_vector[3], c2_vector[4], c2_vector[5], c2_vector[6], c2_vector[0], c2_vector[1], c2_vector[2], c2_vector[3], c2_vector[4], c2_vector[5], c2_vector[6], c2_vector[0], c2_vector[1], c2_vector[2], c2_vector[3], c2_vector[4], c2_vector[5], c2_vector[6], c2_vector[0], c2_vector[1], c2_vector[2], c2_vector[3], c2_vector[4], c2_vector[5], c2_vector[6];

      if (!n.getParam("l_", l_vector)) {
          ROS_ERROR("Could not find l_ parameter");
          return false;
      }
      if (l_vector.size() != 7) {
          ROS_ERROR("l_ parameter should have 2 elements");
          return false;
      }
      l_ << l_vector[0], l_vector[1],l_vector[2], l_vector[3],l_vector[4], l_vector[5],l_vector[6];
      // km_ << 0.5,
      //   0.5;
      // w1_ << 9.0,
      //   9.0;
      // gamma1_ << 5.0,
      //   5.0;
      // miu_ << 0.03,
      //   0.03;
      // eta_ << 2.0,
      //   2.0;
      k3_ << 2.0,
        2.0;
      epsilon_ << 0.25,
        0.25;
      beta0_ << 4.0,
        4.0;
      // 控制律限制
      min_val_ = { {-15.0, -20.0, -20.0, -3.0, -10.0, -3.0} }; // 下限
      max_val_ = { {15.0, 20.0, 20.0, 3.0, 10.0, 3.0} };       // 上限
      q_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
      dq_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
      q_init_ = { {0.0, 0.0, M_PI_2, M_PI_2, 0.0, 0.0} };
      tau_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
      tau1_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
      qd_ << -0.8159,
        0.6772;
      // dqd_ << 0.0,
      //   0.0;
      dqd_ << -0.287574,
        -0.448394;
      Part_of_Integral_ << 0.0,
        0.0;
      // k1_ << 3.0,
      //   3.0;
      g_ << 0.0, 0.0;
      w_hat1_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      w_hat2_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      // 虚拟墙
      K = 0.10;
      B = 0.20;
      delta = M_PI / 1800;
      ke_ = { {2000.0, 3000.0, 3000.0, 2000.0, 2000.0, 2000.0} };
      be_ = { {100.0, 100.0, 100.0, 100.0, 100.0, 100.0} };

      std::cout << "准备构建URDF模型#################################################" << std::endl;
      // urdf
      const std::string urdf_filename = PINOCCHIO_MODEL_DIR;
      pinocchio::urdf::buildModel(urdf_filename, model);
      data = pinocchio::Data(model);

      q_m_ << q_[0], 0.0, M_PI_2, M_PI_2, q_[4], 0.0;
      dq_m_ << dq_[0], 0.0, 0.0, 0.0, dq_[4], 0.0;
      ddq_m_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      tau_m_ << tau_[0], 0.0, 0.0, 0.0, tau_[4], 0.0;

      sub_q_cmd_ = n.subscribe("command", 1, &AdmittanceController::commandCB, this);

      sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/ft_data_raw", 1, &AdmittanceController::updateFTsensor, this);

      // 打开文件
      // data_file_.open("/home/triyi/catkin_ws/src/arm_controllers/src/myprogram/output_data.txt");
      // if (!data_file_.is_open())
      // {
      //   ROS_ERROR("Failed to open the data file.");
      //   return false;
      // }
      output_file_path_ = "/home/triyi/catkin_ws/src/arm_controllers/src/myprogram/output_data.csv";
      data_file_.open(output_file_path_, std::ios::out | std::ios::trunc);
      if (!data_file_.is_open())
      {
        ROS_ERROR_STREAM("Failed to open data file: " << output_file_path_);
        return false;
      }
      // 写入文件头
      data_file_ << "Time,e_0,e_1,s_0,s_1,k1_0,k1_1,q_0,q_4,dq_0,dq_4,dqr_0,dqr_1,dqd_0,dqd_1,ddqd_0,ddqd_1,tau_0,tau_4,Fext_0,Fext_1" << std::endl;

      // data_file_
      //   << t << ", "
      //   << e_(0) << ", "
      //   << e_(1) << ", "
      //   << s_(0) << ", "
      //   << s_(1) << ", "
      //   << k1_(0) << ", "
      //   << k1_(1) << ", "
      //   << q_[0] << ", "
      //   << q_[4] << ", "
      //   << dq_[0] << ", "
      //   << dq_[4] << ", "
      //   << dqr_(0) << ", "
      //   << dqr_(1) << ", "
      //   << dqd_(0) << ", "
      //   << dqd_(1) << ", "
      //   << ddqd_(0) << ", "
      //   << ddqd_(1) << ", "
      //   << tau_cmd_[0] << ", "
      //   << tau_cmd_[4] << ", "
      //   << Fext_(0) << ", "
      //   << Fext_(1) << std::endl;
      return true;
    } // end of initialize

    void starting(const ros::Time& time)
    {
      // get joint positions
      std::cout << "即将激活update###################################################" << std::endl;
      for (size_t i = 0; i < n_joints; i++)
      {
        q_[i] = joints_[i].getPosition();
        dq_[i] = joints_[i].getVelocity();
      }

      t = 0.0;

      // ROS_INFO("Starting Adaptive Impedance Controller");
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
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
    // void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    // {
    //   geometry_msgs::Wrench f_meas = msg->wrench;

    //   f_cur_[0] = f_meas.force.x;
    //   f_cur_[1] = f_meas.force.y;
    //   f_cur_[2] = f_meas.force.z;
    //   f_cur_[3] = f_meas.torque.x;
    //   f_cur_[4] = f_meas.torque.y;
    //   f_cur_[5] = f_meas.torque.z;
    // }
    // void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    // {
    //   /* ---------- 原始读数 ---------- */
    //   const auto& w = msg->wrench;
    //   double raw[6] = { w.force.x, w.force.y, w.force.z,
    //                    w.torque.x, w.torque.y, w.torque.z };

    //   /* ---------- (A) 标定阶段：累计零偏 ---------- */
    //   if (!ft_offset_ready_)
    //   {
    //     for (size_t i = 0; i < 6; ++i)
    //       f_offset_acc_[i] += raw[i];

    //     /* 收够样本后求平均零点并锁定 */
    //     if (++ft_offset_cnt_ >= FT_OFFSET_SAMPLES)
    //     {
    //       for (size_t i = 0; i < 6; ++i)
    //         f_offset_[i] = f_offset_acc_[i] / static_cast<double>(FT_OFFSET_SAMPLES);
    //       ft_offset_ready_ = true;
    //       ROS_INFO_STREAM("[FT] Zero-offset calibrated (samples = "
    //         << FT_OFFSET_SAMPLES << ")");
    //     }
    //     return; // 标定期先退出，不更新 f_cur_

    //   }
    //   /* ---------- (B) 工作阶段：扣除零偏 ---------- */
    //   for (size_t i = 0; i < 6; ++i) {
    //     // f_offset_ = { {22.2, -3.7, -18.0, 0.32, -0.44, -0.34} };
    //     f_cur_[i] = raw[i] - f_offset_[i];
    //   }
    // }

    void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {
      /* ---------- 原始读数 ---------- */
      const auto& w = msg->wrench;
      const double raw[6] = { w.force.x,  w.force.y,  w.force.z,
                              w.torque.x, w.torque.y, w.torque.z };

      /* ---------- (A) 开机标定阶段 ---------- */
      if (!ft_offset_ready_)
      {
        for (size_t i = 0; i < 6; ++i) f_offset_acc_[i] += raw[i];

        if (++ft_offset_cnt_ >= FT_OFFSET_SAMPLES)
        {
          for (size_t i = 0; i < 6; ++i)
            f_offset_[i] = f_offset_acc_[i] / static_cast<double>(FT_OFFSET_SAMPLES);
          ft_offset_ready_ = true;
          ROS_INFO_STREAM("[FT] Zero-offset calibrated (samples = "
            << FT_OFFSET_SAMPLES << ")");
        }
        return;                                   // 标定期不向下传递数据
      }

      /* ---------- (B) 在线零点自适应 ---------- */
      for (size_t i = 0; i < 6; ++i)
      {
        const double delta = raw[i] - f_offset_[i];

        /* 判断是否处于“松手/无接触”状态 */
        if (std::fabs(delta) < FT_REST_THRESHOLD)
        {
          /* 以极小步长把零偏慢慢拉向当前值（对温漂、蠕变有效） */
          f_offset_[i] += FT_OFFSET_ADAPT_GAIN * delta;
        }

        /* ---------- (C) 去零 & 1-阶低通 ---------- */
        const double uncompensated = raw[i] - f_offset_[i];
        f_cur_lpf_[i] = LPF_ALPHA * uncompensated + (1.0 - LPF_ALPHA) * f_cur_lpf_[i];
        f_cur_[i] = f_cur_lpf_[i];            // 下游控制器继续使用 f_cur_
      }
    }


    double sign(double x)
    {
      return (x > 0) - (x < 0);
    }

    Eigen::Matrix<double, 2, 1> get_dqr(double q1, double q2, double l1, double l2, double xc,
      double yc, double r0, double k0, double epsilon1, double VD, double c0,
      Eigen::Matrix<double, 2, 2> J_INV_)
    {
      // 1、定义变量
      double x, y, u, k, c;
      Eigen::Matrix<double, 2, 1> A_, B_, du_, v_, dqr_;

      // 2、正运动学，计算笛卡尔空间下轨迹
      x = l1 * cos(q1) + l2 * cos(q1 + q2); // 末端轨迹横坐标
      y = l1 * sin(q1) + l2 * sin(q1 + q2); // 末端轨迹纵坐标

      // 3、计算中间矩阵A,B_
      A_ << x - xc,
        y - yc;
      B_ << -(y - yc),
        x - xc;

      // 4、计算中间变量u,du_,k,c
      u = (x - xc) * (x - xc) + (y - yc) * (y - yc) - r0 * r0;
      du_ = 2.0 * A_;
      k = k0 / (fabs(u) * sqrt(du_(0) * du_(0) + du_(1) * du_(1)) + epsilon1);
      c = VD * exp((-1.0) * c0 * fabs(u)) / sqrt(du_(0) * du_(0) + du_(1) * du_(1));

      // 5、计算笛卡尔空间下的末端速度
      v_(0) = -2.0 * k * u * A_(0) + 2.0 * c * B_(0);
      v_(1) = -2.0 * k * u * A_(1) + 2.0 * c * B_(1);

      // 6、转关节空间
      dqr_(0) = J_INV_(0, 0) * v_(0) + J_INV_(0, 1) * v_(1);
      dqr_(1) = J_INV_(1, 0) * v_(0) + J_INV_(1, 1) * v_(1);
      return dqr_;
    }

    Eigen::Matrix<double, 2, 1> get_dk1(double s1, double s2, double k1_1, double k1_2,
      Eigen::Matrix<double, 2, 1> km_, Eigen::Matrix<double, 2, 1> w1_,
      Eigen::Matrix<double, 2, 1> gamma1_, Eigen::Matrix<double, 2, 1> eta_,
      Eigen::Matrix<double, 2, 1> miu_)
    {
      // 1、定义变量
      Eigen::Matrix<double, 2, 1> dk1;
      // 2、两个轴分开计算，先算第一个轴:dk1(0)
      // 判断k1是否超限
      if (k1_1 > km_(0))
      {
        dk1(0) = w1_(0) * sqrt(gamma1_(0) / 2.0) * sign(fabs(s1) - miu_(0));
      }
      else
      {
        // k1太小，则以eta为加速度增大k1
        dk1(0) = eta_(0);
      }

      // 3、同理，计算第二个轴:dk1(1)
      if (k1_2 > km_(1))
      {
        dk1(1) = w1_(1) * sqrt(gamma1_(1) / 2.0) * sign(fabs(s2) - miu_(1));
      }
      else
      {
        // k1太小，则以eta为加速度增大k1
        dk1(1) = eta_(1);
      }
      return dk1;
    }

    Eigen::Matrix<double, 2, 1> get_dPart_of_Integral(double e1, double e2)
    {
      Eigen::Matrix<double, 2, 1> dPart_of_Integral_;
      dPart_of_Integral_(0) = pow(fabs(e1), 1.0 / 3.0) * sign(e1);
      dPart_of_Integral_(1) = pow(fabs(e2), 1.0 / 3.0) * sign(e2);
      return dPart_of_Integral_;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      T = period.toSec();
      t += T;

      // 1.获取机器人信息
      for (size_t i = 0; i < n_joints; i++)
      {
        q_[i] = joints_[i].getPosition();
        dq_[i] = joints_[i].getVelocity();
      }

      // 先计算雅可比矩阵
      J_(0, 0) = -l1 * sin(q_[0]) - l2 * sin(q_[0] + q_[4]);
      J_(0, 1) = -l2 * sin(q_[0] + q_[4]);
      J_(1, 0) = l1 * cos(q_[0]) + l2 * cos(q_[0] + q_[4]);
      J_(1, 1) = l2 * cos(q_[0] + q_[4]);
      // 再计算雅可比矩阵的逆
      J_INV_ = J_.inverse();
      // 再计算速度场参考角速度：dqr
      dqr_ = get_dqr(q_[0], q_[4], l1, l2, xc, yc, r0, k0, epsilon1, VD, c0, J_INV_);
      if (t == T)
      {
        dqr_last_(0) = dqr_(0);
        dqr_last_(1) = dqr_(1);
      }
      ddqr_(0) = (dqr_(0) - dqr_last_(0)) / T;
      ddqr_(1) = (dqr_(1) - dqr_last_(1)) / T;
      // 存储dqr
      dqr_last_(0) = dqr_(0);
      dqr_last_(1) = dqr_(1);
      // 外力在xOz平面上
      Fext_(0) = (-1) * f_cur_[0];
      Fext_(1) = (-1) * f_cur_[2];

      // 过滤噪声
      // if (fabs(Fext_(0)) <= 0.5) {
      //   Fext_(0) = 0.0;
      // }
      // else {
      //   if (Fext_(0) > 0) {
      //     Fext_(0) -= 0.5;
      //   }
      //   else {
      //     Fext_(0) += 0.5;
      //   }
      // }

      // if (fabs(Fext_(1)) <= 0.5) {
      //   Fext_(1) = 0.0;
      // }
      // else {
      //   if (Fext_(1) > 0) {
      //     Fext_(1) -= 0.5;
      //   }
      //   else {
      //     Fext_(1) += 0.5;
      //   }
      // }

      // 输出 f_cur_[0] 和 f_cur_[1] 的数据
      // ROS_INFO("f_cur_[0]: %f, f_cur_[1]: %f", f_cur_[0], f_cur_[1]);

      // F_Joint_(0) = -J_(0, 0) * Fext_(0) + J_(1, 0) * Fext_(1);
      // F_Joint_(1) = -J_(0, 1) * Fext_(0) + J_(1, 1) * Fext_(1);

      F_Joint_(0) = J_(0, 0) * Fext_(0) + J_(1, 0) * Fext_(1);
      F_Joint_(1) = J_(0, 1) * Fext_(0) + J_(1, 1) * Fext_(1);


      // 计算Md的逆
      Md_INV_ = Md_.inverse();

      // 导纳模型计算ddqd
      // ddqd_(0) = Md_INV_(0, 0) * (F_Joint_(0) - Bd_(0) * (dqd_(0) - dqr_(0))) + Md_INV_(0, 1) * (F_Joint_(1) - Bd_(1) * (dqd_(1) - dqr_(1))) + ddqr_(0);
      // ddqd_(1) = Md_INV_(1, 0) * (F_Joint_(0) - Bd_(0) * (dqd_(0) - dqr_(0))) + Md_INV_(1, 1) * (F_Joint_(1) - Bd_(1) * (dqd_(1) - dqr_(1))) + ddqr_(1);
      ddqd_(0) = Md_INV_(0, 0) * (F_Joint_(0) - Bd_(0) * (dqd_(0) - dqr_(0))) + ddqr_(0);
      ddqd_(1) = Md_INV_(1, 1) * (F_Joint_(1) - Bd_(1) * (dqd_(1) - dqr_(1))) + ddqr_(1);


      // ddqd_(0) = ddqr_(0);
      // ddqd_(1) = ddqr_(1);

      dqd_(0) += ddqd_(0) * T;
      dqd_(1) += ddqd_(1) * T;
      // dqd_(0) = 1.0/Bd_(0) * (F_Joint_(0) - Md_(0, 0) * (ddqd_(0) - ddqr_(0))) + dqr_(0);
      // dqd_(1) = 1.0/Bd_(1) * (F_Joint_(1) - Md_(1, 1) * (ddqd_(1) - ddqr_(1))) + dqr_(1);


      // dqd_(0) = dqr_(0);
      // dqd_(1) = dqr_(1);
      // qd_(0) += dqd_(0) * T;
      // qd_(1) += dqd_(1) * T;


      e_(0) = dqd_(0) - dq_[0];
      e_(1) = dqd_(1) - dq_[4];

      q_m_ << q_[0], 0.0, M_PI_2, M_PI_2, q_[4], 0.0;
      dq_m_ << dq_[0], 0.0, 0.0, 0.0, dq_[4], 0.0;
      pinocchio::crba(model, data, q_m_);
      M_ << data.M(0, 0), data.M(0, 4),
        data.M(4, 0), data.M(4, 4);
      pinocchio::computeCoriolisMatrix(model, data, q_m_, dq_m_);
      C_ << data.C(0, 0), data.C(0, 4),
        data.C(4, 0), data.C(4, 4);
      pinocchio::computeGeneralizedGravity(model, data, q_m_);
      G_ << data.g(0), data.g(1), data.g(2), data.g(3), data.g(4), data.g(5);

      in1_ << ddqd_(0), dqd_(0), q_[0], dq_[0];
      in2_ << ddqd_(1), dqd_(1), q_[4], dq_[4];
      for (size_t i = 0; i < 7; i++) {
          // 提取c1_的第i列
          col1_ = c1_.col(i);
          col2_ = c2_.col(i);
          // 计算范数
          norm_val1 = (in1_ - col1_).norm();
          norm_val2 = (in2_ - col2_).norm();
          // 计算指数部分
          exponent1 = -1 * std::pow(norm_val1, 2) / (2 * std::pow(b_(i), 2));
          exponent2 = -1 * std::pow(norm_val2, 2) / (2 * std::pow(b_(i), 2));
          // 计算最终结果
          h1_(i) = std::exp(exponent1);
          h2_(i) = std::exp(exponent2);
      }

      for (size_t i = 0; i < 7; i++) {
          dw_hat1_(i) = e_(0) * l_(i) * h1_(i);
          dw_hat2_(i) = e_(1) * l_(i) * h2_(i);
      }
      for (size_t i = 0; i < 7; i++) {
          w_hat1_(i) += dw_hat1_(i) * T;
          w_hat2_(i) += dw_hat2_(i) * T;
      }

      N_hat_(0) = w_hat1_(0) * h1_(0) + w_hat1_(1) * h1_(1) + w_hat1_(2) * h1_(2) + w_hat1_(3) * h1_(3) + w_hat1_(4) * h1_(4) + w_hat1_(5) * h1_(5) + w_hat1_(6) * h1_(6);
      N_hat_(1) = w_hat2_(0) * h2_(0) + w_hat2_(1) * h2_(1) + w_hat2_(2) * h2_(2) + w_hat2_(3) * h2_(3) + w_hat2_(4) * h2_(4) + w_hat2_(5) * h2_(5) + w_hat2_(6) * h2_(6);

      // tau=N_hat+kp.*tanh(e)+epsilon_N.*sign(e);
      tau_[0] = N_hat_(0) + kp_(0) * tanh(e_(0)) + epsilon_N_(0) * sign(e_(0));
      tau_[4] = N_hat_(1) + kp_(1) * tanh(e_(1)) + epsilon_N_(1) * sign(e_(1));
      //  给其他轴施加虚拟墙。
      //     实现步骤：
      //         获取其他关节的目标关节角度q07x1
      // q_init_
      //         根据公式:tau1_=-ke_((q_-q0)-delta),|q_-q0|>delta计算虚拟墙的力tau1
      // 不同的轴分开：轴0到轴6
      if (fabs(q_[0] - q_init_[0]) > delta)
      {
        tau1_[0] = (-1) * be_[0] * tanh(B * dq_[0]) - ke_[0] * tanh(K * ((q_[0] - q_init_[0]) - delta));
      } // 轴0
      if (fabs(q_[1] - q_init_[1]) > delta)
      {
        tau1_[1] = (-1) * be_[1] * tanh(B * dq_[1]) - ke_[1] * tanh(K * ((q_[1] - q_init_[1]) - delta));
      } // 轴1
      if (fabs(q_[2] - q_init_[2]) > delta)
      {
        tau1_[2] = (-1) * be_[2] * tanh(B * dq_[2]) - ke_[2] * tanh(K * ((q_[2] - q_init_[2]) - delta));
      } // 轴2
      if (fabs(q_[3] - q_init_[3]) > delta)
      {
        tau1_[3] = (-1) * be_[3] * tanh(B * dq_[3]) - ke_[3] * tanh(K * ((q_[3] - q_init_[3]) - delta));
      } // 轴3
      if (fabs(q_[4] - q_init_[4]) > delta)
      {
        tau1_[4] = (-1) * be_[4] * tanh(B * dq_[4]) - ke_[4] * tanh(K * ((q_[4] - q_init_[4]) - delta));
      } // 轴4
      if (fabs(q_[5] - q_init_[5]) > delta)
      {
        tau1_[5] = (-1) * be_[5] * tanh(B * dq_[5]) - ke_[5] * tanh(K * ((q_[5] - q_init_[5]) - delta));
      } // 轴5
      // 加入到控制率tau中即可
      // 再加上重力补偿项
      tau_[1] = tau1_[1] + G_(1);
      tau_[2] = tau1_[2] + G_(2);
      tau_[3] = tau1_[3] + G_(3);
      tau_[5] = tau1_[5] + G_(5);

      tau_cmd_ = tau_;
      // 对扭矩命令进行限幅
      for (size_t i = 0; i < n_joints; i++)
      {
        tau_cmd_[i] = std::max(min_val_[i], std::min(max_val_[i], tau_cmd_[i]));
      }
      for (size_t i = 0; i < n_joints; i++)
      {
        joints_[i].setCommand(tau_cmd_[i]);
      }

      // // 设置输出格式：固定小数位+宽度对齐
      // std::cout << std::fixed << std::setprecision(4); // 保留4位小数
      // std::cout << "M_: ["
      //           << std::setw(10) << M_(0, 0) << ", " // 宽度10，右对齐
      //           << std::setw(10) << M_(0, 1) << "; "
      //           << std::setw(10) << M_(1, 0) << ", "
      //           << std::setw(10) << M_(1, 1) << "]  |  "
      //           << "C_: ["
      //           << std::setw(10) << C_(0, 0) << ", "
      //           << std::setw(10) << C_(0, 1) << "; "
      //           << std::setw(10) << C_(1, 0) << ", "
      //           << std::setw(10) << C_(1, 1) << "]  |  "
      //           << "G_: ["
      //           << std::setw(10) << G_(0) << ", "
      //           << std::setw(10) << G_(1) << "]"
      //           << std::endl; // 换行

      // // 恢复默认格式（可选，避免影响其他输出）
      // std::cout.unsetf(std::ios::fixed);

      // 设置输出格式：固定小数位+宽度对齐

      std::cout << std::fixed << std::setprecision(4); // 保留4位小数

      if (count < 10)
      {
        count++;
      }
      else
      {
        std::cout
          // << std::setw(10) << lambda1_(0) << ", "
          //   << std::setw(10) << lambda1_(1) << ", "
          //   << std::setw(10) << lambda2_(0) << ", "
          //   << std::setw(10) << lambda2_(1) << ", "
          //   << std::setw(10) << lambda3_(0) << ", "
          //   << std::setw(10) << lambda3_(1) << ", "
          << std::setw(10) << dqr_(0) << ", "
          << std::setw(10) << dqr_(1) << ", "
          //   << std::setw(10) << ddqr_(0) << ", "
          //   << std::setw(10) << ddqr_(1) << ", "
          //   << std::setw(10) << F_Joint_(0) << ", "
          //   << std::setw(10) << F_Joint_(1) << ", "
          //   << std::setw(10) << J_(0, 0) << ", "
          //   << std::setw(10) << J_(0, 1) << ", "
          //   << std::setw(10) << J_(1, 0) << ", "
          //   << std::setw(10) << J_(1, 1) << ", "
          // << std::setw(10) << Md_INV_(0, 0) << ", "
          // << std::setw(10) << Md_INV_(0, 1) << ", "
          // << std::setw(10) << Md_INV_(1, 0) << ", "
          // << std::setw(10) << Md_INV_(1, 1) << ", "
          << std::setw(10) << dqd_(0) << ", "
          << std::setw(10) << dqd_(1) << ", "
          // << std::setw(10) << ddqd_(0) << ", "
          // << std::setw(10) << ddqd_(1) << ", "
          // << std::setw(10) << dq_[0] << ", "
          // << std::setw(10) << dq_[4] << ", "
          // << std::setw(10) << e_(0) << ", "
          // << std::setw(10) << e_(1) << ", "
          // << std::setw(10) << tau_[0] << ", "
          // << std::setw(10) << tau_[4] << ", "
          << std::setw(10) << Fext_(0) << ", "
          << std::setw(10) << Fext_(1)
          << std::endl; // 换行
        count = 0;
      }

      // 恢复默认格式（可选）
      std::cout.unsetf(std::ios::fixed);

      // 判断是否运行到20s
      if (t <= 20.0)
      {

        // 将数据写入文件
        data_file_
          << t << ", "
          << e_(0) << ", "
          << e_(1) << ", "
          << s_(0) << ", "
          << s_(1) << ", "
          << k1_(0) << ", "
          << k1_(1) << ", "
          << q_[0] << ", "
          << q_[4] << ", "
          << dq_[0] << ", "
          << dq_[4] << ", "
          << dqr_(0) << ", "
          << dqr_(1) << ", "
          << dqd_(0) << ", "
          << dqd_(1) << ", "
          << ddqd_(0) << ", "
          << ddqd_(1) << ", "
          << tau_cmd_[0] << ", "
          << tau_cmd_[4] << ", "
          << Fext_(0) << ", "
          << Fext_(1) << std::endl;
        data_file_.flush();
        // "Time,e_0,e_1,s_0,s_1,q_0,q_4,dqr_0,dqr_1,dqd_0,dqd_1,ddqd_0,ddqd_1,tau_0,tau_4,Fext_0,Fext_1"
      }
      else
      {
        if (data_file_.is_open())
        {
          data_file_.flush();
          data_file_.close();
          ROS_INFO_STREAM("Data saved to: " << output_file_path_);
        }
        ros::shutdown(); // 关闭ROS节点，停止控制器
        return;
      }
    }

    void stopping(const ros::Time& time)
    {
      if (data_file_.is_open())
      {
        data_file_.flush();
        data_file_.close();
        ROS_INFO_STREAM("Data saved to: " << output_file_path_);
      }
    }

  private:
    std::vector<std::string> joint_names_;
    unsigned int n_joints;
    std::vector<hardware_interface::JointHandle> joints_;

    double t, T, l1, l2, xc, yc, r0, k0, epsilon1, VD, c0, B, K, delta, count;
    std::array<double, 6> q_, dq_, q_init_;
    std::array<double, 6> tau_, tau_cmd_, tau1_, min_val_, max_val_;
    std::array<double, 6> f_cur_, be_, ke_;
    std::vector<double> lambda1_vector, lambda2_vector, lambda3_vector, fc_vector, fv_vector, fs_vector, kf_vector, wf_vector, km_vector, w1_vector, gamma1_vector, miu_vector, eta_vector, k1_vector, Md_vector, Bd_vector, ki_vector, kp_vector, epsilon_N_vector;
    std::vector<double> l_vector, c1_vector, c2_vector;
    
    Eigen::Matrix<double, 6, 1> q_cmd_;
    Eigen::Matrix<double, 2, 2> M_, C_, J_, J_INV_, Md_, Md_INV_;
    Eigen::Matrix<double, 2, 1> k1_, dk1_, e_, s_, km_, w1_, gamma1_, eta_, miu_, ki_;
    Eigen::Matrix<double, 2, 1> k2_, g_, dg_, Phi1_, Phi2_, Part_of_Integral_, dPart_of_Integral_;
    Eigen::Matrix<double, 2, 1> lambda1_, lambda2_, lambda3_, k3_, epsilon_, beta0_, qd_, dqd_, ddqd_, Bd_;
    Eigen::Matrix<double, 2, 1> F_Joint_, Fext_, dqr_, ddqr_, dqr_last_, fc_, fv_, fs_, kf_, wf_, N_hat_, kp_, epsilon_N_;
    Eigen::Matrix<double, 6, 1> q_m_, dq_m_, ddq_m_, tau_m_, G_;
    ros::Subscriber sub_q_cmd_, sub_forcetorque_sensor_;
    pinocchio::Model model;
    pinocchio::Data data;
    std::ofstream data_file_;
    std::string output_file_path_;

    /* ——  力-矩传感器零点标定 —— */
    // std::array<double, 6> f_offset_{ {0, 0, 0, 0, 0, 0} };     // 零偏结果
    // std::array<double, 6> f_offset_acc_{ {0, 0, 0, 0, 0, 0} }; // 累加器
    // bool ft_offset_ready_{ false };                            // 标定完成标志
    // std::size_t ft_offset_cnt_{ 0 };                           // 已采样计数
    // static constexpr std::size_t FT_OFFSET_SAMPLES = 200;    // 标定用样本数

    /* —— 力-矩传感器零点标定 —— */
    std::array<double, 6> f_offset_{ { 0, 0, 0, 0, 0, 0 } };
    std::array<double, 6> f_offset_acc_{ { 0, 0, 0, 0, 0, 0 } };
    bool ft_offset_ready_{ false };
    std::size_t ft_offset_cnt_{ 0 };
    static constexpr std::size_t FT_OFFSET_SAMPLES = 200;  // 开机静态标定样本数
    static constexpr double FT_REST_THRESHOLD = 0.3;  // 判定“松手”阈值 (N / N·m)
    static constexpr double FT_OFFSET_ADAPT_GAIN = 0.0005;  // 在线零偏自适应步长
    static constexpr double LPF_ALPHA = 0.05;  // 1-阶低通系数（0=全滤，1=无滤）
    std::array<double, 6> f_cur_lpf_{ { 0, 0, 0, 0, 0, 0 } };  // 低通后的输出（可选）

    Eigen::Matrix<double, 7, 1> w_hat1_, w_hat2_, h1_, h2_, dw_hat1_, dw_hat2_;
    Eigen::Matrix<double, 4, 1> in1_, in2_;
    Eigen::VectorXd col1_, col2_;
    double norm_val1,norm_val2, exponent1, exponent2;
    Eigen::Matrix<double, 4, 7> c1_, c2_;
    Eigen::Matrix<double, 7, 1> b_, l_;
  };
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdmittanceController, controller_interface::ControllerBase)