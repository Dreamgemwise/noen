
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

//

#define PI 3.141592

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
      std::cout << "即将对参数进行初始化啦######################################" << std::endl;
      // 初始化变量
      // 机器人臂长;
      l1 = 0.184;
      l2 = 0.324;
      // 速度场参数
      VD = 0.02 * M_PI;
      xc = 0.2;
      yc = -0.2;
      r0 = 0.1;
      epsilon1 = 0.00075;
      c0 = 250;
      k0 = 0.35;
      // 阻抗参数
      Md_ << 100.0, 0.0,
          0.0, 100.0;
      Bd_ << 2.0,
          2.0;

      // 滑膜参数
      lambda1_ << 1.0,
          0.5;
      lambda2_ << 0.1,
          0.1;
      lambda3_ << 0.5,
          0.5;
      km_ << 1.0,
          1.0;
      w1_ << 5.0,
          5.0;
      gamma1_ << 5.0,
          5.0;
      miu_ << 0.03,
          0.03;
      eta_ << 2.0,
          2.0;
      k3_ << 2.0,
          2.0;
      epsilon_ << 0.25,
          0.25;
      beta0_ << 4.0,
          4.0;
      // 控制律限制
      min_val = -15.0; // 下限
      max_val = 15.0;  // 上限
      q_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      dq_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      tau_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      qd_ << -0.8159,
          0.6772;
      Part_of_Integral_ << 0.0,
          0.0;
      k1_ << 1.5,
          1.5;
      g_ << 0.0,
          0.0;
      std::cout << "准备构建URDF模型#################################################" << std::endl;
      // urdf
      const std::string urdf_filename = PINOCCHIO_MODEL_DIR;
      pinocchio::urdf::buildModel(urdf_filename, model);
      data = pinocchio::Data(model);

      q_m_ << 0.0, 0.0, q_[2], 0.0, q_[4], 0.0;
      dq_m_ << 0.0, 0.0, dq_[2], 0.0, dq_[4], 0.0;
      ddq_m_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      tau_m_ << 0.0, 0.0, tau_[2], 0.0, tau_[4], 0.0;

      sub_q_cmd_ = n.subscribe("command", 1, &AdmittanceController::commandCB, this);

      sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor", 1, &AdmittanceController::updateFTsensor, this);

      return true;
    } // end of initialize

    void starting(const ros::Time &time)
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
    void update(const ros::Time &time, const ros::Duration &period)
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
      J_(0, 0) = -l1 * sin(q_[2]) - l2 * sin(q_[2] + q_[4]);
      J_(0, 1) = -l2 * sin(q_[2] + q_[4]);
      J_(1, 0) = l1 * cos(q_[2]) + l2 * cos(q_[2] + q_[4]);
      J_(1, 1) = l2 * cos(q_[2] + q_[4]);
      // 再计算雅可比矩阵的逆
      J_INV_ = J_.inverse();
      // 再计算速度场参考角速度：dqr
      dqr_ = get_dqr(q_[2], q_[4], l1, l2, xc, yc, r0, k0, epsilon1, VD, c0, J_INV_);
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
      Fext_(0) = f_cur_[0];
      Fext_(1) = f_cur_[2];
      // 输出 f_cur_[0] 和 f_cur_[1] 的数据
      ROS_INFO("f_cur_[0]: %f, f_cur_[1]: %f", f_cur_[0], f_cur_[1]);

      F_Joint_(0) = J_(0, 0) * Fext_(0) + J_(1, 0) * Fext_(1);
      F_Joint_(1) = J_(0, 1) * Fext_(0) + J_(1, 1) * Fext_(1);

      // 计算Md的逆
      Md_INV_ = Md_.inverse();

      // 导纳模型计算ddqd
      ddqd_(0) = Md_INV_(0, 0) * (F_Joint_(0) - Bd_(0) * (dqd_(0) - dqr_(0))) + Md_INV_(0, 1) * (F_Joint_(1) - Bd_(1) * (dqd_(1) - dqr_(1))) + ddqr_(0);
      ddqd_(1) = Md_INV_(1, 0) * (F_Joint_(0) - Bd_(0) * (dqd_(0) - dqr_(0))) + Md_INV_(1, 1) * (F_Joint_(1) - Bd_(1) * (dqd_(1) - dqr_(1))) + ddqr_(1);

      dqd_(0) += ddqd_(0) * T;
      dqd_(1) += ddqd_(1) * T;
      qd_(0) += dqd_(0) * T;
      qd_(1) += dqd_(1) * T;

      e_(0) = dq_[2] - dqd_(0);
      e_(1) = dq_[4] - dqd_(1);

      q_m_ << 0.0, 0.0, q_[2], 0.0, q_[4], 0.0;
      dq_m_ << 0.0, 0.0, dq_[2], 0.0, dq_[4], 0.0;
      pinocchio::crba(model, data, q_m_);
      M_ << data.M(2, 2), data.M(2, 4),
          data.M(4, 2), data.M(4, 4);
      pinocchio::computeCoriolisMatrix(model, data, q_m_, dq_m_);
      C_ << data.C(2, 2), data.C(2, 4),
          data.C(4, 2), data.C(4, 4);
      pinocchio::computeGeneralizedGravity(model, data, q_m_);
      G_ << data.g(2), data.g(4);

      dPart_of_Integral_ = get_dPart_of_Integral(e_(0), e_(1));
      Part_of_Integral_(0) += dPart_of_Integral_(0) * T;
      Part_of_Integral_(1) += dPart_of_Integral_(1) * T;

      s_(0) = lambda1_(0) * e_(0) + lambda2_(0) * sign(e_(0)) * pow(fabs(e_(0)), 13.0 / 6.0) + lambda3_(0) * Part_of_Integral_(0);
      s_(1) = lambda1_(1) * e_(1) + lambda2_(1) * sign(e_(1)) * pow(fabs(e_(1)), 13.0 / 6.0) + lambda3_(1) * Part_of_Integral_(1);

      dk1_ = get_dk1(s_(0), s_(1), k1_(0), k1_(1), km_, w1_, gamma1_, eta_, miu_);

      Phi1_(0) = sqrt(fabs(s_(0))) * sign(s_(0)) + k3_(0) * s_(0);
      Phi1_(1) = sqrt(fabs(s_(1))) * sign(s_(1)) + k3_(1) * s_(1);

      Phi2_(0) = 0.5 * sign(s_(0)) + 1.5 * k3_(0) * sqrt(fabs(s_(0))) * sign(s_(0)) + k3_(0) * k3_(0) * s_(0);
      Phi2_(1) = 0.5 * sign(s_(1)) + 1.5 * k3_(1) * sqrt(fabs(s_(1))) * sign(s_(1)) + k3_(1) * k3_(1) * s_(1);

      k1_(0) += dk1_(0) * T;
      k1_(1) += dk1_(1) * T;

      k2_(0) = 2.0 * epsilon_(0) * k1_(0) + beta0_(0) + 4.0 * epsilon_(0) * epsilon_(0);
      k2_(1) = 2.0 * epsilon_(1) * k1_(1) + beta0_(1) + 4.0 * epsilon_(1) * epsilon_(1);

      dg_(0) = (-1.0) * k2_(0) * Phi2_(0);
      dg_(1) = (-1.0) * k2_(1) * Phi2_(1);
      g_(0) += dg_(0) * T;
      g_(1) += dg_(1) * T;

      tau_[2] = C_(0, 0) * dq_[2] + C_(0, 1) * dq_[4] + G_(0) - M_(0, 0) * ((-1.0) * k1_(0) * Phi1_(0) + g_(0) + (lambda1_(0) + (13.0 / 6.0) * lambda2_(0) * pow(fabs(e_(0)), 7.0 / 6.0)) * ddqd_(0) - lambda3_(0) * pow(fabs(e_(0)), 1.0 / 3.0) * sign(e_(0))) / (lambda1_(0) + (13.0 / 6.0) * lambda2_(0) * pow(fabs(e_(0)), 7.0 / 6.0)) - M_(0, 1) * ((-1.0) * k1_(1) * Phi1_(1) + g_(1) + (lambda1_(1) + (13.0 / 6.0) * lambda2_(1) * pow(fabs(e_(1)), 7.0 / 6.0)) * ddqd_(1) - lambda3_(1) * pow(fabs(e_(1)), 1.0 / 3.0) * sign(e_(1))) / (lambda1_(0) + (13.0 / 6.0) * lambda2_(0) * pow(fabs(e_(0)), 7.0 / 6.0));
      tau_[4] = C_(1, 0) * dq_[2] + C_(1, 1) * dq_[4] + G_(1) - M_(1, 0) * ((-1.0) * k1_(0) * Phi1_(0) + g_(0) + (lambda1_(0) + (13.0 / 6.0) * lambda2_(0) * pow(fabs(e_(0)), 7.0 / 6.0)) * ddqd_(0) - lambda3_(0) * pow(fabs(e_(0)), 1.0 / 3.0) * sign(e_(0))) / (lambda1_(1) + (13.0 / 6.0) * lambda2_(1) * pow(fabs(e_(1)), 7.0 / 6.0)) - M_(1, 1) * ((-1.0) * k1_(1) * Phi1_(1) + g_(1) + (lambda1_(1) + (13.0 / 6.0) * lambda2_(1) * pow(fabs(e_(1)), 7.0 / 6.0)) * ddqd_(1) - lambda3_(1) * pow(fabs(e_(1)), 1.0 / 3.0) * sign(e_(1))) / (lambda1_(1) + (13.0 / 6.0) * lambda2_(1) * pow(fabs(e_(1)), 7.0 / 6.0));

      tau_cmd_ = tau_;
      // 对扭矩命令进行限幅
      for (size_t i = 0; i < n_joints; i++)
      {
        tau_cmd_[i] = std::max(min_val, std::min(max_val, tau_cmd_[i]));
      }
      for (size_t i = 0; i < n_joints; i++)
      {
        joints_[i].setCommand(tau_cmd_[i]);
      }
    }

    void stopping(const ros::Time &time) {}

  private:
    std::vector<std::string> joint_names_;
    unsigned int n_joints;
    std::vector<hardware_interface::JointHandle> joints_;

    double t, T, l1, l2, xc, yc, r0, k0, epsilon1, VD, c0, min_val, max_val;
    std::array<double, 6> q_, dq_;
    std::array<double, 6> tau_, tau_cmd_;
    std::array<double, 6> f_cur_;

    Eigen::Matrix<double, 6, 1> q_cmd_;
    Eigen::Matrix<double, 2, 2> M_, C_, J_, J_INV_, Md_, Md_INV_;
    Eigen::Matrix<double, 2, 1> k1_, dk1_, e_, s_, km_, w1_, gamma1_, eta_, miu_;
    Eigen::Matrix<double, 2, 1> k2_, g_, dg_, Phi1_, Phi2_, Part_of_Integral_, dPart_of_Integral_;
    Eigen::Matrix<double, 2, 1> lambda1_, lambda2_, lambda3_, k3_, epsilon_, beta0_, qd_, dqd_, ddqd_, Bd_;
    Eigen::Matrix<double, 2, 1> F_Joint_, Fext_, dqr_, ddqr_, dqr_last_, G_;
    Eigen::Matrix<double, 6, 1> q_m_, dq_m_, ddq_m_, tau_m_;
    ros::Subscriber sub_q_cmd_, sub_forcetorque_sensor_;
    pinocchio::Model model;
    pinocchio::Data data;
  };
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdmittanceController, controller_interface::ControllerBase)
