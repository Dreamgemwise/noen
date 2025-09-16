
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "HR_Pro.h"
#include "HR_Errorcode.h"
constexpr int BOX_ID = 0;
constexpr int RBT_ID = 0;
const char *CPS_IP = "192.168.43.185"; // 替换成你传感器的 IP
const int CPS_PORT = 10003;

bool connectToFTSensor()
{
  HRIF_DisConnect(BOX_ID);
  int ret = HRIF_Connect(BOX_ID, CPS_IP, CPS_PORT);
  if (ret != 0)
  {
    ROS_ERROR("HRIF_Connect failed with code: %d", ret);
    return false;
  }
  ROS_INFO("Connected to FT sensor successfully.");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ft_data_advertiser");
  ros::NodeHandle nh;

  ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor", 10);
  ros::Rate rate(250); // 250Hz

  bool connected = connectToFTSensor();

  while (ros::ok())
  {
    if (!connected || !HRIF_IsConnected(BOX_ID))
    {
      ROS_WARN_THROTTLE(2.0, "FT sensor disconnected. Attempting to reconnect...");
      connected = connectToFTSensor();
      ros::Duration(2.0).sleep();
      continue;
    }

    double fx, fy, fz, tx, ty, tz;
    int ret = HRIF_ReadFTCabData(BOX_ID, RBT_ID, fx, fy, fz, tx, ty, tz);

    if (ret == REC_Successed)
    {
      geometry_msgs::WrenchStamped wrench;
      wrench.header.stamp = ros::Time::now();
      wrench.wrench.force.x = fx;
      wrench.wrench.force.y = fy;
      wrench.wrench.force.z = fz;
      wrench.wrench.torque.x = tx;
      wrench.wrench.torque.y = ty;
      wrench.wrench.torque.z = tz;
      ft_pub.publish(wrench);
    }
    else if (ret == REC_SDK_NotConnected)
    {
      ROS_WARN_THROTTLE(2.0, "FT sensor disconnected.");
      connected = false;
    }
    else
    {
      ROS_ERROR_THROTTLE(2.0, "Error reading FT data, code: %d", ret);
    }

    ros::spinOnce();
    rate.sleep();
  }

  HRIF_DisConnect(BOX_ID);
  ROS_INFO("Disconnected from FT sensor. Exiting node.");
  return 0;
}

// /*******************************************************
//  *  FTData_advertiser.cpp
//  *  读取 HansRobot 控制箱 FT 传感器数据并发布 ROS 话题（已去除硬件连接逻辑）
//  * -----------------------------------------------------
//  *  话题 : /elfin/ft_raw  (geometry_msgs::WrenchStamped)
//  *  诊断 : /diagnostics   (FT Online / Offline)
//  *  SDK  : HR_Pro.h + libHR_Pro.so
//  *  日期 : 2025-07-06
//  *******************************************************/

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <diagnostic_updater/diagnostic_updater.h>
// #include <diagnostic_updater/publisher.h>
// #include <thread>
// #include <atomic>

// // ---------- HansRobot SDK ----------
// #include "HR_Pro.h"
// #include "HR_Errorcode.h"           // 含 REC_SDK_NotConnected=39500

// // ---------- 参数 ----------
// static constexpr uint32_t BOX_ID    = 0;                 // 电箱编号
// static constexpr uint32_t RBT_ID    = 0;                 // 机器人编号
// static constexpr double   LOOP_HZ   = 500.0;             // 发布频率
// static const     char*    CPS_IP    = "192.168.43.183";   // 控制箱 IP
// static constexpr int      CPS_PORT  = 10003;             // SDK 默认端口
// static constexpr double   OFFLINE_T = 0.2;               // 0.2 s 无数据视为掉线

// class FTNode
// {
// public:
//   FTNode()
//   : nh_("~"),
//     pub_(nh_.advertise<geometry_msgs::WrenchStamped>("/elfin/ft_raw", 10)),
//     running_(true)
//   {
//     diag_updater_.setHardwareID("Hans FT Sensor");
//     diag_updater_.add("FT connection", [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
//     {
//       if((ros::Time::now() - last_ok_).toSec() < OFFLINE_T)
//         stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "FT online");
//       else
//         stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "FT offline / no data");
//     });

//     th_ = std::thread(&FTNode::loop, this);
//   }

//   ~FTNode()
//   {
//     running_ = false;
//     if(th_.joinable()) th_.join();
//   }

// private:
//   // -------- 读循环线程 --------
//   void loop()
//   {
//     ros::Rate rate(LOOP_HZ);
//     while(running_ && ros::ok())
//     {
//       double fx=0, fy=0, fz=0, tx=0, ty=0, tz=0;
//       int ret = HRIF_ReadFTCabData(BOX_ID, RBT_ID, fx, fy, fz, tx, ty, tz);

//       if(ret == 0)                              // 成功
//       {
//         last_ok_ = ros::Time::now();
//         geometry_msgs::WrenchStamped msg;
//         msg.header.stamp = last_ok_;
//         msg.wrench.force.x  =  fx;
//         msg.wrench.force.y  =  fy;
//         msg.wrench.force.z  =  fz;
//         msg.wrench.torque.x =  tx;
//         msg.wrench.torque.y =  ty;
//         msg.wrench.torque.z =  tz;
//         pub_.publish(msg);
//       }
//       else if(ret == REC_SDK_NotConnected)      // 39500：掉线
//       {
//         ROS_WARN_THROTTLE(2.0, "FT sensor disconnected.");
//       }
//       else                                      // 其他错误
//       {
//         ROS_ERROR_THROTTLE(2.0, "HRIF_ReadFTCabData error=%d", ret);
//       }

//       diag_updater_.update();
//       rate.sleep();
//     }
//   }

//   // -------- 成员 --------
//   ros::NodeHandle nh_;
//   ros::Publisher  pub_;
//   diagnostic_updater::Updater diag_updater_;
//   ros::Time last_ok_{0};          // 最近一次成功时间

//   std::thread th_;
//   std::atomic<bool> running_;
// };

// // ---------------- main -----------------
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "hans_ft_sensor_node");
//   try{
//     FTNode node;
//     ros::spin();
//   }catch(const std::exception& e){
//     ROS_FATAL("%s", e.what());
//   }
//   return 0;
// }

// /*******************************************************
//  *  FTData_advertiser.cpp
//  *  读取 HansRobot 控制箱 FT 传感器数据并发布 ROS 话题
//  * -----------------------------------------------------
//  *  话题 : /elfin/ft_raw  (geometry_msgs::WrenchStamped)
//  *  诊断 : /diagnostics   (FT Online / Offline)
//  *  SDK  : HR_Pro.h + libHR_Pro.so
//  *  日期 : 2025-07-06
//  *******************************************************/

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <diagnostic_updater/diagnostic_updater.h>
// #include <diagnostic_updater/publisher.h>
// #include <thread>
// #include <atomic>

// // ---------- HansRobot SDK ----------
// #include "HR_Pro.h"
// #include "HR_Errorcode.h"           // 含 REC_SDK_NotConnected=39500

// // ---------- 参数 ----------
// static constexpr uint32_t BOX_ID    = 0;                 // 电箱编号
// static constexpr uint32_t RBT_ID    = 0;                 // 机器人编号
// static constexpr double   LOOP_HZ   = 500.0;             // 发布频率
// static const     char*    CPS_IP    = "192.168.43.183";   // 控制箱 IP
// static constexpr int      CPS_PORT  = 10003;             // SDK 默认端口
// static constexpr double   OFFLINE_T = 0.2;               // 0.2 s 无数据视为掉线

// class FTNode
// {
// public:
//   FTNode()
//   : nh_("~"),
//     pub_(nh_.advertise<geometry_msgs::WrenchStamped>("/elfin/ft_raw", 10)),
//     running_(true)
//   {
//     diag_updater_.setHardwareID("Hans FT Sensor");
//     diag_updater_.add("FT connection", [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
//     {
//       if((ros::Time::now() - last_ok_).toSec() < OFFLINE_T)
//         stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "FT online");
//       else
//         stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "FT offline / no data");
//     });

//     connectOnce();                       // 首次连接（失败会抛异常）
//     th_ = std::thread(&FTNode::loop, this);
//   }

//   ~FTNode()
//   {
//     running_ = false;
//     if(th_.joinable()) th_.join();
//     HRIF_DisConnect(BOX_ID);
//   }

// private:
//   // -------- 尝试连接一次，失败抛异常 --------
//   void connectOnce()
//   {
//     int ret = HRIF_Connect(BOX_ID, CPS_IP, CPS_PORT);
//     if(ret != 0)
//     {
//       ROS_FATAL("HRIF_Connect(%s:%d) failed, code=%d", CPS_IP, CPS_PORT, ret);
//       throw std::runtime_error("Cannot connect FT CPS");
//     }
//     ROS_INFO("Connected to CPS %s:%d (BOX_ID=%d)", CPS_IP, CPS_PORT, BOX_ID);
//     // 若固件需登录，可在此调用 HRIF_Login(…)
//   }

//   // -------- 读循环线程 --------
//   void loop()
//   {
//     ros::Rate rate(LOOP_HZ);
//     while(running_ && ros::ok())
//     {
//       double fx=0, fy=0, fz=0, tx=0, ty=0, tz=0;
//       int ret = HRIF_ReadFTCabData(BOX_ID, RBT_ID, fx, fy, fz, tx, ty, tz);

//       if(ret == 0)                              // 成功
//       {
//         last_ok_ = ros::Time::now();
//         geometry_msgs::WrenchStamped msg;
//         msg.header.stamp = last_ok_;
//         msg.wrench.force.x  =  fx;
//         msg.wrench.force.y  =  fy;
//         msg.wrench.force.z  =  fz;
//         msg.wrench.torque.x =  tx;
//         msg.wrench.torque.y =  ty;
//         msg.wrench.torque.z =  tz;
//         pub_.publish(msg);
//       }
//       else if(ret == REC_SDK_NotConnected)      // 39500：掉线
//       {
//         ROS_WARN_THROTTLE(2.0, "FT sensor disconnected, reconnecting…");
//         HRIF_DisConnect(BOX_ID);
//         // 重连直到成功或节点退出
//         while(running_ && ros::ok())
//         {
//           try{
//             connectOnce();
//             break;
//           }catch(...){
//             ros::Duration(1.0).sleep();
//           }
//         }
//       }
//       else                                      // 其他错误
//       {
//         ROS_ERROR_THROTTLE(2.0, "HRIF_ReadFTCabData error=%d", ret);
//       }

//       diag_updater_.update();
//       rate.sleep();
//     }
//   }

//   // -------- 成员 --------
//   ros::NodeHandle nh_;
//   ros::Publisher  pub_;
//   diagnostic_updater::Updater diag_updater_;
//   ros::Time last_ok_{0};          // 最近一次成功时间

//   std::thread th_;
//   std::atomic<bool> running_;
// };

// // ---------------- main -----------------
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "hans_ft_sensor_node");
//   try{
//     FTNode node;
//     ros::spin();
//   }catch(const std::exception& e){
//     ROS_FATAL("%s", e.what());
//   }
//   return 0;
// }

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>  // 修改为 geometry_msgs
// #include <diagnostic_updater/diagnostic_updater.h>
// #include <diagnostic_updater/publisher.h>
// #include <thread>  // 添加 <thread> 头文件
// #include "HR_Pro.h"               // SDK 头

// static constexpr double LOOP_HZ = 500.0;     // 读取/发布频率
// static constexpr unsigned BOX_ID = 0;
// static constexpr unsigned RBT_ID = 0;

// class FTNode
// {
// public:
//   FTNode()
//   : nh_("~"),
//     pub_(nh_.advertise<geometry_msgs::WrenchStamped>("/elfin/ft_raw", 10)),  // 修改为 geometry_msgs
//     diag_updater_(),
//     last_ok_(ros::Time::now())
//   {
//     diag_updater_.setHardwareID("HansRobot FT Sensor");
//     diag_updater_.add("FT connection", boost::bind(&FTNode::produceDiag, this, _1));

//     // 连接电箱 （若 SDK 需要，可选）
//     // int ret = HRIF_Connect2Box(BOX_ID);
//     // if(ret != 0) ROS_WARN("HRIF_Connect2Box error = %d", ret);

//     // 启动读取线程
//     th_ = std::thread([this](){ this->loop(); });
//   }

//   ~FTNode()
//   {
//     running_ = false;
//     if(th_.joinable()) th_.join();
//     // HRIF_DisconnectBox(BOX_ID);
//   }

// private:
//   void loop()
//   {
//     ros::Rate rate(LOOP_HZ);
//     while(running_ && ros::ok())
//     {
//       double fx, fy, fz, tx, ty, tz;
//       int ret = HRIF_ReadFTCabData(BOX_ID, RBT_ID, fx, fy, fz, tx, ty, tz);

//       if(ret == 0)
//       {
//         // 正常读取
//         last_ok_ = ros::Time::now();
//         geometry_msgs::WrenchStamped msg;  // 修改为 geometry_msgs
//         msg.header.stamp = last_ok_;
//         msg.wrench.force.x  = fx;  msg.wrench.force.y  = fy;  msg.wrench.force.z  = fz;
//         msg.wrench.torque.x = tx;  msg.wrench.torque.y = ty;  msg.wrench.torque.z = tz;
//         pub_.publish(msg);
//       }
//       else
//       {
//         ROS_ERROR_THROTTLE(2.0, "HRIF_ReadFTCabData error=%d", ret);
//       }

//       diag_updater_.update();
//       rate.sleep();
//     }
//   }

//   void produceDiag(diagnostic_updater::DiagnosticStatusWrapper& stat)
//   {
//     const double timeout = 0.1;                      // 100 ms 掉线阈值
//     if((ros::Time::now() - last_ok_).toSec() < timeout)
//       stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "FT sensor online");
//     else
//       stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "FT sensor offline / no data");
//   }

//   // data -----------------------------
//   ros::NodeHandle nh_;
//   ros::Publisher  pub_;
//   diagnostic_updater::Updater diag_updater_;
//   ros::Time last_ok_;
//   std::thread th_;
//   std::atomic<bool> running_{true};
// };

// int main(int argc,char** argv)
// {
//   ros::init(argc, argv, "hansrobot_ft_sensor");
//   FTNode node;
//   ros::spin();
//   return 0;
// }

// // FTData_advertiser.cpp  ——  发布大族 FT 传感器数据到 ROS
// // ----------------------------------------------------------
// // 依赖：libHR_Pro.so + HR_Pro.h
// // 编译：在 CMakeLists.txt 里  target_link_libraries(… HR_Pro ${catkin_LIBRARIES})
// // 话题：/elfin/ft_raw   geometry_msgs/WrenchStamped
// // 诊断：/diagnostics    diagnostic_msgs/DiagnosticArray

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <diagnostic_updater/diagnostic_updater.h>
// #include <diagnostic_updater/publisher.h>
// #include "HR_Pro.h"               // SDK 头

// #include <thread>

// static constexpr double LOOP_HZ = 500.0;     // 读取/发布频率
// static constexpr unsigned BOX_ID = 0;
// static constexpr unsigned RBT_ID = 0;

// class FTNode
// {
// public:
//   FTNode()
//   : nh_("~"),
//     pub_(nh_.advertise<sensor_msgs::WrenchStamped>("/elfin/ft_raw", 10)),
//     diag_updater_(),
//     last_ok_(ros::Time::now())
//   {
//     diag_updater_.setHardwareID("HansRobot FT Sensor");
//     diag_updater_.add("FT connection", boost::bind(&FTNode::produceDiag, this, _1));

//     // 连接电箱 （若 SDK 需要，可选）
//     int ret = HRIF_Connect2Box(BOX_ID);
//     if(ret != 0) ROS_WARN("HRIF_Connect2Box error = %d", ret);

//     // 启动读取线程
//     th_ = std::thread([this](){ this->loop(); });
//   }

//   ~FTNode()
//   {
//     running_ = false;
//     if(th_.joinable()) th_.join();
//     HRIF_DisconnectBox(BOX_ID);
//   }

// private:
//   void loop()
//   {
//     ros::Rate rate(LOOP_HZ);
//     while(running_ && ros::ok())
//     {
//       double fx, fy, fz, tx, ty, tz;
//       int ret = HRIF_ReadFTCabData(BOX_ID, RBT_ID, fx, fy, fz, tx, ty, tz);

//       if(ret == 0)
//       {
//         // 正常读取
//         last_ok_ = ros::Time::now();
//         sensor_msgs::WrenchStamped msg;
//         msg.header.stamp = last_ok_;
//         msg.wrench.force.x  = fx;  msg.wrench.force.y  = fy;  msg.wrench.force.z  = fz;
//         msg.wrench.torque.x = tx;  msg.wrench.torque.y = ty;  msg.wrench.torque.z = tz;
//         pub_.publish(msg);
//       }
//       else
//       {
//         ROS_ERROR_THROTTLE(2.0, "HRIF_ReadFTCabData error=%d", ret);
//       }

//       diag_updater_.update();
//       rate.sleep();
//     }
//   }

//   void produceDiag(diagnostic_updater::DiagnosticStatusWrapper& stat)
//   {
//     const double timeout = 0.1;                      // 100 ms 掉线阈值
//     if((ros::Time::now() - last_ok_).toSec() < timeout)
//       stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "FT sensor online");
//     else
//       stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "FT sensor offline / no data");
//   }

//   // data -----------------------------
//   ros::NodeHandle nh_;
//   ros::Publisher  pub_;
//   diagnostic_updater::Updater diag_updater_;
//   ros::Time last_ok_;
//   std::thread th_;
//   std::atomic<bool> running_{true};
// };

// int main(int argc,char** argv)
// {
//   ros::init(argc, argv, "hansrobot_ft_sensor");
//   FTNode node;
//   ros::spin();
//   return 0;
// }

// // ft_sensor_node.cpp
// #include <ros/ros.h>
// #include <sensor_msgs/WrenchStamped.h>
// #include "HR_Pro.h"
// // #include <HansRobotV5/FTData.h>
// // #include <HansRobotV5/HansRobotAPI.h>

// int main(int argc,char** argv){
//   ros::init(argc,argv,"elfin_ft_sensor");
//   ros::NodeHandle nh;
//   ros::Publisher pub = nh.advertise<sensor_msgs::WrenchStamped>("elfin/ft_raw",10);

//   HansRobot::FTData ft;
//   ros::Rate loop(500);               // 与控制器一致
//   while(ros::ok()){
//     if(HansRobot::GetFTData(ft)==0){ // SDK 调用
//       sensor_msgs::WrenchStamped msg;
//       msg.header.stamp = ros::Time::now();
//       msg.wrench.force.x  = ft.fx;   msg.wrench.force.y  = ft.fy;   msg.wrench.force.z  = ft.fz;
//       msg.wrench.torque.x = ft.tx;   msg.wrench.torque.y = ft.ty;   msg.wrench.torque.z = ft.tz;
//       pub.publish(msg);
//     }
//     loop.sleep();
//   }
//   return 0;
// }
