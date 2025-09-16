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
#include <duration.h>
#include <xmate_exception.h>
#include <model.h>
#include <robot.h>

#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "print_rci.h"
#include "move.h"
#include "identify_model.h"

#include <vector>
#include <string>

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/robot/robot_model/xMatePro3_acc.urdf"
#endif

/**
 * @轨迹跟踪，定增益超扭曲，终端滑膜
 */

/**
 * 符号函数
 */
double sign(double x)
{
    return (x > 0) - (x < 0);
}
/**
 * 速度场
 * 返回:关节空间的速度dqr
 */
Eigen::Matrix<double, 2, 1> get_dqr(double q1, double q2, double l1, double l2, double xc,
                                    double yc, double r0, double k0, double epsilon1, double VD, double c0,
                                    Eigen::Matrix<double, 2, 2> J_INV)
{
    // 1、定义变量
    double x, y, u, k, c;
    Eigen::Matrix<double, 2, 1> A, B, du, v, dqr;

    // 2、正运动学，计算笛卡尔空间下轨迹
    x = l1 * cos(q1) + l2 * cos(q1 + q2); // 末端轨迹横坐标
    y = l1 * sin(q1) + l2 * sin(q1 + q2); // 末端轨迹纵坐标

    // 3、计算中间矩阵A,B
    A << x - xc,
        y - yc;
    B << -(y - yc),
        x - xc;

    // 4、计算中间变量u,du,k,c
    u = (x - xc) * (x - xc) + (y - yc) * (y - yc) - r0 * r0;
    du = 2.0 * A;
    k = k0 / (fabs(u) * sqrt(du(0) * du(0) + du(1) * du(1)) + epsilon1);
    c = VD * exp((-1.0) * c0 * fabs(u)) / sqrt(du(0) * du(0) + du(1) * du(1));

    // 5、计算笛卡尔空间下的末端速度
    v(0) = -2.0 * k * u * A(0) + 2.0 * c * B(0);
    v(1) = -2.0 * k * u * A(1) + 2.0 * c * B(1);

    // 6、转关节空间
    dqr(0) = J_INV(0, 0) * v(0) + J_INV(0, 1) * v(1);
    dqr(1) = J_INV(1, 0) * v(0) + J_INV(1, 1) * v(1);
    return dqr;
}
/**
 * 计算虚拟墙产生的力Fr
 */
Eigen::Matrix<double, 2, 1> get_Fr(double q1, double q2, double dq1, double dq2,
                                   double l1, double l2, double xc, double yc,
                                   double Vw, Eigen::Matrix<double, 2, 1> Br, double b,
                                   double Pw, Eigen::Matrix<double, 2, 1> Kr, double k)
{
    // 1、定义变量
    double x, y, vx, vy;
    Eigen::Matrix<double, 2, 1> Fr, du, n_vector, t_vector;

    // 2、正运动学:计算当前末端再笛卡尔空间下的位置和速度
    x = l1 * cos(q1) + l2 * cos(q1 + q2);                                        // x方向位置
    y = l1 * sin(q1) + l2 * sin(q1 + q2);                                        // y方向位置
    vx = (-l1 * sin(q1) - l2 * sin(q1 + q2)) * dq1 + (-l2 * sin(q1 + q2) * dq2); // x方向速度
    vy = (l1 * cos(q1) + l2 * cos(q1 + q2)) * dq1 + (l2 * cos(q1 + q2) * dq2);   // y方向速度
    // 3、计算机械臂末端在速度场轨迹中的  单位法向量和  单位切向量
    // 构建中间变量du
    du << 2.0 * (x - xc),
        2.0 * (y - yc);
    // 计算单位法向量
    n_vector(0) = du(0) / sqrt(du(0) * du(0) + du(1) * du(1));
    n_vector(1) = du(1) / sqrt(du(0) * du(0) + du(1) * du(1));
    // 计算单位切向量
    t_vector(0) = -2.0 * (y - yc) / sqrt(du(0) * du(0) + du(1) * du(1));
    t_vector(1) = 2.0 * (x - xc) / sqrt(du(0) * du(0) + du(1) * du(1));

    // 4、增大阻尼实现速度虚拟墙
    // 定义一个阻尼系数用于计算虚拟墙力
    Eigen::Matrix<double, 2, 1> Brr;
    // 判断速度是否超限了
    if (Vw < sqrt(vx * vx + vy * vy))
    {
        // 如果超限了，Brr根据超出的大小计算
        Brr(0) = Br(0) * tanh(b * (sqrt(vx * vx + vy * vy) - Vw));
        Brr(1) = Br(1) * tanh(b * (sqrt(vx * vx + vy * vy) - Vw));
    }
    else
    {
        Brr << 0.0,
            0.0;
    }

    // 5、增大刚度实现位置虚拟墙
    // 定义一个新的刚度用于计算虚拟墙力
    Eigen::Matrix<double, 2, 1> Krr;
    // 判断机械臂末端是不是超过了虚拟墙边界
    if (Pw < sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc)))
    {
        // 如果超过了边界，根据超出的大小计算新的刚度系数
        Krr(0) = Kr(0) * tanh(k * (sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc)) - Pw));
        Krr(1) = Kr(1) * tanh(k * (sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc)) - Pw));
    }
    else
    {
        Krr << 0.0,
            0.0;
    }
    // 7、计算笛卡尔空间下，虚拟墙的合力
    Fr(0) = Brr(0) * (Vw - sqrt(vx * vx + vy * vy)) * t_vector(0) + Krr(0) * (Pw - sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc))) * n_vector(0);
    Fr(1) = Brr(1) * (Vw - sqrt(vx * vx + vy * vy)) * t_vector(1) + Krr(1) * (Pw - sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc))) * n_vector(1);
    return Fr;
}

/**
 * 用于计算滑膜面s的某个积分部分
 */
Eigen::Matrix<double, 2, 1> get_dPart_of_Integral(double e1, double e2)
{
    Eigen::Matrix<double, 2, 1> dPart_of_Integral;
    dPart_of_Integral(0) = pow(fabs(e1), 1.0 / 3.0) * sign(e1);
    dPart_of_Integral(1) = pow(fabs(e2), 1.0 / 3.0) * sign(e2);
    return dPart_of_Integral;
}
Eigen::Matrix<double, 2, 1> get_dk1(double s1, double s2, double k1_1, double k1_2,
                                    Eigen::Matrix<double, 2, 1> km, Eigen::Matrix<double, 2, 1> w1,
                                    Eigen::Matrix<double, 2, 1> gamma1, Eigen::Matrix<double, 2, 1> eta,
                                    Eigen::Matrix<double, 2, 1> miu)
{
    // 1、定义变量
    Eigen::Matrix<double, 2, 1> dk1;
    // 2、两个轴分开计算，先算第一个轴:dk1(0)
    // 判断k1是否超限
    if (k1_1 > km(0))
    {
        dk1(0) = w1(0) * sqrt(gamma1(0) / 2.0) * sign(fabs(s1) - miu(0));
    }
    else
    {
        // k1太小，则以eta为加速度增大k1
        dk1(0) = eta(0);
    }

    // 3、同理，计算第二个轴:dk1(1)
    if (k1_2 > km(1))
    {
        dk1(1) = w1(1) * sqrt(gamma1(1) / 2.0) * sign(fabs(s2) - miu(1));
    }
    else
    {
        // k1太小，则以eta为加速度增大k1
        dk1(1) = eta(1);
    }
    return dk1;
}
using namespace xmate;
using TorqueControl = std::function<Torques(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[])
{
    // 读取外界力扰动数据
    std::ifstream file("/home/robot/RCI/rci_client(复件)(复件)/myprograms/F1_data.csv");
    std::string line;
    std::vector<double> Fd1, Fd2;

    if (!file.is_open())
    {
        std::cerr << "无法打开文件！" << std::endl;
        return 1;
    }

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string value1Str, value2Str;

        // 假设每行都有两个以逗号分隔的值
        if (std::getline(iss, value1Str, ',') && std::getline(iss, value2Str))
        {
            try
            {
                double value1 = std::stod(value1Str);
                double value2 = std::stod(value2Str);
                Fd1.push_back(value1);
                Fd2.push_back(value2);
            }
            catch (const std::invalid_argument &e)
            {
                // 处理转换错误（例如，非数字字符串）
                std::cerr << "转换错误: " << e.what() << std::endl;
                continue; // 或者可以退出循环，取决于你的需求
            }
            catch (const std::out_of_range &e)
            {
                // 处理范围错误（例如，数字太大或太小）
                std::cerr << "范围错误: " << e.what() << std::endl;
                continue; // 或者可以退出循环
            }
        }
    }

    file.close();

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

    // 2、创建机器人变量，连接上位机
    std::string ipaddr = "192.168.2.160";                    // IP地址
    uint16_t port = 1337;                                    // 端口号
    xmate::Robot robot(ipaddr, port, XmateType::XMATE3_PRO); // 创建机器人对象,连接上位机
    sleep(1);

    // 3、将机器人运动到初始位置，设置机器人控制运动模式
    robot.setMotorPower(1); // 上电
    std::array<double, 7> q_init;
    std::array<double, 7> q_drag = {{0.0, 0.0, -0.8159, M_PI / 2.0, M_PI / 2.0, 0.6772, 0.0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, q_drag, robot); // 机器人运动到初始位置

    xmate::XmateModel model1(&robot, xmate::XmateType::XMATE3_PRO);
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    // 4、创建文件输出流
    std::ofstream tau_outfile("tau_data.csv");
    std::ofstream e_outfile("e_data.csv");
    std::ofstream q_outfile("q_data.csv");
    std::ofstream qd_outfile("qd_data.csv");
    std::ofstream k1_outfile("k1_data.csv");
    std::ofstream k2_outfile("k2_data.csv");
    std::ofstream dq_outfile("dq_data.csv");
    std::ofstream s_outfile("s_data.csv");
    std::ofstream F_outfile("F_data.csv");
    std::ofstream Fr_outfile("Fr_data.csv");
    std::ofstream F_force_outfile("F_force_data.csv");
    std::ofstream F1_outfile("F1_data.csv");
    std::ofstream a_hat_outfile("a_hat_data.csv");
    std::ofstream F_Joint_outfile("F_Joint_data.csv");
    std::ofstream Fr_Joint_outfile("Fr_Joint_data.csv");
    // 5、定义并赋值循环需要使用到的变量
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
    Eigen::Matrix<double, 2, 2> C; // 科氏矩阵
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

    // 6、使用URDF构建机器人模型，用于提取机器人参数
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR;
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    Eigen::VectorXd q_m(model.nq), dq_m(model.nv), ddq_m(model.nv), tau_m(model.nq);
    q_m << 0.0, 0.0, q[2], M_PI / 2.0, M_PI / 2.0, q[5], 0.0;
    dq_m << 0.0, 0.0, dq[2], 0.0, 0.0, dq[5], 0.0;
    ddq_m << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    tau_m << 0.0, 0.0, tau[2], 0.0, 0.0, tau[5], 0.0;

    // 7、lambda表达式，程序循环
    TorqueControl torque_control_callback;
    torque_control_callback = [&](RCI::robot::RobotState robot_state) -> Torques
    {
        t += T;

        // 1、参考轨迹
        // xr = xc + r0 * cos(0.2 * M_PI * t);
        // yr = yc + r0 * sin(0.2 * M_PI * t);
        // q0 = atan2(yr, xr);
        // a0 = pow(xr * xr + yr * yr, 0.5);
        // temp = (xr * xr + yr * yr + l1 * l1 - l2 * l2) / (2 * a0 * l1);
        // qr1 = (-1) * acos(temp) + q0;
        // temp = (xr * xr + yr * yr - l1 * l1 + l2 * l2) / (2 * a0 * l2);
        // qr2 = acos(temp) + q0 - qr1;

        // 2、计算期望轨迹
        // a获取机器人信息
        pos = robot_state.toolTobase_pos_m; // 末端位置
        q = robot_state.q;                  // 关节角度
        dq = robot_state.dq_m;              // 关节角速度
        F1 = robot_state.tau_ext_in_base;   // 基坐标系下的外部力
        F2[0] = F1[0] - Fd1[t / T - 1];
        F2[1] = F1[1] - Fd2[t / T - 1];
        // F2[0] = F1[0];
        // F2[1] = F1[1];
        F_force = robot_state.tau_ext_in_stiff; // 力控坐标系下的外界力
        //     一阶低通IIR滤波器，公式：y[n]=α*x[n]+(1-α)*y[n-1]
        // 选择一个合适的α值，该值决定了滤波器的截止频率
        // 初始化前一个输出y[n-1]为0或某个初始值
        // 在每个循环周期中，使用上述差分方程计算新的输出y[n]

        F[0] = alpha * F2[0] + (1 - alpha) * F_last[0];
        F[1] = alpha * F2[1] + (1 - alpha) * F_last[1];

        F_last[0] = F[0];
        F_last[1] = F[1];

        if (t == T)
        {
            dq_last = dq;
        }
        for (int i = 0; i < 7; i++)
        {
            ddq[i] = (dq[i] - dq_last[i]) / T;
        }
        dq_last = dq;
        // model1.GetTauWithFriction(q, dq, ddq, trq_full, trq_inertial, trq_coriolis, trq_friction, trq_gravity);
        // b速度场计算参考角速度:dqr
        // 先计算雅可比矩阵
        J(0, 0) = -l1 * sin(q[2]) - l2 * sin(q[2] + q[5]);
        J(0, 1) = -l2 * sin(q[2] + q[5]);
        J(1, 0) = l1 * cos(q[2]) + l2 * cos(q[2] + q[5]);
        J(1, 1) = l2 * cos(q[2] + q[5]);
        // 再计算雅可比矩阵的逆
        J_INV = J.inverse();
        // 再计算速度场参考角速度:dqr
        dqr = get_dqr(q[2], q[5], l1, l2, xc, yc, r0, k0, epsilon1, VD, c0, J_INV);
        // c计算dqr的微分:ddqr
        // 初始化ddqr_last
        if (t == T)
        {
            dqr_last(0) = dqr(0);
            dqr_last(1) = dqr(1);
        }
        // 计算ddqr
        ddqr(0) = (dqr(0) - dqr_last(0)) / T;
        ddqr(1) = (dqr(1) - dqr_last(1)) / T;
        // 存储dqr
        dqr_last(0) = dqr(0);
        dqr_last(1) = dqr(1);
        // 存在的基础干扰
        // F[0] = F[0] - F_dx;
        // F[1] = F[1] - F_dy;
        // 滤除噪声产生的干扰:x方向[-5,5]，y方向[-5,5]
        if (F[0] <= F_d && F[0] >= (-1) * F_d)
        {
            F[0] = 0;
        }
        else
        {
            if (F[0] > 0)
            {
                F[0] = F[0] - F_d;
            }
            else
            {
                F[0] = F[0] + F_d;
            }
        }
        if (F[1] <= F_d && F[1] >= (-1) * F_d)
        {
            F[1] = 0;
        }
        else
        {
            if (F[1] > 0)
            {
                F[1] = F[1] - F_d;
            }
            else
            {
                F[1] = F[1] + F_d;
            }
        }

        if (F[0] < 0.0)
        {
            F[0] = 0.0;
        }

        if (F[1] < 0.0)
        {
            F[1] = 0.0;
        }
        // 转关节空间
        F_Joint(0) = J(0, 0) * (-1) * F[0] + J(1, 0) * (-1) * F[1];
        F_Joint(1) = J(0, 1) * (-1) * F[0] + J(1, 1) * (-1) * F[1];
        // F_Joint(0) = 0.0;
        // F_Joint(1) = 0.0;
        // f计算虚拟墙力fr
        Fr = get_Fr(q[2], q[5], dq[2], dq[5], l1, l2, xc, yc, Vw, Br, b, Pw, Kr, k);
        // 转关节空间
        Fr_Joint(0) = J(0, 0) * Fr(0) + J(1, 0) * Fr(1);
        Fr_Joint(1) = J(0, 1) * Fr(0) + J(1, 1) * Fr(1);
        // 计算Md的逆
        Md_INV = Md.inverse();
        // 初始化dqd
        if (t == T)
        {
            dqd(0) = dqr(0);
            dqd(1) = dqr(1);
        }
        // 导纳模型计算ddqd
        ddqd(0) = Md_INV(0, 0) * (F_Joint(0) + Fr_Joint(0) - Bd(0) * (dqd(0) - dqr(0))) + Md_INV(0, 1) * (F_Joint(1) + Fr_Joint(1) - Bd(1) * (dqd(1) - dqr(1))) + ddqr(0);
        ddqd(1) = Md_INV(1, 0) * (F_Joint(0) + Fr_Joint(0) - Bd(0) * (dqd(0) - dqr(0))) + Md_INV(1, 1) * (F_Joint(1) + Fr_Joint(1) - Bd(1) * (dqd(1) - dqr(1))) + ddqr(1);
        // h计算dqd,qd
        dqd(0) += ddqd(0) * T;
        dqd(1) += ddqd(1) * T;
        qd(0) += dqd(0) * T;
        qd(1) += dqd(1) * T;

        // 4计算误差e
        e(0) = dq[2] - dqd(0);
        e(1) = dq[5] - dqd(1);
        // b获取惯性矩阵M,C
        q_m << 0.0, 0.0, q[2], M_PI / 2.0, M_PI / 2.0, q[5], 0.0;
        dq_m << 0.0, 0.0, dq[2], 0.0, 0.0, dq[5], 0.0;
        pinocchio::crba(model, data, q_m);
        M << data.M(2, 2), data.M(2, 5),
            data.M(5, 2), data.M(5, 5);
        pinocchio::computeCoriolisMatrix(model, data, q_m, dq_m);
        C << data.C(2, 2), data.C(2, 5),
            data.C(5, 2), data.C(5, 5);
        // c计算滑膜面s
        // 计算滑膜面中的积分项
        dPart_of_Integral = get_dPart_of_Integral(e(0), e(1));
        Part_of_Integral(0) += dPart_of_Integral(0) * T;
        Part_of_Integral(1) += dPart_of_Integral(1) * T;
        // 计算s
        s(0) = lambda1(0) * e(0) + lambda3(0) * sign(e(0)) * pow(fabs(e(0)), 13.0 / 6.0) + lambda2(0) * Part_of_Integral(0);
        s(1) = lambda1(1) * e(1) + lambda3(1) * sign(e(1)) * pow(fabs(e(1)), 13.0 / 6.0) + lambda2(1) * Part_of_Integral(1);
        // d计算Phi1
        Phi1(0) = sqrt(fabs(s(0))) * sign(s(0)) + k3(0) * s(0);
        Phi1(1) = sqrt(fabs(s(1))) * sign(s(1)) + k3(1) * s(1);
        // e计算Phi2
        Phi2(0) = 0.5 * sign(s(0)) + 1.5 * k3(0) * sqrt(fabs(s(0))) * sign(s(0)) + k3(0) * k3(0) * s(0);
        Phi2(1) = 0.5 * sign(s(1)) + 1.5 * k3(1) * sqrt(fabs(s(1))) * sign(s(1)) + k3(1) * k3(1) * s(1);
        // f计算k1(t)
        // 先计算dk1(t)
        dk1 = get_dk1(s(0), s(1), k1(0), k1(1), km, w1, gamma1, eta, miu);
        // 积分
        k1(0) += dk1(0) * T;
        k1(1) += dk1(1) * T;
        // k1(0) = 0.0;
        // k1(1) = 0.0;
        // g计算k2(t)
        k2(0) = 2.0 * epsilon(0) * k1(0) + beta0(0) + 4.0 * epsilon(0) * epsilon(0);
        k2(1) = 2.0 * epsilon(1) * k1(1) + beta0(1) + 4.0 * epsilon(1) * epsilon(1);
        // h计算g
        // 先计算g的微分dg
        dg(0) = (-1.0) * k2(0) * Phi2(0);
        dg(1) = (-1.0) * k2(1) * Phi2(1);
        // 积分
        g(0) += dg(0) * T;
        g(1) += dg(1) * T;
        // i计算tau
        // 计算惯性矩阵的逆
        M_INV = M.inverse();
        // 计算自适应律
        dot_a_hat(0) = (-1) * h(0) * a_hat(0) + (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) * (M_INV(0, 0) * s(0) + M_INV(0, 1) * s(1));
        dot_a_hat(1) = (-1) * h(1) * a_hat(1) + (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) * (M_INV(1, 0) * s(0) + M_INV(1, 1) * s(1));

        a_hat(0) = a_hat(0) + dot_a_hat(0) * T;
        a_hat(1) = a_hat(1) + dot_a_hat(1) * T;
        tau[2] = C(0, 0) * dq[2] + C(0, 1) * dq[5] + M(0, 0) * ((-1.0) * k1(0) * Phi1(0) + g(0) + (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) * ddqd(0) - lambda2(0) * pow(fabs(e(0)), 1.0 / 3.0) * sign(e(0))) / (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) + M(0, 1) * ((-1.0) * k1(1) * Phi1(1) + g(1) + (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) * ddqd(1) - lambda2(1) * pow(fabs(e(1)), 1.0 / 3.0) * sign(e(1))) / (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) + sign(s(0)) * a_hat(0);
        tau[5] = C(1, 0) * dq[2] + C(1, 1) * dq[5] + M(1, 0) * ((-1.0) * k1(0) * Phi1(0) + g(0) + (lambda1(0) + (13.0 / 6.0) * lambda3(0) * pow(fabs(e(0)), 7.0 / 6.0)) * ddqd(0) - lambda2(0) * pow(fabs(e(0)), 1.0 / 3.0) * sign(e(0))) / (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) + M(1, 1) * ((-1.0) * k1(1) * Phi1(1) + g(1) + (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) * ddqd(1) - lambda2(1) * pow(fabs(e(1)), 1.0 / 3.0) * sign(e(1))) / (lambda1(1) + (13.0 / 6.0) * lambda3(1) * pow(fabs(e(1)), 7.0 / 6.0)) + sign(s(1)) * a_hat(1);

        // 控制律限幅
        if (tau[2] > max_val)
        {
            tau[2] = max_val;
        }
        else if (tau[2] < min_val)
        {
            tau[2] = min_val;
        }

        if (tau[5] > max_val)
        {
            tau[5] = max_val;
        }
        else if (tau[5] < min_val)
        {
            tau[5] = min_val;
        }
        //  给其他轴施加虚拟墙。
        //     实现步骤：
        //         获取其他关节的目标关节角度q07x1
        // q_drag
        //         根据公式:tau1=-ke((q-q0)-delta),|q-q0|>delta计算虚拟墙的力tau1
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
        if (fabs(q[6] - q_drag[6]) > delta)
        {
            tau1[6] = (-1) * be[6] * tanh(B * dq[6]) - ke[6] * tanh(K * ((q[6] - q_drag[6]) - delta));
        } // 轴6
        // 加入到控制率tau中即可
        // tau[0] = tau[0] + tau1[0];
        // tau[1] = tau[1] + tau1[1];
        // tau[3] = tau[3] + tau1[3];
        // tau[4] = tau[4] + tau1[4];
        // tau[6] = tau[6] + tau1[6];
        tau[0] = tau1[0];
        tau[1] = tau1[1];
        tau[3] = tau1[3];
        tau[4] = tau1[4];
        tau[6] = tau1[6];
        // tau[0] = 0.0;
        // tau[1] = 0.0;
        // tau[3] = 0.0;
        // tau[4] = 0.0;
        // tau[6] = 0.0;
        // 4、输出
        Torques output{};
        output.tau_c = tau;

        tau_outfile << tau[2] << "," << tau[5] << "\n";
        e_outfile << e(0) << "," << e(1) << "\n";
        q_outfile << q[2] << "," << q[5] << "\n";
        qd_outfile << qd(0) << "," << qd(1) << "\n";
        k1_outfile << k1(0) << "," << k1(1) << "\n";
        k2_outfile << k2(0) << "," << k2(1) << "\n";
        dq_outfile << dq[2] << "," << dq[5] << "\n";
        s_outfile << s(0) << "," << s(1) << "\n";
        F_outfile << F[0] << "," << F[1] << "\n";
        Fr_outfile << Fr(0) << "," << Fr(1) << "\n";
        F_force_outfile << F_force[0] << "," << F_force[1] << "\n";
        F1_outfile << F1[0] << "," << F1[1] << "\n";
        a_hat_outfile << a_hat(0) << "," << a_hat(1) << "\n";
        F_Joint_outfile << F_Joint(0) << "," << F_Joint(1) << "\n";
        Fr_Joint_outfile << Fr_Joint(0) << "," << Fr_Joint(1) << "\n";

        if (count < 10)
        {
            count++;
        }
        else
        {
            std::cout << std::left << std::setw(20) << F_Joint(0)
                      << std::left << std::setw(20) << F_Joint(1)
                      << std::left << std::setw(20) << F[0]
                      << std::left << std::setw(20) << F[1]
                      << std::left << std::setw(20) << M(0, 0)
                      << std::left << std::setw(20) << M(0, 1)
                      << std::left << std::setw(20) << M(1, 0)
                      << std::left << std::setw(20) << M(1, 1)
                      << std::left << std::setw(20) << C(0, 0)
                      << std::left << std::setw(20) << C(0, 1)
                      << std::left << std::setw(20) << C(1, 0)
                      << std::left << std::setw(20) << C(1, 1)
                      << std::endl;
            count = 0;
        }
        // 5、结束运动
        if (t > 20)
        {
            tau_outfile.close();
            e_outfile.close();
            q_outfile.close();
            qd_outfile.close();
            k1_outfile.close();
            k2_outfile.close();
            dq_outfile.close();
            s_outfile.close();
            F_outfile.close();
            Fr_outfile.close();
            F_force_outfile.close();
            F1_outfile.close();
            a_hat_outfile.close();
            F_Joint_outfile.close();
            Fr_Joint_outfile.close();
            std::cout << "终端滑膜_自适应变增益STA:运动结束" << std::endl;
            return MotionFinished(output);
        }
        return output;
    };
    robot.Control(torque_control_callback);
    // robot.setMotorPower(0);
    return 0;
}