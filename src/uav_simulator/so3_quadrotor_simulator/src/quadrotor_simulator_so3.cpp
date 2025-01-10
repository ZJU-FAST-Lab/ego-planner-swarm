#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <so3_quadrotor_simulator/Quadrotor.h>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_utils/geometry_utils.h>

typedef struct _Control
{
    double rpm[4];
} Control;

typedef struct _Command
{
    float force[3];
    float qx, qy, qz, qw;
    float kR[3];
    float kOm[3];
    float corrections[3];
    float current_yaw;
    bool use_external_yaw;
} Command;

typedef struct _Disturbance
{
    Eigen::Vector3d f;
    Eigen::Vector3d m;
} Disturbance;

static Command command;
static Disturbance disturbance;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state,
                    nav_msgs::msg::Odometry &odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor &quad,
                  sensor_msgs::msg::Imu &imu);

static Control
// 根据无人机的当前状态和目标控制指令计算四个电机所需的转速（RPM）
getControl(const QuadrotorSimulator::Quadrotor &quad, const Command &cmd)
{
    const double _kf = quad.getPropellerThrustCoefficient(); // 推力系数
    const double _km = quad.getPropellerMomentCoefficient(); // 力矩系数
    const double kf = _kf - cmd.corrections[0];              // 补偿无人机推力偏差
    const double km = _km / _kf * kf;                        // 更新推力和力矩的比例

    // 获取无人机的臂长 d 和惯性矩阵 J
    const double d = quad.getArmLength();
    const Eigen::Matrix3f J = quad.getInertia().cast<float>();
    const float I[3][3] = {{J(0, 0), J(0, 1), J(0, 2)},
                           {J(1, 0), J(1, 1), J(1, 2)},
                           {J(2, 0), J(2, 1), J(2, 2)}};
    const QuadrotorSimulator::Quadrotor::State state = quad.getState();

    // Rotation, may use external yaw
    // 当前姿态的旋转矩阵转换为欧拉角
    Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);
    Eigen::Vector3d ypr = _ypr;
    if (cmd.use_external_yaw)
        ypr[0] = cmd.current_yaw;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
    float R11 = R(0, 0);
    float R12 = R(0, 1);
    float R13 = R(0, 2);
    float R21 = R(1, 0);
    float R22 = R(1, 1);
    float R23 = R(1, 2);
    float R31 = R(2, 0);
    float R32 = R(2, 1);
    float R33 = R(2, 2);
    /*
      float R11 = state.R(0,0);
      float R12 = state.R(0,1);
      float R13 = state.R(0,2);
      float R21 = state.R(1,0);
      float R22 = state.R(1,1);
      float R23 = state.R(1,2);
      float R31 = state.R(2,0);
      float R32 = state.R(2,1);
      float R33 = state.R(2,2);
    */
    float Om1 = state.omega(0);
    float Om2 = state.omega(1);
    float Om3 = state.omega(2);

    // 通过指令的数据生成目标姿态的旋转矩阵
    float Rd11 =
        cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
    float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
    float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
    float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
    float Rd22 =
        cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
    float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
    float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
    float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
    float Rd33 =
        cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

    // 通过当前旋转矩阵和目标旋转矩阵计算稳定性指标
    float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                                Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                                Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

    float force = 0;
    // 如果稳定性条件满足，根据期望的力方向和当前旋转矩阵计算推力
    if (Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
        force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

    // 计算旋转误差和角速度误差
    float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                        R32 * Rd33 - R33 * Rd32);
    float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                        R31 * Rd33 + R33 * Rd31);
    float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                        R31 * Rd32 - R32 * Rd31);

    float eOm1 = Om1;
    float eOm2 = Om2;
    float eOm3 = Om3;

    // 计算角速度引起的非线性耦合补偿项
    float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
                Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
    float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
                Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
    float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
                Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);
    /*
      // Robust Control --------------------------------------------
      float c2       = 0.6;
      float epsilonR = 0.04;
      float deltaR   = 0.1;
      float eA1 = eOm1 + c2 * 1.0/I[0][0] * eR1;
      float eA2 = eOm2 + c2 * 1.0/I[1][1] * eR2;
      float eA3 = eOm3 + c2 * 1.0/I[2][2] * eR3;
      float neA = sqrt(eA1*eA1 + eA2*eA2 + eA3*eA3);
      float muR1 = -deltaR*deltaR * eA1 / (deltaR * neA + epsilonR);
      float muR2 = -deltaR*deltaR * eA2 / (deltaR * neA + epsilonR);
      float muR3 = -deltaR*deltaR * eA3 / (deltaR * neA + epsilonR);
      // Robust Control --------------------------------------------
    */
    // 根据旋转误差、角速度误差和耦合补偿计算力矩
    float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1; // - I[0][0]*muR1;
    float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2; // - I[1][1]*muR2;
    float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3; // - I[2][2]*muR3;

    // 通过线性分配公式，将总推力和力矩分配到四个电机
    float w_sq[4];
    w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
    w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
    w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
    w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

    Control control;
    for (int i = 0; i < 4; i++)
    {
        // 将负的推力值截断为零，并通过平方根计算每个电机的转速
        if (w_sq[i] < 0)
            w_sq[i] = 0;

        control.rpm[i] = sqrtf(w_sq[i]);
    }
    return control;
}

static void
cmd_callback(const quadrotor_msgs::msg::SO3Command::ConstPtr &cmd)
{
    command.force[0] = cmd->force.x;
    command.force[1] = cmd->force.y;
    command.force[2] = cmd->force.z;
    command.qx = cmd->orientation.x;
    command.qy = cmd->orientation.y;
    command.qz = cmd->orientation.z;
    command.qw = cmd->orientation.w;
    command.kR[0] = cmd->kr[0];
    command.kR[1] = cmd->kr[1];
    command.kR[2] = cmd->kr[2];
    command.kOm[0] = cmd->kom[0];
    command.kOm[1] = cmd->kom[1];
    command.kOm[2] = cmd->kom[2];
    command.corrections[0] = cmd->aux.kf_correction;
    command.corrections[1] = cmd->aux.angle_corrections[0];
    command.corrections[2] = cmd->aux.angle_corrections[1];
    command.current_yaw = cmd->aux.current_yaw;
    command.use_external_yaw = cmd->aux.use_external_yaw;
}

static void
force_disturbance_callback(const geometry_msgs::msg::Vector3::ConstPtr &f)
{
    disturbance.f(0) = f->x;
    disturbance.f(1) = f->y;
    disturbance.f(2) = f->z;
}

static void
moment_disturbance_callback(const geometry_msgs::msg::Vector3::ConstPtr &m)
{
    disturbance.m(0) = m->x;
    disturbance.m(1) = m->y;
    disturbance.m(2) = m->z;
}

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State &state,
                    nav_msgs::msg::Odometry &odom)
{
    odom.pose.pose.position.x = state.x(0);
    odom.pose.pose.position.y = state.x(1);
    odom.pose.pose.position.z = state.x(2);

    Eigen::Quaterniond q(state.R);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = state.v(0);
    odom.twist.twist.linear.y = state.v(1);
    odom.twist.twist.linear.z = state.v(2);

    odom.twist.twist.angular.x = state.omega(0);
    odom.twist.twist.angular.y = state.omega(1);
    odom.twist.twist.angular.z = state.omega(2);
}

void quadToImuMsg(const QuadrotorSimulator::Quadrotor &quad, sensor_msgs::msg::Imu &imu)

{
    QuadrotorSimulator::Quadrotor::State state = quad.getState();
    Eigen::Quaterniond q(state.R);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();

    imu.angular_velocity.x = state.omega(0);
    imu.angular_velocity.y = state.omega(1);
    imu.angular_velocity.z = state.omega(2);

    imu.linear_acceleration.x = quad.getAcc()[0];
    imu.linear_acceleration.y = quad.getAcc()[1];
    imu.linear_acceleration.z = quad.getAcc()[2];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 初始化节点
    auto node = rclcpp::Node::make_shared("quadrotor_simulator_so3");

    // 发布者
    auto odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    auto imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    // 订阅者
    auto cmd_sub_ = node->create_subscription<quadrotor_msgs::msg::SO3Command>(
        "cmd", 100, cmd_callback);

    auto f_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
        "force_disturbance", 100, force_disturbance_callback);

    auto m_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
        "moment_disturbance", 100, moment_disturbance_callback);

    // 获取参数
    node->declare_parameter("simulator/init_state_x", 0.0);
    node->declare_parameter("simulator/init_state_y", 0.0);
    node->declare_parameter("simulator/init_state_z", 1.0);
    node->declare_parameter("rate/simulation", 1000.0);
    node->declare_parameter("rate/odom", 100.0);
    node->declare_parameter("quadrotor_name", "quadrotor");

    QuadrotorSimulator::Quadrotor quad;
    double _init_x, _init_y, _init_z;
    node->get_parameter("simulator/init_state_x", _init_x);
    node->get_parameter("simulator/init_state_y", _init_y);
    node->get_parameter("simulator/init_state_z", _init_z);


    Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
    quad.setStatePos(position);

    double simulation_rate;
    node->get_parameter("rate/simulation", simulation_rate);
    rcpputils::assert_true(simulation_rate > 0);

    double odom_rate;
    node->get_parameter("rate/odom", odom_rate);
    rclcpp::Duration odom_pub_duration(1.0 / odom_rate, 0);

    std::string quad_name;
    quad_name = node->get_parameter("quadrotor_name").as_string();

    QuadrotorSimulator::Quadrotor::State state = quad.getState();

    rclcpp::Rate r(simulation_rate);
    const double dt = 1.0 / simulation_rate;

    Control control;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "/world";
    odom_msg.child_frame_id = "/" + quad_name;

    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "/simulator";

    rclcpp::Time next_odom_pub_time = node->now();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        auto last = control;
        control = getControl(quad, command);
        for (int i = 0; i < 4; ++i)
        {
            //! @bug might have nan when the input is legal
            if (std::isnan(control.rpm[i]))
            {
                control.rpm[i] = last.rpm[i];
            }
        }

        quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2], control.rpm[3]);
        quad.setExternalForce(disturbance.f);
        quad.setExternalMoment(disturbance.m);
        quad.step(dt);

        rclcpp::Time tnow = node->now();

        if (tnow >= next_odom_pub_time)
        {
            next_odom_pub_time += odom_pub_duration;
            odom_msg.header.stamp = tnow;
            auto state = quad.getState();
            stateToOdomMsg(state, odom_msg);
            quadToImuMsg(quad, imu);
            odom_pub_->publish(odom_msg);
            imu_pub_->publish(imu);
        }

        r.sleep();
    }
}