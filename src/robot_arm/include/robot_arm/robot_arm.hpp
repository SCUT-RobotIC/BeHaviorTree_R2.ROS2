#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP

#include <rclcpp/rclcpp.hpp>
#include "pose_interfaces/srv/arm_solve.hpp"
#include <cmath>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>

using ArmSolve = pose_interfaces::srv::ArmSolve;

class RobotArmNode : public rclcpp::Node {
public:
    RobotArmNode();

private:
    double L1_;
    double L2_;
    double L3_;
    double relate_height_;
    double Angle1_MIN_;
    double Angle1_MAX_;
    double Angle2_MIN_;
    double Angle2_MAX_;
    double Angle3_MIN_;
    double Angle3_MAX_;

    // 当前关节状态 (用于插值起始点)
    double current_theta1_; 
    double current_theta2_;
    double current_theta3_;
    double current_yaw_;
    int steps_;

    double t_x;
    double t_y;
    double t_z;
    double r_ang;
    double T[4][4] = { 
                        {1, 0, 0, 0},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1} 
                    };
    double Rotation[4][4] = { 
                                {1, 0, 0, 0},
                                {0, 1, 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1} 
                            };

    double r_buttom_min;

    // 服务声明
    rclcpp::Service<ArmSolve>::SharedPtr arm_solve_service_;

    // 五次多项式插值算法
    double quintic_interp(double start, double end, double t);

    // 结果发布者声明
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angle_pub_;

    // 服务处理函数
    void handle_service(const std::shared_ptr<ArmSolve::Request> request,
                        std::shared_ptr<ArmSolve::Response> response);
};

#endif // ROBOT_ARM_HPP
