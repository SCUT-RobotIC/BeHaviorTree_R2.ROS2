#include "robot_arm/robot_arm.hpp"
#include <algorithm>
#include <sstream>

RobotArmNode::RobotArmNode() : Node("robot_arm_node") {
    // Parameters
    this->declare_parameter<double>("L1", 0.55); // Length of first link (m)
    this->declare_parameter<double>("L2", 0.400); // Length of second link (m)
    this->declare_parameter<double>("L3", 0.070); // Offset (m)
    this->declare_parameter<double>("relate_height", 0.175); // Relative height (m)
    this->declare_parameter<double>("r_buttom_min", 0.364); // 底座避障半径 (m)
    this->declare_parameter<double>("Angle1_MIN", 48.86); // 度
    this->declare_parameter<double>("Angle1_MAX", 140.7); // 度
    this->declare_parameter<double>("Angle2_MIN", 48.86); // 度
    this->declare_parameter<double>("Angle2_MAX", 140.7); // 度
    this->declare_parameter<double>("Angle3_MIN", 48.86); // 度
    this->declare_parameter<double>("Angle3_MAX", 140.7); // 度
    this->declare_parameter<double>("t_x", 0.18722); // 相机原点在底盘坐标系下 x (m)
    this->declare_parameter<double>("t_y", 0.2525); // 相机原点在底盘坐标系下 y (m)
    this->declare_parameter<double>("t_z", 0.13197); // 相机原点在底盘坐标系下 z (m)
    this->declare_parameter<double>("r_ang", 30); // 旋转角度
    
    this->get_parameter("L1", L1_);
    this->get_parameter("L2", L2_);
    this->get_parameter("L3", L3_);
    this->get_parameter("relate_height", relate_height_);
    this->get_parameter("r_buttom_min", r_buttom_min);
    this->get_parameter("Angle1_MIN", Angle1_MIN_);
    this->get_parameter("Angle1_MAX", Angle1_MAX_);
    this->get_parameter("Angle2_MIN", Angle2_MIN_);
    this->get_parameter("Angle2_MAX", Angle2_MAX_);
    this->get_parameter("Angle3_MIN", Angle3_MIN_);
    this->get_parameter("Angle3_MAX", Angle3_MAX_);
    this->get_parameter("t_x", t_x);
    this->get_parameter("t_y", t_y);
    this->get_parameter("t_z", t_z);
    this->get_parameter("r_ang", r_ang); 

    // 当前关节状态 (用于插值起始点)
    current_theta1_ = 0; // 默认初始位置
    current_theta2_ = 0;
    current_theta3_ = 0;
    current_yaw_ = 0.0;
    steps_ = 10;

    T[0][3] = t_x;
    T[1][3] = t_y;
    T[2][3] = t_z;

    const double pitch_rad = r_ang * M_PI / 180.0;
    Rotation[0][0] = cos(pitch_rad);
    Rotation[0][2] = sin(pitch_rad);
    Rotation[2][0] = -sin(pitch_rad);
    Rotation[2][2] = cos(pitch_rad);

    arm_solve_service_ = this->create_service<ArmSolve>(
        "solve_arm_ik", 
        std::bind(&RobotArmNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("arm_angles", 10);

    RCLCPP_INFO(this->get_logger(), "Robot Arm IK Service Ready.");
    RCLCPP_INFO(
        this->get_logger(),
        "Extrinsic set: t=(%.5f, %.5f, %.5f) m, pitch_y=%.2f deg, x_cam_in_base=(%.4f, %.4f, %.4f)",
        t_x, t_y, t_z, r_ang,
        Rotation[0][0], Rotation[1][0], Rotation[2][0]
    );
}

// 五次多项式插值算法
double RobotArmNode::quintic_interp(double start, double end, double t) {
    if (t <= 0.0) return start;
    if (t >= 1.0) return end;
    // s(t) = 10t^3 - 15t^4 + 6t^5
    return start + (end - start) * (t * t * t * (10.0 - 15.0 * t + 6.0 * t * t));
}

void RobotArmNode::handle_service(const std::shared_ptr<ArmSolve::Request> request,
                    std::shared_ptr<ArmSolve::Response> response) {
    // 获取请求参数（KFS坐标）
    const double x_cam = request->x;
    const double y_cam = request->y;
    const double z_cam = request->z;

    // p_arm = R * p_cam + t
    double x1 = Rotation[0][0] * x_cam + Rotation[0][2] * z_cam + T[0][3];
    double y1 = y_cam + T[1][3];
    double z1 = Rotation[2][0] * x_cam + Rotation[2][2] * z_cam + T[2][3];

    double r_buttom = hypot(x1, y1);
    RCLCPP_INFO(
        this->get_logger(),
        "IK input cam=(%.4f, %.4f, %.4f) -> base=(%.4f, %.4f, %.4f), r_base=%.4f, r_min=%.4f",
        x_cam, y_cam, z_cam, x1, y1, z1, r_buttom, r_buttom_min
    );

    if (r_buttom < r_buttom_min) {
        response->success = false;
        response->message = "Target too close to base (r_buttom limit).";
        return;
    }

    // 解算
    double yaw_rad = std::atan2(y1, x1);
    // x,y是第二根连杆的末端位置
    double x = x1;
    double y = y1;
    double z = z1- (L3_ + relate_height_);
    RCLCPP_INFO(this->get_logger(), "IK input: x=%.4f, y=%.4f, z=%.4f", x, y, z);
    double yaw_deg = yaw_rad * 180.0 / M_PI;

    double r = std::hypot(x, y);
    double d = std::hypot(r, z); 

    double L_max = std::sqrt(L1_*L1_ + L2_*L2_ - 2.0 * L1_ * L2_ * std::cos(Angle2_MAX_ * M_PI / 180.0));
    double L_min = std::sqrt(L1_*L1_ + L2_*L2_ - 2.0 * L1_ * L2_ * std::cos(Angle2_MIN_ * M_PI / 180.0));

    if (d > L_max + 1e-6 || d < L_min - 1e-6) {
        response->success = false;
        std::ostringstream oss;
        oss << "Target out of reach (distance check): d=" << d
            << " m, r=" << r
            << " m, z=" << z
            << " m, allowed=[" << L_min << ", " << L_max << "] m";
        response->message = oss.str();
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    double c2 = (L1_*L1_ + L2_*L2_ - d*d) / (2.0 * L1_ * L2_);
    c2 = std::clamp(c2, -1.0, 1.0);
    double theta2 = std::acos(c2);

    // 判断限位
    const double Angle2_MIN = Angle2_MIN_ * M_PI / 180.0;
    const double Angle2_MAX = Angle2_MAX_ * M_PI / 180.0;
    if (!(theta2 >= Angle2_MIN - 1e-9 && theta2 <= Angle2_MAX + 1e-9)) {
        response->success = false;
        response->message = "Theta2 out of range.";
        return;
    }

    double beta = std::atan2(z, r);
    double k1 = L1_ - L2_ * c2;
    double k2 = L2_ * std::sin(theta2);
    double theta1 = beta + std::atan2(k2, k1);

    // 判断限位
    const double Angle1_MIN = Angle1_MIN_ * M_PI / 180.0;
    const double Angle1_MAX = Angle1_MAX_ * M_PI / 180.0;
    if (!(theta1 >= Angle1_MIN - 1e-9 && theta1 <= Angle1_MAX + 1e-9)) {
        response->success = false;
        response->message = "Theta1 out of range.";
        return;
    }

    double theta1_deg = theta1 * 180.0 / M_PI;
    double theta2_deg = theta2 * 180.0 / M_PI;
    double theta3_deg = 270 - theta1_deg - theta2_deg; 

    // 判断限位
    const double Angle3_MIN = Angle3_MIN_ ;    
    const double Angle3_MAX = Angle3_MAX_ ;
    if (!(theta3_deg >= Angle3_MIN - 1e-9 && theta3_deg <= Angle3_MAX + 1e-9)) {
        response->success = false;
        response->message = "Theta3 out of range.";
        return;
    }

    if (theta2_deg > 90.0) {
        theta2_deg = theta2_deg - 90;
    }
    else {
        theta2_deg = 90 - theta2_deg;
    }
    theta2_deg = theta2_deg + theta1_deg;
    if (theta3_deg > 10.8) {
        theta3_deg = theta3_deg - 10.8;
    } 
    else {
        theta3_deg = 10.8 - theta3_deg;
    }

    std::vector<double> angles[4];  
    for (int i = 1; i <= steps_; i++) {
        double t = static_cast<double>(i) / steps_;
        double interp_theta1 = quintic_interp(current_theta1_, theta1_deg, t);
        double interp_theta2 = quintic_interp(current_theta2_, theta2_deg, t);
        double interp_theta3 = quintic_interp(current_theta3_, theta3_deg, t);
        double interp_yaw = quintic_interp(current_yaw_, yaw_deg, t);
        
        angles[0].push_back(static_cast<int>(interp_theta1));
        angles[1].push_back(static_cast<int>(interp_theta2));
        angles[2].push_back(static_cast<int>(interp_theta3));
        angles[3].push_back(static_cast<int>(interp_yaw));
        RCLCPP_INFO(this->get_logger(), "Step %d: Theta1=%.2f, Theta2=%.2f, Theta3=%.2f, Yaw=%.2f",
                    i, interp_theta1, interp_theta2, interp_theta3, interp_yaw);
    }

    current_theta1_ = theta1_deg;
    current_theta2_ = theta2_deg;
    current_theta3_ = theta3_deg;
    current_yaw_ = yaw_deg;
    response->success = true;
    response->theta_sum1 = angles[0];
    response->theta_sum2 = angles[1];
    response->theta_sum3 = angles[2];
    response->theta_sum_yaw = angles[3];
    response->message = "Solved.";

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArmNode>());
    rclcpp::shutdown();
    return 0;
}
