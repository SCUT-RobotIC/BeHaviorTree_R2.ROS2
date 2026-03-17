#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rmw/qos_profiles.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <small_gicp/registration/registration_helper.hpp>

class GlimToSmallGICPNode : public rclcpp::Node {
public:
  GlimToSmallGICPNode() : Node("glim2small_gicp_node") {
    target_ply_path_ = this->declare_parameter<std::string>("target_ply_path", "");
    source_map_topic_ = this->declare_parameter<std::string>("source_map_topic", "/glim_ros/map");
    source_map_qos_ = this->declare_parameter<std::string>("source_map_qos", "auto");
    source_pose_topic_ = this->declare_parameter<std::string>("source_pose_topic", "/glim_ros/pose_corrected");
    target_frame_id_ = this->declare_parameter<std::string>("target_frame_id", "target_map");
    output_pose_topic_ = this->declare_parameter<std::string>("output_pose_topic", "/glim2small_gicp/robot_pose");
    output_position_topic_ = this->declare_parameter<std::string>("output_position_topic", "/glim2small_gicp/robot_position");
    output_alignment_topic_ = this->declare_parameter<std::string>("output_alignment_topic", "/glim2small_gicp/source_map_pose");
    output_relocalized_pose_topic_ = this->declare_parameter<std::string>("output_relocalized_pose_topic", "/glim2small_gicp/relocalized_robot_pose");
    output_relocalized_position_topic_ = this->declare_parameter<std::string>("output_relocalized_position_topic", "/glim2small_gicp/relocalized_robot_position");
    num_threads_ = this->declare_parameter<int>("num_threads", 4);
    num_neighbors_ = this->declare_parameter<int>("num_neighbors", 20);
    min_source_points_ = this->declare_parameter<int>("min_source_points", 500);
    downsampling_resolution_ = this->declare_parameter<double>("downsampling_resolution", 0.25);
    max_correspondence_distance_ = this->declare_parameter<double>("max_correspondence_distance", 2.0);
    verbose_topic_log_ = this->declare_parameter<bool>("verbose_topic_log", true);
    initial_guess_x_ = this->declare_parameter<double>("initial_guess_x", 0.0);
    initial_guess_y_ = this->declare_parameter<double>("initial_guess_y", 0.0);
    initial_guess_z_ = this->declare_parameter<double>("initial_guess_z", 0.0);
    initial_guess_roll_ = this->declare_parameter<double>("initial_guess_roll", 0.0);
    initial_guess_pitch_ = this->declare_parameter<double>("initial_guess_pitch", 0.0);
    initial_guess_yaw_ = this->declare_parameter<double>("initial_guess_yaw", 0.0);

    if (target_ply_path_.empty()) {
      throw std::runtime_error("parameter 'target_ply_path' must be set");
    }

    load_target_map();

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, output_qos);
    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_position_topic_, output_qos);
    alignment_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_alignment_topic_, output_qos);
    relocalized_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_relocalized_pose_topic_, output_qos);
    relocalized_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_relocalized_position_topic_, output_qos);

    auto map_qos = create_source_map_qos();
    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      source_map_topic_, map_qos, std::bind(&GlimToSmallGICPNode::map_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      source_pose_topic_, 10, std::bind(&GlimToSmallGICPNode::pose_callback, this, std::placeholders::_1));

    status_timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&GlimToSmallGICPNode::status_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "target_ply_path: %s", target_ply_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "source_map_topic: %s", source_map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "source_map_qos: %s", source_map_qos_.c_str());
    RCLCPP_INFO(this->get_logger(), "source_pose_topic: %s", source_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "verbose_topic_log: %s", verbose_topic_log_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "continuous_pose_topic: %s", output_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "continuous_position_topic: %s", output_position_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "relocalized_pose_topic: %s", output_relocalized_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "relocalized_position_topic: %s", output_relocalized_position_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "initial_guess xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
      initial_guess_x_, initial_guess_y_, initial_guess_z_, initial_guess_roll_, initial_guess_pitch_, initial_guess_yaw_);
    RCLCPP_INFO(this->get_logger(), "loaded %zu target points", target_points_.size());
  }

private:
  rclcpp::QoS create_source_map_qos() const {
    std::string qos_mode = source_map_qos_;
    std::transform(qos_mode.begin(), qos_mode.end(), qos_mode.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (qos_mode == "auto") {
      qos_mode = source_map_topic_ == "/glim_ros/map" ? "transient_local" : "volatile";
    }

    if (qos_mode == "transient_local") {
      return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    }

    if (qos_mode == "sensor_data") {
      return rclcpp::SensorDataQoS();
    }

    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  }

  Eigen::Isometry3d initial_guess_transform() const {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(initial_guess_x_, initial_guess_y_, initial_guess_z_);

    const Eigen::AngleAxisd roll_angle(initial_guess_roll_, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(initial_guess_pitch_, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(initial_guess_yaw_, Eigen::Vector3d::UnitZ());
    transform.linear() = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    return transform;
  }

  static Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose& pose) {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    q.normalize();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = q.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return transform;
  }

  static geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d& transform) {
    Eigen::Quaterniond q(transform.linear());
    q.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  std::string translation_to_string(const Eigen::Isometry3d& transform) const {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "(" << transform.translation().x() << ", " << transform.translation().y() << ", " << transform.translation().z() << ")";
    return oss.str();
  }

  void status_timer_callback() {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      this->get_logger(),
      "status: map_received=%s pose_received=%s last_source_points=%zu last_filtered_points=%zu last_publish=%s last_pose_translation=%s",
      has_source_map_ ? "true" : "false",
      has_source_pose_ ? "true" : "false",
      last_source_width_,
      last_filtered_points_,
        has_map_alignment_ ? "true" : "false",
      has_source_pose_ ? translation_to_string(latest_source_pose_).c_str() : "n/a");
  }

  void load_target_map() {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    if (pcl::io::loadPLYFile(target_ply_path_, pcl_cloud) < 0) {
      throw std::runtime_error("failed to load PLY file: " + target_ply_path_);
    }

    target_points_.reserve(pcl_cloud.size());
    for (const auto& point : pcl_cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      target_points_.emplace_back(point.x, point.y, point.z, 1.0f);
    }

    if (target_points_.empty()) {
      throw std::runtime_error("target map is empty after filtering invalid points");
    }

    std::tie(target_cloud_, target_tree_) = small_gicp::preprocess_points(
      target_points_, downsampling_resolution_, num_neighbors_, num_threads_);
  }

  std::vector<Eigen::Vector4f> pointcloud2_to_points(const sensor_msgs::msg::PointCloud2& msg) const {
    std::vector<Eigen::Vector4f> points;
    points.reserve(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      points.emplace_back(*iter_x, *iter_y, *iter_z, 1.0f);
    }

    return points;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Eigen::Isometry3d latest_source_pose;
    Eigen::Isometry3d latest_alignment;
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_source_pose_ = pose_to_isometry(msg->pose);
      latest_source_pose_stamp_ = msg->header.stamp;
      has_source_pose_ = true;
      latest_source_pose = latest_source_pose_;
      latest_alignment = map_alignment_;
      should_publish = has_map_alignment_;
    }

    if (verbose_topic_log_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "received pose topic=%s stamp=%d.%09u translation=%s",
        source_pose_topic_.c_str(),
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        translation_to_string(latest_source_pose).c_str());
    }

    if (should_publish) {
      publish_robot_pose(latest_alignment * latest_source_pose, msg->header.stamp);
      if (verbose_topic_log_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "republished continuous robot pose from latest alignment using pose topic=%s",
          source_pose_topic_.c_str());
      }
    }
  }

  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_source_map_ = true;
      last_source_width_ = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    }

    if (verbose_topic_log_) {
      RCLCPP_INFO(
        this->get_logger(),
        "received map topic=%s stamp=%d.%09u width=%u height=%u frame_id=%s",
        source_map_topic_.c_str(),
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        msg->width,
        msg->height,
        msg->header.frame_id.c_str());
    }

    const auto source_points = pointcloud2_to_points(*msg);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_filtered_points_ = source_points.size();
    }

    if (verbose_topic_log_) {
      RCLCPP_INFO(this->get_logger(), "filtered source points: %zu", source_points.size());
    }

    if (source_points.size() < static_cast<size_t>(min_source_points_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "source map has only %zu points after filtering, waiting for more data", source_points.size());
      return;
    }

    auto [source_cloud, source_tree] = small_gicp::preprocess_points(
      source_points, downsampling_resolution_, num_neighbors_, num_threads_);

    small_gicp::RegistrationSetting setting;
    setting.num_threads = num_threads_;
    setting.max_correspondence_distance = max_correspondence_distance_;

    Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_map_alignment_) {
        init_guess = map_alignment_;
      } else {
        init_guess = initial_guess_transform();
      }
    }

    if (verbose_topic_log_) {
      RCLCPP_INFO(
        this->get_logger(),
        "starting registration target_points=%zu source_points=%zu init_translation=%s max_corr=%.3f",
        target_points_.size(),
        source_points.size(),
        translation_to_string(init_guess).c_str(),
        max_correspondence_distance_);
    }

    const auto result = small_gicp::align(*target_cloud_, *source_cloud, *target_tree_, init_guess, setting);

    if (!result.converged) {
      RCLCPP_WARN(
        this->get_logger(),
        "small_gicp registration did not converge topic=%s error=%.6f iterations=%zu inliers=%zu",
        source_map_topic_.c_str(),
        result.error,
        static_cast<size_t>(result.iterations),
        static_cast<size_t>(result.num_inliers));
      return;
    }

    Eigen::Isometry3d robot_in_target = result.T_target_source;
    bool has_pose = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_alignment_ = result.T_target_source;
      has_map_alignment_ = true;
      if (has_source_pose_) {
        robot_in_target = map_alignment_ * latest_source_pose_;
        has_pose = true;
      }
    }

    publish_alignment_pose(result.T_target_source, msg->header.stamp);
    publish_robot_pose(robot_in_target, msg->header.stamp);
    publish_relocalized_pose(robot_in_target, msg->header.stamp);

    RCLCPP_INFO(
      this->get_logger(),
      "published result pose_topic=%s position_topic=%s relocalized_pose_topic=%s relocalized_position_topic=%s alignment_topic=%s robot_translation=%s error=%.6f iterations=%zu inliers=%zu pose_source=%s",
      output_pose_topic_.c_str(),
      output_position_topic_.c_str(),
      output_relocalized_pose_topic_.c_str(),
      output_relocalized_position_topic_.c_str(),
      output_alignment_topic_.c_str(),
      translation_to_string(robot_in_target).c_str(),
      result.error,
      static_cast<size_t>(result.iterations),
      static_cast<size_t>(result.num_inliers),
      has_pose ? "glim_pose" : "source_map_origin");
  }

  void publish_alignment_pose(const Eigen::Isometry3d& alignment, const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = target_frame_id_;
    msg.pose = to_pose_msg(alignment);
    alignment_pub_->publish(msg);
  }

  void publish_robot_pose(const Eigen::Isometry3d& robot_pose, const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = target_frame_id_;
    pose_msg.pose = to_pose_msg(robot_pose);
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = pose_msg.header;
    point_msg.point.x = robot_pose.translation().x();
    point_msg.point.y = robot_pose.translation().y();
    point_msg.point.z = robot_pose.translation().z();
    position_pub_->publish(point_msg);
  }

  void publish_relocalized_pose(const Eigen::Isometry3d& robot_pose, const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = target_frame_id_;
    pose_msg.pose = to_pose_msg(robot_pose);
    relocalized_pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = pose_msg.header;
    point_msg.point.x = robot_pose.translation().x();
    point_msg.point.y = robot_pose.translation().y();
    point_msg.point.z = robot_pose.translation().z();
    relocalized_position_pub_->publish(point_msg);
  }

private:
  std::string target_ply_path_;
  std::string source_map_topic_;
  std::string source_map_qos_;
  std::string source_pose_topic_;
  std::string target_frame_id_;
  std::string output_pose_topic_;
  std::string output_position_topic_;
  std::string output_alignment_topic_;
  std::string output_relocalized_pose_topic_;
  std::string output_relocalized_position_topic_;
  bool verbose_topic_log_ = true;
  int num_threads_ = 4;
  int num_neighbors_ = 20;
  int min_source_points_ = 500;
  double downsampling_resolution_ = 0.25;
  double max_correspondence_distance_ = 2.0;
  double initial_guess_x_ = 0.0;
  double initial_guess_y_ = 0.0;
  double initial_guess_z_ = 0.0;
  double initial_guess_roll_ = 0.0;
  double initial_guess_pitch_ = 0.0;
  double initial_guess_yaw_ = 0.0;

  std::mutex mutex_;
  bool has_source_map_ = false;
  bool has_source_pose_ = false;
  bool has_map_alignment_ = false;
  size_t last_source_width_ = 0;
  size_t last_filtered_points_ = 0;
  builtin_interfaces::msg::Time latest_source_pose_stamp_;
  Eigen::Isometry3d latest_source_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_alignment_ = Eigen::Isometry3d::Identity();

  std::vector<Eigen::Vector4f> target_points_;
  small_gicp::PointCloud::Ptr target_cloud_;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr target_tree_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr alignment_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relocalized_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr relocalized_position_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GlimToSmallGICPNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "glim2small_gicp_node failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}