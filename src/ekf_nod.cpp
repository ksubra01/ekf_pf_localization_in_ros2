#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "ekf_pf_localization/common.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;

class EKFNode : public rclcpp::Node {
public:
  EKFNode() : Node("ekf_node") {
    this->declare_parameter("ekf.meas_range_std", 0.08);
    this->declare_parameter("ekf.meas_bearing_std", 0.05);
    this->declare_parameter("ekf.motion_noise", std::vector<double>{0.01,0.01,0.01});

    this->get_parameter("ekf.meas_range_std", meas_range_std_);
    this->get_parameter("ekf.meas_bearing_std", meas_bearing_std_);
    this->get_parameter("ekf.motion_noise", motion_noise_);

    // initial state
    x_.setZero();
    P_.setIdentity() * 0.1;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, std::bind(&EKFNode::odom_cb, this, std::placeholders::_1));
    meas_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "landmark_measurements", 10, std::bind(&EKFNode::meas_cb, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);

    // landmarks (same as simulator)
    landmarks_ = {
      {0, 2.0, 0.0},
      {1, -1.0, 2.0},
      {2, 4.0, 3.0},
      {3, -2.5, -1.5}
    };
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Use odom pose as noisy motion observation; do a simple EKF predict assuming small dt
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);

    // Simple motion update: set state to odom with process noise (this is a simple filter)
    // A more correct approach: compute delta and propagate, but for clarity we do a small predictive covariance increase
    Vector3d z;
    z << x, y, yaw;
    // prediction: x = x (no change) but enlarge covariance
    Matrix3d Q = Matrix3d::Zero();
    Q(0,0) = motion_noise_[0];
    Q(1,1) = motion_noise_[1];
    Q(2,2) = motion_noise_[2];
    P_ = P_ + Q;

    // Do a moderate "odometry" measurement update (interpret odom as control+measurement)
    // H = I
    Matrix3d H = Matrix3d::Identity();
    Matrix3d R = Matrix3d::Zero();
    R(0,0) = 0.05; R(1,1) = 0.05; R(2,2) = 0.02;
    Vector3d yres = z - x_;
    // normalize yaw error to [-pi,pi]
    yres(2) = normalize_angle(yres(2));
    Matrix3d S = H * P_ * H.transpose() + R;
    Matrix3d K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * yres;
    P_ = (Matrix3d::Identity() - K*H) * P_;

    publish_state(msg->header.stamp);
  }

  void meas_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // message format: repeated [id, range, bearing]
    auto &data = msg->data;
    for (size_t i=0;i+2 < data.size(); i += 3) {
      int id = (int)data[i];
      double rng = data[i+1];
      double bearing = data[i+2];
      // find landmark
      Landmark lm;
      bool found = false;
      for (auto &L : landmarks_) {
        if (L.id == id) { lm = L; found=true; break;}
      }
      if (!found) continue;

      // measurement model h(x) = [range; bearing]
      double dx = lm.x - x_(0);
      double dy = lm.y - x_(1);
      double expected_range = std::hypot(dx,dy);
      double expected_bearing = std::atan2(dy,dx) - x_(2);
      // build Jacobian H (2x3)
      Eigen::Matrix<double,2,3> H;
      double q = dx*dx + dy*dy;
      double sqrtq = std::sqrt(q);
      H(0,0) = -dx / sqrtq;
      H(0,1) = -dy / sqrtq;
      H(0,2) = 0.0;
      H(1,0) = dy / q;
      H(1,1) = -dx / q;
      H(1,2) = -1.0;

      // measurement covariance
      Eigen::Matrix2d R;
      R << meas_range_std_*meas_range_std_, 0,
           0, meas_bearing_std_*meas_bearing_std_;

      // innovation
      Eigen::Vector2d z;
      z << rng, bearing;
      Eigen::Vector2d h;
      h << expected_range, expected_bearing;
      Eigen::Vector2d yres = z - h;
      yres(1) = normalize_angle(yres(1));

      Eigen::Matrix2d S = H * P_ * H.transpose() + R;
      Eigen::Matrix<double,3,2> K = P_ * H.transpose() * S.inverse();

      x_ = x_ + K * yres;
      P_ = (Matrix3d::Identity() - K * H) * P_;
    }
  }

  void publish_state(const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = "map";
    p.pose.position.x = x_(0);
    p.pose.position.y = x_(1);
    tf2::Quaternion q;
    q.setRPY(0,0,x_(2));
    p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w();
    pose_pub_->publish(p);

    // path (single point per publish for simplicity)
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    path.poses.push_back(p);
    path_pub_->publish(path);
  }

  static double normalize_angle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr meas_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  Vector3d x_;
  Matrix3d P_;
  double meas_range_std_, meas_bearing_std_;
  std::vector<double> motion_noise_;
  std::vector<Landmark> landmarks_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
