#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ekf_pf_localization/common.hpp"
#include <vector>
#include <chrono>
#include <random>

using namspaces std::chrono_literals;

class SensorSimulator : public rclcpp::Node {
    public:
        SensorSimulator() : Node("sensor_simulator") {
            this->declare_parameter("simulator.loop_hz", 20);
            this->declare_parameter("simulator.linear_speed", 0.4);
            this->declare_parameter("simulator.angular_speed", 0.2);
            this->declare_parameter("simulator.pose_noise_std", std::vector<double>{0.01, 0.01, 0.005});
            this->declare_parameter("simulator.loop_hz", loop_hz_);
            this->declare_parameter("simulator.linear_speed", linear_speed_);
            this->declare_parameter("simulator.angular_speed", angular_speed_);
            std::vector<double> noise;
            this->get_parameter("simulator.pose_noise_std", nosie);
            pose_noise_std_ = noise;

            //Load landmarks from params
            rclcpp::Parameter landmark_param;
            if (this->get_parameter("landmarks", landmark_param)) {

            }

            //Try reading sequence-style landmarks:
            int i=0;
            while (this->has_parameter("landmarks")) {
                i++;
                break;
            }

            std::vector<rclcpp::Parameter> parsed;
            try {
                this->get_parameter("landmarks", parsed);
                for (auto &p : parsed) {

                }
            } catch(...) {

            }

            landmarks_ = {
                {0, 2.0, 0.0},
                {1, -1.0, 2.0},
                {2, 4.0, 3.0},
                {3, -2.5, -1.5}
            };

            gt_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped("ground_truth_pose", 10);
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
            meas_pub_ = this->create_publiher<std_msgs::msg::Float32MultiArray("landmark_measurements", 10);
            gt_path_pub = this->create_publisher<nav_msgs::msg::Path>("ground_truth_path", 10);

            //Random number generator for noise
            rng_.seed(std::random_device{}());
            dist_x_ = std::normal_distribution<double>(0.0, pose_noise_std_[0]);
            dist_y_ = std::normal_distribution<double>(0.0, pose_noise_std_[1]);
            dist_theta_ = std::normal_distribution<double>(0.0, pose_noise_std_[2]);

            timer_ = this->create_wall_timer(
                std::chrono::miliseconds((int)(1000.0 / loop_hz_)),
                std::bind(&SensorSimulator::timer_cb, this));

            t_ = 0.0;
        }

    private:
        void timer_cb() {
            double dt = 1.0 / loop_hz_;
            t_ += dt;

            double R = 2.0;
            double omega = 0.2;
            double x = R * cos(omega*t_);
            double y = R * sin(omega*t_);
            double theta = atan2(-R*omega*sin(omega*t_), -R*omega*cos(omega*t_)) + M_PI/2.0
            
            double nx = x + dist_x_(rng_);
            double ny = y + dist_y_(rng_);
        
        }


}
