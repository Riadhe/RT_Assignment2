#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "assignment2_custom_msgs/msg/robot_state.hpp"
#include "assignment2_custom_msgs/srv/get_avg_vel.hpp"
#include "assignment2_custom_msgs/srv/change_threshold.hpp"
#include <deque>

using namespace std::chrono_literals;
using std::placeholders::_1; 
using std::placeholders::_2;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        // 1. Setup Communication (Using 'auto' to keep it short)
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&RobotController::scan_callback, this, _1));
        vel_sub_  = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&RobotController::vel_callback, this, _1));
        
        vel_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_pub_ = create_publisher<assignment2_custom_msgs::msg::RobotState>("/robot_state", 10);
        
        srv_avg_ = create_service<assignment2_custom_msgs::srv::GetAvgVel>("get_avg_vel", std::bind(&RobotController::get_avg, this, _1, _2));
        srv_thr_ = create_service<assignment2_custom_msgs::srv::ChangeThreshold>("change_threshold", std::bind(&RobotController::change_thr, this, _1, _2));
        
        RCLCPP_INFO(get_logger(), "Controller Ready.");
    }

private:
    float threshold_ = 1.0;
    std::deque<geometry_msgs::msg::Twist> history_;
    
    // ROS Objects (We declare them here)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<assignment2_custom_msgs::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Service<assignment2_custom_msgs::srv::GetAvgVel>::SharedPtr srv_avg_;
    rclcpp::Service<assignment2_custom_msgs::srv::ChangeThreshold>::SharedPtr srv_thr_;

    // --- MAIN LOGIC ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. Find closest point
        float min_dist = 100.0;
        int idx = -1;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            if (msg->ranges[i] < min_dist && msg->ranges[i] > msg->range_min) {
                min_dist = msg->ranges[i];
                idx = i;
            }
        }

        // 2. Determine Direction
        std::string dir = "Unknown";
        if (idx != -1) {
            if (idx < (int)msg->ranges.size() / 3) dir = "Right";
            else if (idx < (int)msg->ranges.size() * 2 / 3) dir = "Front";
            else dir = "Left";
        }

        // 3. Publish Custom Message
        auto state = assignment2_custom_msgs::msg::RobotState();
        state.distance = min_dist; state.direction = dir; state.threshold = threshold_;
        state_pub_->publish(state);

        // 4. Safety Logic (Simplified)
        if (min_dist < threshold_) {
            geometry_msgs::msg::Twist stop_cmd;
            // Only go back if the wall is in FRONT. Otherwise just stop to avoid crashing behind.
            if (dir == "Front") {
                stop_cmd.linear.x = -0.5; // Go back
            } else {
                stop_cmd.linear.x = 0.0;  // Just stop
                stop_cmd.angular.z = 0.5; // Turn a little
            }
            vel_pub_->publish(stop_cmd);
        }
    }

    // Store Velocity for Average Calculation
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        history_.push_back(*msg);
        if (history_.size() > 5) history_.pop_front();
    }

    // Service 1: Calculate Average
    void get_avg(const std::shared_ptr<assignment2_custom_msgs::srv::GetAvgVel::Request>,
                 std::shared_ptr<assignment2_custom_msgs::srv::GetAvgVel::Response> res) {
        if (history_.empty()) return;
        float sum_l = 0, sum_a = 0;
        for (auto v : history_) { sum_l += v.linear.x; sum_a += v.angular.z; }
        res->avg_linear = sum_l / history_.size();
        res->avg_angular = sum_a / history_.size();
    }

    // Service 2: Change Threshold
    void change_thr(const std::shared_ptr<assignment2_custom_msgs::srv::ChangeThreshold::Request> req,
                    std::shared_ptr<assignment2_custom_msgs::srv::ChangeThreshold::Response> res) {
        threshold_ = req->new_threshold;
        res->success = true;
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}