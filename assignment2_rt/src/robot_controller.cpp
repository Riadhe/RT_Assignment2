#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// Include custom message and service headers
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
        // 1. Communication Setup 
        // Subscriber for USER commands 
        // We listen to '/user_cmd' instead of '/cmd_vel' directly. 
        // This allows this node to decide IF the command is safe to forward to the wheels.
        user_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/user_cmd", 10, std::bind(&RobotController::user_cmd_callback, this, _1));
        
        // Subscriber for Laser Scan (The Safety Sensor)
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RobotController::scan_callback, this, _1));
        
        // Publisher for Wheel Commands 
        // We publish to '/cmd_vel' only when it is safe.
        vel_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Publisher for the Custom Message (Assignment Requirement)
        state_pub_ = create_publisher<assignment2_custom_msgs::msg::RobotState>("/robot_state", 10);
        
        // 2. Service Servers
        // Service to calculate average velocity
        srv_avg_ = create_service<assignment2_custom_msgs::srv::GetAvgVel>(
            "get_avg_vel", std::bind(&RobotController::get_avg, this, _1, _2));
            
        // Service to dynamically change the safety threshold
        srv_thr_ = create_service<assignment2_custom_msgs::srv::ChangeThreshold>(
            "change_threshold", std::bind(&RobotController::change_thr, this, _1, _2));
        
        RCLCPP_INFO(get_logger(), "Gatekeeper Controller Ready.");
    }

private:
    // Internal Variables
    float threshold_ = 1.0;          // Distance limit for obstacles
    bool backing_up_ = false;        // Safety mode flag
    std::deque<geometry_msgs::msg::Twist> history_; // Stores recent commands for averaging
    
    // ROS Objects
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr user_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<assignment2_custom_msgs::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Service<assignment2_custom_msgs::srv::GetAvgVel>::SharedPtr srv_avg_;
    rclcpp::Service<assignment2_custom_msgs::srv::ChangeThreshold>::SharedPtr srv_thr_;

    // CALLBACK 1: USER COMMANDS
    // This function receives commands from the Python UI.
    void user_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 1. Store the command in history (Queue of size 5)
        history_.push_back(*msg);
        if (history_.size() > 5) history_.pop_front();
    
        // IF the robot is NOT in "Safety Backup Mode" (backing_up_ == false),
        // THEN forward the user's command to the wheels.
        if (!backing_up_) {
            vel_pub_->publish(*msg);
        }
        // ELSE: If backing_up_ is true, we IGNORE the user command.
    }

    // CALLBACK 2: SAFETY LOGIC
    // This function processes laser data to detect obstacles.
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. Find the closest obstacle
        float min_dist = 100.0;
        int idx = -1;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            // Check for valid range readings (ignore infinity or errors)
            if (msg->ranges[i] < min_dist && msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
                min_dist = msg->ranges[i];
                idx = i;
            }
        }

        // 2. Determine obstacle direction (Right, Front, Left) based on index
        std::string dir = "Unknown";
        if (idx != -1) {
            if (idx < (int)msg->ranges.size() / 3) dir = "Right";
            else if (idx < (int)msg->ranges.size() * 2 / 3) dir = "Front";
            else dir = "Left";
        }

        // 3. Publish Custom Message (RobotState)
        auto state = assignment2_custom_msgs::msg::RobotState();
        state.distance = min_dist; 
        state.direction = dir; 
        state.threshold = threshold_;
        state_pub_->publish(state);

        // 4. SAFETY LOGIC 
        // TRIGGER: If obstacle is closer than threshold, activate backup mode.
        if (min_dist < threshold_) {
            backing_up_ = true;
        }

        // ACTION: If backup mode is active, take control of the robot.
        if (backing_up_) {
            // Buffer Zone Check: Keep backing up until we are SAFELY away (+0.5m)
            if (min_dist > (threshold_ + 0.5)) {
                // We have backed up enough. Disable safety mode.
                backing_up_ = false; // Exit safety mode
            } 
            else {
                // FORCE the robot to move backward.
                geometry_msgs::msg::Twist safety_cmd;
                
                if (dir == "Front") {
                    safety_cmd.linear.x = -0.5; // Reverse
                    safety_cmd.angular.z = 0.0;
                } else {
                    // If obstacle is to the side, just stop (or you can add turning logic)
                    safety_cmd.linear.x = 0.0;
                    safety_cmd.angular.z = 0.0; 
                }                
                // Publish directly to wheels to OVERRIDE user commands
                vel_pub_->publish(safety_cmd);
            }
        }
    }

    // SERVICE 1: GET AVERAGE VELOCITY 
    void get_avg(const std::shared_ptr<assignment2_custom_msgs::srv::GetAvgVel::Request>,
                 std::shared_ptr<assignment2_custom_msgs::srv::GetAvgVel::Response> res) {
        if (history_.empty()) return;
        
        float sum_l = 0, sum_a = 0;
        // Iterate through the deque to sum linear and angular velocities
        for (auto v : history_) { 
            sum_l += v.linear.x; 
            sum_a += v.angular.z; 
        }
        
        // Calculate averages
        res->avg_linear = sum_l / history_.size();
        res->avg_angular = sum_a / history_.size();
    }

    // SERVICE 2: CHANGE THRESHOLD 
    void change_thr(const std::shared_ptr<assignment2_custom_msgs::srv::ChangeThreshold::Request> req,
                    std::shared_ptr<assignment2_custom_msgs::srv::ChangeThreshold::Response> res) {
        // Update the threshold variable with the user's request
        threshold_ = req->new_threshold;
        res->success = true;
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // Spin the node to process callbacks
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
