#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

enum class State {
    INIT,
    WAITING_FOR_POSE, // New state to handle teleport lag
    TURNING,
    MOVING,
    STOPPED
};

struct Waypoint {
    float x;
    float y;
};

enum class TrajectoryType {
    SQUARE,
    TRIANGLE
};

class TurtleController : public rclcpp::Node
{
public:
    TurtleController(const std::string& turtle_name)
    : Node("turtle_controller"), turtle_name_(turtle_name)
    {
        state_ = State::INIT;
        
        // Define trajectories (Explicitly listing all points to visit)
        // Square: Start 1,1 -> 1,9 -> 9,9 -> 9,1 -> 1,1 (Close loop)
        square_trajectory_ = {{1.0, 9.0}, {9.0, 9.0}, {9.0, 1.0}, {1.0, 1.0}};
        
        // Triangle: Start 1,1 -> 9,1 -> 5,9 -> 1,1 (Close loop)
        triangular_trajectory_ = {{9.0, 1.0}, {5.0, 9.0}, {1.0, 1.0}};

        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // Timer to start the chain
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TurtleController::init, this));
    }

private:
    void init()
    {
        init_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Starting initialization...");
        kill_turtle("turtle1");
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;

        // 1. CRITICAL FIX: Handle Stale Data
        // If we just teleported, ignore old pose messages until we see the robot at the new location
        if (state_ == State::WAITING_FOR_POSE) {
            float dist_to_teleport = std::sqrt(
                std::pow(current_pose_.x - expected_pose_.x, 2) + 
                std::pow(current_pose_.y - expected_pose_.y, 2));

            if (dist_to_teleport < 0.5) {
                RCLCPP_INFO(this->get_logger(), "Pose stabilized. Starting trajectory.");
                state_ = State::TURNING;
            } else {
                return; // Ignore this callback, it's stale data
            }
        }

        if (state_ == State::STOPPED || state_ == State::INIT) {
            return;
        }

        // 2. Control Logic
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        // Get current target
        Waypoint target = active_trajectory_[waypoint_index_];
        
        float dx = target.x - current_pose_.x;
        float dy = target.y - current_pose_.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        float target_angle = std::atan2(dy, dx);
        float angle_diff = target_angle - current_pose_.theta;

        // Normalize angle
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (state_ == State::TURNING) {
            if (std::abs(angle_diff) > 0.02) { // Tighter tolerance
                twist_msg->angular.z = (angle_diff > 0) ? 1.8 : -1.8;
                twist_msg->linear.x = 0.0;
            } else {
                state_ = State::MOVING;
                twist_msg->angular.z = 0.0;
            }
        } 
        else if (state_ == State::MOVING) {
            // Simple P-controller for angle correction while moving
            twist_msg->angular.z = angle_diff * 2.0; 

            if (distance > 0.1) {
                twist_msg->linear.x = 2.0;
            } else {
                // Waypoint reached
                twist_msg->linear.x = 0.0;
                twist_msg->angular.z = 0.0;
                
                waypoint_index_++;
                
                // Check if trajectory finished
                if (waypoint_index_ >= active_trajectory_.size()) {
                    handle_trajectory_completion();
                } else {
                    state_ = State::TURNING;
                }
            }
        }
        
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    void handle_trajectory_completion() {
        if (current_trajectory_type_ == TrajectoryType::SQUARE) {
            RCLCPP_INFO(this->get_logger(), "Square Done. Teleporting to start of Triangle.");
            
            // Switch to Triangle
            current_trajectory_type_ = TrajectoryType::TRIANGLE;
            active_trajectory_ = triangular_trajectory_;
            waypoint_index_ = 0;
            
            // Teleport to (1,1) to ensure clean start
            teleport_to(1.0, 1.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "Triangle Done. Stopping.");
            state_ = State::STOPPED;
        }
    }

    // --- Service Clients ---

    void kill_turtle(const std::string& name) {
        if (!kill_client_->wait_for_service(std::chrono::seconds(2))) return;
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        kill_client_->async_send_request(request, [this](std::shared_future<std::shared_ptr<turtlesim::srv::Kill::Response>> future) {
            (void)future;
            this->spawn_turtle(this->turtle_name_, 5.5, 5.5, 0.0);
        });
    }

    void spawn_turtle(const std::string& name, float x, float y, float theta) {
        if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) return;
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x; request->y = y; request->theta = theta; request->name = name;
        
        spawn_client_->async_send_request(request, [this](std::shared_future<std::shared_ptr<turtlesim::srv::Spawn::Response>> future) {
            RCLCPP_INFO(this->get_logger(), "Spawned %s", future.get()->name.c_str());
            
            // Init interfaces
            std::string ns = "/" + this->turtle_name_;
            this->teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>(ns + "/teleport_absolute");
            this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(ns + "/cmd_vel", 10);
            this->pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
                ns + "/pose", 10, std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));

            // Setup Square Trajectory
            this->active_trajectory_ = this->square_trajectory_;
            this->current_trajectory_type_ = TrajectoryType::SQUARE;
            this->waypoint_index_ = 0;

            // Teleport to start of Square (1,1)
            this->teleport_to(1.0, 1.0);
        });
    }

    void teleport_to(float x, float y) {
        if (!teleport_client_->wait_for_service(std::chrono::seconds(1))) return;
        
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = 0.0;

        // Set expectation for pose filtering
        expected_pose_.x = x;
        expected_pose_.y = y;
        
        // Stop processing logic until pose updates
        state_ = State::WAITING_FOR_POSE; 

        teleport_client_->async_send_request(request, [this](std::shared_future<std::shared_ptr<turtlesim::srv::TeleportAbsolute::Response>> future) {
            (void)future;
            RCLCPP_INFO(this->get_logger(), "Teleport requested...");
        });
    }

    std::string turtle_name_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;

    turtlesim::msg::Pose current_pose_;
    Waypoint expected_pose_; // To filter stale data
    
    State state_;
    std::vector<Waypoint> active_trajectory_;
    std::vector<Waypoint> square_trajectory_;
    std::vector<Waypoint> triangular_trajectory_;
    size_t waypoint_index_ = 0;
    TrajectoryType current_trajectory_type_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>("Turtle_Azat");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
