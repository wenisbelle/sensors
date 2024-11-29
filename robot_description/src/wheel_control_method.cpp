#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node {
private:
    double WHEEL_RADIUS = 0.15;
    double L = 0.35;
    double W = 0.20;
    float vx;
    float vy;
    float vz;
    double wheel_speed_left_front;
    double wheel_speed_right_front;
    double wheel_speed_left_back;
    double wheel_speed_right_back;


public:

  MoveRobot() : Node("wheel_control_method") {
    joints_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_forward_controller/commands", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MoveRobot::timer_callback, this));

    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MoveRobot::velocity_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback() {

    calculate_wheel_speeds(vx, vy, vz);
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<double> wheel_speed = {wheel_speed_left_front, wheel_speed_right_front, wheel_speed_left_back, wheel_speed_right_back};    
    wheel_speed.insert(wheel_speed.end(), wheel_speed.begin(), wheel_speed.end()); // Duplicate the wheel speeds to match the number of joints in the controller
    message.data = wheel_speed;
    joints_publisher_->publish(message);
    //RCLCPP_INFO(this->get_logger(), "Linear X: '%f'. Linear Y: '%f'. Angular:'%f'",vx, vy, vz);
  }

  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->angular.z;
    
  }

  void calculate_wheel_speeds(float vx, float vy, float wz) {
    wheel_speed_left_front = (vx - vy - (L+W) * wz) / WHEEL_RADIUS;
    wheel_speed_right_front = (vx + vy + (L+W) * wz) / WHEEL_RADIUS;
    wheel_speed_left_back = (vx + vy - (L+W) * wz)/ WHEEL_RADIUS;
    wheel_speed_right_back = (vx - vy + (L+W) * wz) / WHEEL_RADIUS;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}