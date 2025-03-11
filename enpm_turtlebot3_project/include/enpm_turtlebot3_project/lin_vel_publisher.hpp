#ifndef LIN_VEL_PUBLISHER_HPP_
#define LIN_VEL_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define MAX_LINEAR_VELOCITY 1.0
#define MIN_LINEAR_VELOCITY 0.0
#define VELOCITY_STEP 0.1

class LinVelPublisher : public rclcpp::Node
{
public:
  LinVelPublisher();

private:
  // ROS2 Publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lin_vel_pub_;

  // ROS2 Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;

  // Velocity variable
  double linear_velocity_;

  // Functions
  void publish_velocity();
  void keyboard_control();
  int get_key();
};

#endif  // LIN_VEL_PUBLISHER_HPP_
