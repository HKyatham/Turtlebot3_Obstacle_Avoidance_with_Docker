#include "enpm_turtlebot3_project/lin_vel_publisher.hpp"

using namespace std::chrono_literals;

LinVelPublisher::LinVelPublisher()
: Node("lin_vel_publisher"), linear_velocity_(0.5)  // Start with 0.5 m/s
{
  lin_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lin_vel", 10);

  // Create timers
  timer_ = this->create_wall_timer(500ms, std::bind(&LinVelPublisher::publish_velocity, this));
  keyboard_timer_ = this->create_wall_timer(100ms, std::bind(&LinVelPublisher::keyboard_control, this));

  RCLCPP_INFO(this->get_logger(), "LinVelPublisher Node Started");
}

void LinVelPublisher::publish_velocity()
{
  auto msg = std_msgs::msg::Float32();
  msg.data = linear_velocity_;
  lin_vel_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published Linear Velocity: %.2f", linear_velocity_);
}

void LinVelPublisher::keyboard_control()
{
  int key = get_key();
  if (key == 'w') {
    if (linear_velocity_ + VELOCITY_STEP <= MAX_LINEAR_VELOCITY) {
      linear_velocity_ += VELOCITY_STEP;
    }
  } else if (key == 's') {
    if (linear_velocity_ - VELOCITY_STEP >= MIN_LINEAR_VELOCITY) {
      linear_velocity_ -= VELOCITY_STEP;
    }
  }
}

int LinVelPublisher::get_key()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinVelPublisher>());
  rclcpp::shutdown();
  return 0;
}
