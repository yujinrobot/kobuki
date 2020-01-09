#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "kobuki_velocity_smoother/velocity_smoother.hpp"

int main(int argc, char** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<kobuki_velocity_smoother::VelocitySmoother>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
