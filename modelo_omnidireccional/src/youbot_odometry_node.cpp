#include <rclcpp/rclcpp.hpp>
#include "youbot_odometry.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin((std::make_shared<robmovil::YoubotOdometry>()));
  rclcpp::shutdown();
  return 0;
}
