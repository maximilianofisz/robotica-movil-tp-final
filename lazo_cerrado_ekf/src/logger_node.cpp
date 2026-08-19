#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

std::string formatTime(const boost::posix_time::ptime& time, const char* format)
{
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format(format);

  std::stringstream stream;
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << time;

  return stream.str();
}

std::string timestamp()
{
  return formatTime(boost::posix_time::second_clock::local_time(), "%Y-%m-%d_%H:%M:%S");
}

class Logger : public rclcpp::Node
{
public:
  Logger();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::ofstream vel_real_logfile_;
  std::ofstream vel_ref_logfile_;

  void handleOdom(const nav_msgs::msg::Odometry& msg);
  void handleCmdVel(const geometry_msgs::msg::Twist& msg);
};

Logger::Logger() :
  Node("velocity_logger"),
  vel_real_logfile_(timestamp() + "_vel_real.log"),
  vel_ref_logfile_(timestamp() + "_vel_ref.log")
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/robot/odometry",
    rclcpp::QoS(10),
    std::bind(&Logger::handleOdom, this, std::placeholders::_1)
  );

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10),
    std::bind(&Logger::handleCmdVel, this, std::placeholders::_1)
  );
}

void Logger::handleOdom(const nav_msgs::msg::Odometry& msg)
{
  double t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  vel_real_logfile_
    << t << " "
    << msg.twist.twist.linear.x << " "
    << msg.twist.twist.linear.y << " "
    << msg.twist.twist.angular.z
    << std::endl;
}

void Logger::handleCmdVel(const geometry_msgs::msg::Twist& msg)
{
  double t = this->now().seconds();

  vel_ref_logfile_
    << t << " "
    << msg.linear.x << " "
    << msg.linear.y << " "
    << msg.angular.z
    << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto logger = std::make_shared<Logger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
}

