#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/* =========================
 * Helper: yaw from quaternion
 * ========================= */
double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  return yaw;
}

/* =========================
 * Logger node
 * ========================= */
class Logger : public rclcpp::Node
{
public:
  Logger();

private:
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;

  // Log files
  std::ofstream vel_ref_log_;
  std::ofstream vel_real_log_;
  std::ofstream pose_odom_log_;
  std::ofstream pose_gt_log_;

  // Callbacks
  void onCmdVel(const geometry_msgs::msg::Twist &msg);
  void onOdom(const nav_msgs::msg::Odometry &msg);
  void onGT(const nav_msgs::msg::Odometry &msg);
};

/* =========================
 * Constructor
 * ========================= */
Logger::Logger() : Node("logger")
{
  vel_ref_log_.open("vel_ref.log");
  vel_real_log_.open("vel_real.log");
  pose_odom_log_.open("pose_odom.log");
  pose_gt_log_.open("pose_gt.log");

  if (!vel_ref_log_ || !vel_real_log_ || !pose_odom_log_ || !pose_gt_log_) {
    RCLCPP_ERROR(get_logger(), "No se pudieron abrir los archivos de log");
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10),
    std::bind(&Logger::onCmdVel, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/robot/odometry",
    rclcpp::QoS(10),
    std::bind(&Logger::onOdom, this, std::placeholders::_1));

  gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/robot/ground_truth",
    rclcpp::QoS(10),
    std::bind(&Logger::onGT, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Logger iniciado");
}

/* =========================
 * Callbacks
 * ========================= */

// Velocidades de referencia (/cmd_vel)
void Logger::onCmdVel(const geometry_msgs::msg::Twist &msg)
{
  double t = now().seconds();

  vel_ref_log_
    << t << " "
    << msg.linear.x << " "
    << msg.linear.y << " "
    << msg.angular.z << std::endl;
}

// Odometría: pose + velocidades reales
void Logger::onOdom(const nav_msgs::msg::Odometry &msg)
{
  double t = msg.header.stamp.sec +
             msg.header.stamp.nanosec * 1e-9;

  double yaw = yawFromQuat(msg.pose.pose.orientation);

  // Pose odométrica
  pose_odom_log_
    << t << " "
    << msg.pose.pose.position.x << " "
    << msg.pose.pose.position.y << " "
    << yaw << std::endl;

  // Velocidades reales (estimadas por odometría)
  vel_real_log_
    << t << " "
    << msg.twist.twist.linear.x << " "
    << msg.twist.twist.linear.y << " "
    << msg.twist.twist.angular.z << std::endl;
}

// Ground truth del simulador
void Logger::onGT(const nav_msgs::msg::Odometry &msg)
{
  double t = msg.header.stamp.sec +
             msg.header.stamp.nanosec * 1e-9;

  double yaw = yawFromQuat(msg.pose.pose.orientation);

  pose_gt_log_
    << t << " "
    << msg.pose.pose.position.x << " "
    << msg.pose.pose.position.y << " "
    << yaw << std::endl;
}

/* =========================
 * Main
 * ========================= */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Logger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

