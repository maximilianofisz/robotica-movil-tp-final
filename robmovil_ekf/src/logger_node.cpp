#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

// --------------------------------------------------
// Time utilities
// --------------------------------------------------

std::string formatTime(const boost::posix_time::ptime& time, const char* format)
{
    auto* facet = new boost::posix_time::time_facet();
    facet->format(format);

    std::stringstream stream;
    stream.imbue(std::locale(std::locale::classic(), facet));
    stream << time;

    return stream.str();
}

std::string timestamp()
{
    return formatTime(
        boost::posix_time::second_clock::local_time(),
        "%Y-%m-%d_%H:%M:%S"
    );
}

// --------------------------------------------------
// Logger node
// --------------------------------------------------

class Logger : public rclcpp::Node
{
public:
    Logger();

private:
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_poses_sub_;

    // Log files
    std::ofstream robot_odom_logfile_;
    std::ofstream robot_ekf_logfile_;
    std::ofstream ground_truth_logfile_;
    std::ofstream goal_poses_logfile_;

    // Callbacks
    void handleRobotPose(const nav_msgs::msg::Odometry& msg);
    void handleEkfPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
    void handleGroundTruthPose(const nav_msgs::msg::Odometry& msg);
    void handleGoalPose(const geometry_msgs::msg::PoseStamped& msg);
};

// --------------------------------------------------
// Logger implementation
// --------------------------------------------------

Logger::Logger()
    : Node("nodeLogger"),
      robot_odom_logfile_(timestamp() + "_odom_poses.log"),
      robot_ekf_logfile_(timestamp() + "_ekf_poses.log"),
      ground_truth_logfile_(timestamp() + "_ground-truth.log"),
      goal_poses_logfile_(timestamp() + "_goals.log")
{
    ekf_pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/pose",
        rclcpp::QoS(10),
        std::bind(&Logger::handleEkfPose, this, std::placeholders::_1)
    );

    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/robot/odometry",
    rclcpp::QoS(10),
    std::bind(&Logger::handleRobotPose, this, _1)
);

    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot/ground_truth",
        rclcpp::QoS(10),
        std::bind(&Logger::handleGroundTruthPose, this, std::placeholders::_1)
    );

    goal_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        rclcpp::QoS(10),
        std::bind(&Logger::handleGoalPose, this, std::placeholders::_1)
    );
}

void Logger::handleEkfPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
    double t = msg.header.stamp.sec +
               msg.header.stamp.nanosec * 1e-9;

    robot_ekf_logfile_
        << t << " "
        << msg.pose.pose.position.x << " "
        << msg.pose.pose.position.y << " "
        << tf2::getYaw(msg.pose.pose.orientation)
        << std::endl;
}

void Logger::handleRobotPose(const nav_msgs::msg::Odometry& msg) { double t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9; robot_odom_logfile_ << t << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw(msg.pose.pose.orientation) << std::endl; }

void Logger::handleGroundTruthPose(const nav_msgs::msg::Odometry& msg)
{
    double t = msg.header.stamp.sec +
               msg.header.stamp.nanosec * 1e-9;

    ground_truth_logfile_
        << t << " "
        << msg.pose.pose.position.x << " "
        << msg.pose.pose.position.y << " "
        << tf2::getYaw(msg.pose.pose.orientation)
        << std::endl;
}

void Logger::handleGoalPose(const geometry_msgs::msg::PoseStamped& msg)
{
    double t = msg.header.stamp.sec +
               msg.header.stamp.nanosec * 1e-9;

    goal_poses_logfile_
        << t << " "
        << msg.pose.position.x << " "
        << msg.pose.position.y << " "
        << tf2::getYaw(msg.pose.orientation)
        << std::endl;
}

// --------------------------------------------------
// Main
// --------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto logger = std::make_shared<Logger>();
    rclcpp::spin(logger);

    rclcpp::shutdown();
    return 0;
}
