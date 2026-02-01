#include "youbot_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.35
#define WHEEL_RADIUS 0.05
#define ENCODER_TICKS 500.0

YoubotOdometry::YoubotOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&YoubotOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_front_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&YoubotOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void YoubotOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  /** Completar los mensajes de velocidad vLeft y vRight*/
  double x = twist->linear.x;
  double y = twist->linear.y;
  double theta = twist-> angular.z;

  // Modelar la cinematica INVERSA para este nuevo robot (Se a donde quiero ir, como debo moverme?)
  // Las ruedas toman velocidad angular, tenemos que traducir los comandos x,y,theta en x rad/s
  // De la eq20 podemos calcular la velocidad de cada rueda
  // lx = lix para cada rueda, ly = liy para cada rueda.
  // lix es la mitad de la distancia entre centros de las ruedas frontales. liy es la mitad de la distancia entre centros de las ruedas front y rear
  double lix = WHEEL_BASELINE / 2;
  double liy = WHEEL_BASELINE / 2;

  double baseline_term = lix + liy;
  double w1 = 1 / WHEEL_RADIUS * (x - y - (baseline_term) * theta);
  double w2 = 1 / WHEEL_RADIUS * (x + y + (baseline_term) * theta);
  double w3 = 1 / WHEEL_RADIUS * (x + y - (baseline_term) * theta);
  double w4 = 1 / WHEEL_RADIUS * (x - y + (baseline_term) * theta);

  // pifiaron un indice entre la figura 2 y las eq pero pareceria que wi y cada rueda se mappea así
  // front_left = 1, front_right = 2, rear_left = 3, rear_right = 4

  // publish front left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = w1;

    vel_pub_front_left_->publish(msg);
  }

  // publish front right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = w2;

    vel_pub_front_right_->publish(msg);
  }

    // publish rear left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = w3;

    vel_pub_rear_left_->publish(msg);
  }

    // publish rear right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = w4;

    vel_pub_rear_right_->publish(msg);
  }
}

void YoubotOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_front_left_ = encoder->ticks[0];
    last_ticks_front_right_ = encoder->ticks[1];
    last_ticks_rear_left_ = encoder->ticks[2];
    last_ticks_rear_right_ = encoder->ticks[3];
    last_ticks_time = encoder->header.stamp;
    return;
  }

  /* Utilizar este delta de tiempo entre momentos */
  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  int32_t delta_ticks_front_left = encoder->ticks[0] - last_ticks_front_left_;
  int32_t delta_ticks_front_right = encoder->ticks[1] - last_ticks_front_right_;
  int32_t delta_ticks_rear_left = encoder->ticks[2] - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right = encoder->ticks[3] - last_ticks_rear_right_;

  double d1 = (M_PI * 2) * delta_ticks_front_left / ENCODER_TICKS; // (rad) osea desplazamientos
  double d2 = (M_PI * 2) * delta_ticks_front_right / ENCODER_TICKS;
  double d3 = (M_PI * 2) * delta_ticks_rear_left / ENCODER_TICKS ;
  double d4 = (M_PI * 2) * delta_ticks_rear_right / ENCODER_TICKS;

  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  double lix = WHEEL_BASELINE / 2;
  double liy = WHEEL_BASELINE / 2;
  double baseline_term = lix + liy;

  // las variables estas tienen la suma de los desplazamientos pero puedo sumar igual la equ 2x
  // integrar ambos lados por tiempo etc etc

  double dx = (d1 + d2 + d3 + d4) * (WHEEL_RADIUS / 4);
  double dy = (-d1 + d2 + d3 - d4) * (WHEEL_RADIUS / 4);
  double dtheta = (-d1 + d2 - d3 + d4) * (WHEEL_RADIUS / (4 * (baseline_term)));

  // rotamos al marco del odom
  double dx_odom = dx * std::cos(theta_) - dy * std::sin(theta_);
  double dy_odom = dx * std::sin(theta_) + dy * std::cos(theta_);

  x_ += dx_odom;
  y_ += dy_odom;
  theta_ += dtheta;

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Posicion estimada respecto al origen de la simulacion
  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocidades
  msg.twist.twist.linear.x = dx_odom / delta_t;
  msg.twist.twist.linear.y = dy_odom / delta_t;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = dtheta / delta_t;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_front_left_ = encoder->ticks[0];
  last_ticks_front_right_ = encoder->ticks[1];
  last_ticks_rear_left_ = encoder->ticks[2];
  last_ticks_rear_right_ = encoder->ticks[3];
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);

}

