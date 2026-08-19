#include <angles/angles.h>
#include "KinematicPositionController.h"

KinematicPositionController::KinematicPositionController() :
  TrajectoryFollower(), tfBuffer_(this->get_clock()),transform_listener_( tfBuffer_ )
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
    qos_profile.reliable();
    qos_profile.durability_volatile();

    expected_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));

         
    std::string goal_selection = this->declare_parameter("goal_selection", "TIME_BASED");
    fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 3.0);
    fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 0.0);
    fixed_goal_a_ = this->declare_parameter("fixed_goal_a", -M_PI_2);
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const rclcpp::Time& t0, const rclcpp::Time& t1, double y0, double y1, const rclcpp::Time& t)
{
  return y0 + (t - t0).seconds() * (y1 - y0) / (t1 - t0).seconds();
}

bool KinematicPositionController::updateCurrentPoseFromTF()
{
  try
  {
    geometry_msgs::msg::TransformStamped tf;

    tf = tfBuffer_.lookupTransform(
      "map",
      "base_link_ekf",
      tf2::TimePointZero);

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    a = yaw;

    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
    return false;
  }
}


#define K_X 1.5
#define K_Y 1.5
#define K_A 3.5

bool KinematicPositionController::control(const rclcpp::Time& t, double& vx, double& vy, double& w)
{
  // Se obtiene la pose actual de la transformada de EKF
  if (!updateCurrentPoseFromTF())
    return false;

  double current_x = x;
  double current_y = y;
  double current_a = a;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  /** EJERCICIO 1: COMPLETAR: Aqui deberan realizar las cuentas necesarias para determinar:
   *             - la velocidad lineal: asignando la variable v
   *             - la velocidad angular: asignando la variable w 
   *  
   *  RECORDAR: cambiar el marco de referencia en que se encuentran dx, dy y theta */

  // Reemplazamos el control de Stewart con un controlador proporcional basico (Kp * error) para cada grado de libertad

  double dx = goal_x - current_x; // error en x, y, theta
  double dy = goal_y - current_y;
  
  
  double goal_a_override = atan2(current_y, current_x); //calculo segun donde estoy la orientacion opuesta al centro
  double da = angles::normalize_angle(goal_a_override - current_a);

  double x_p = dx * K_X;
  double y_p = dy * K_Y;
  double a_p = da * K_A;

  double dx_rot =  cos(current_a) * x_p + sin(current_a) * y_p; 
  double dy_rot = -sin(current_a) * x_p + cos(current_a) * y_p;

  vx = dx_rot;
  vy = dy_rot;
  w = a_p;

  RCLCPP_INFO(this->get_logger(),
  "ex: %.2f ey: %.2f eθ: %.2f | vx: %.2f vy: %.2f w: %.2f", dx, dy, da, vx, vy, w);

  RCLCPP_INFO(this->get_logger(), "goal_x: %.2f, goal_y: %.2f, goal_a: %.2f, current_x: %.2f, current_y: %.2f, current_a: %.2f",
            goal_x, goal_y, goal_a_override, current_x, current_y, current_a);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

bool KinematicPositionController::getPursuitBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::msg::Trajectory& trajectory = getTrajectory();
  
  /** EJERCICIO 3:
   * Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
   * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */
  double distancia_minima = std::numeric_limits<double>::max();
  size_t indice_cercano = 0;
  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */  
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);
    
    
    if (dist2(current_x, current_y, wpoint_x, wpoint_y) < distancia_minima) {
      distancia_minima = dist2(current_x, current_y, wpoint_x, wpoint_y);
      indice_cercano = i;
    }
  }
  const robmovil_msgs::msg::TrajectoryPoint& last_wpoint = trajectory.points.back(); 
  

  double lookahead_distance = 0.1; // [m] distancia de anticipacion
  for(unsigned int j = indice_cercano; j < trajectory.points.size(); j++)
  {
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[j];
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double dx = wpoint_x - current_x;
    double dy = wpoint_y - current_y;
    double dist = sqrt(dx*dx + dy*dy);

    if (dist > lookahead_distance) {
      x = wpoint_x;
      y = wpoint_y;
      a = wpoint_a;
      return true;
    }
  }

  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return false;
}

bool KinematicPositionController::getTimeBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const robmovil_msgs::msg::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::msg::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const rclcpp::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}
