#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TrajectoryController final : public rclcpp::Node
{
public:
  TrajectoryController()
  : Node("trajectory_controller"), last_tick_(std::chrono::steady_clock::now())
  {
    mode_ = declare_parameter<std::string>("mode", "mission");
    pose_method_ = declare_parameter<std::string>("pose_method", "continuous");
    loop_mission_ = declare_parameter<bool>("loop_mission", true);
    waypoint_radius_ = declare_parameter<double>("waypoint_radius", 0.35);
    tracking_mode_ = declare_parameter<std::string>("tracking_mode", "feedback");
    kff_ = declare_parameter<double>("kff", 1.0);
    a_ = declare_parameter<double>("A", 1.0);
    b_ = declare_parameter<double>("B", 2.0);
    omega_ = declare_parameter<double>("omega", 0.5);
    max_linear_ = declare_parameter<double>("max_linear_speed", 0.5);
    max_angular_ = declare_parameter<double>("max_angular_speed", 1.5);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.10);
    heading_tolerance_ = declare_parameter<double>("heading_tolerance", 0.10);
    const double i_limit = declare_parameter<double>("integral_limit", 0.5);

    const double linear_kp = declare_parameter<double>("linear_kp", 0.8);
    const double linear_ki = declare_parameter<double>("linear_ki", 0.0);
    const double linear_kd = declare_parameter<double>("linear_kd", 0.02);
    const double heading_kp = declare_parameter<double>("heading_kp", 2.4);
    const double heading_ki = declare_parameter<double>("heading_ki", 0.0);
    const double heading_kd = declare_parameter<double>("heading_kd", 0.04);
    const double lateral_kp = declare_parameter<double>("lateral_kp", 1.8);
    const double lateral_ki = declare_parameter<double>("lateral_ki", 0.0);
    const double lateral_kd = declare_parameter<double>("lateral_kd", 0.02);
    linear_pid_.initPid(linear_kp, linear_ki, linear_kd, i_limit, -i_limit, true);
    heading_pid_.initPid(heading_kp, heading_ki, heading_kd, i_limit, -i_limit, true);
    lateral_pid_.initPid(lateral_kp, lateral_ki, lateral_kd, i_limit, -i_limit, true);

    waypoint_x_ = declare_parameter<std::vector<double>>(
      "waypoint_x", std::vector<double>{1.5, 1.5, 0.0});
    waypoint_y_ = declare_parameter<std::vector<double>>(
      "waypoint_y", std::vector<double>{0.0, 1.0, 1.0});
    waypoint_theta_ = declare_parameter<std::vector<double>>(
      "waypoint_theta", std::vector<double>{0.0, 1.5707963, 3.1415926});
    waypoint_dwell_ = declare_parameter<std::vector<double>>(
      "waypoint_dwell", std::vector<double>{});

    if (waypoint_dwell_.empty()) {
      waypoint_dwell_.resize(waypoint_x_.size(), 0.0);
    }

    if (mode_ == "mission" &&
      (waypoint_x_.size() != waypoint_y_.size() ||
      waypoint_x_.size() != waypoint_theta_.size() ||
      waypoint_x_.size() != waypoint_dwell_.size() || waypoint_x_.empty()))
    {
      throw std::runtime_error(
              "waypoint_x, waypoint_y, waypoint_theta e waypoint_dwell devem ter o mesmo tamanho");
    }
    if (std::any_of(waypoint_dwell_.begin(), waypoint_dwell_.end(),
      [](double seconds) {return seconds < 0.0;}))
    {
      throw std::runtime_error("waypoint_dwell deve conter tempos em segundos >= 0");
    }
    if (waypoint_radius_ <= 0.0) {
      throw std::runtime_error("waypoint_radius deve ser maior que zero");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    desired_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/trajectory/desired_pose", 10);
    actual_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("/trajectory/actual_pose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/mission/status", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&TrajectoryController::on_odom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "/goal_pose", 10,
      std::bind(&TrajectoryController::on_goal, this, std::placeholders::_1));
    timer_ = create_wall_timer(20ms, std::bind(&TrajectoryController::step, this));

    if (mode_ != "pose" && mode_ != "mission" && mode_ != "figure8") {
      throw std::runtime_error("mode deve ser pose, mission ou figure8");
    }
    if (tracking_mode_ != "open_loop" && tracking_mode_ != "feedback") {
      throw std::runtime_error("tracking_mode deve ser open_loop ou feedback");
    }
    RCLCPP_INFO(get_logger(), "Controle iniciado: mode=%s pose_method=%s tracking=%s",
      mode_.c_str(), pose_method_.c_str(), tracking_mode_.c_str());
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static double wrap(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double pid(control_toolbox::Pid & controller, double error, double dt_seconds)
  {
    const auto dt_ns = static_cast<uint64_t>(std::max(1.0, dt_seconds * 1.0e9));
    return controller.computeCommand(error, dt_ns);
  }

  void reset_pids()
  {
    linear_pid_.reset();
    heading_pid_.reset();
    lateral_pid_.reset();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;
    if (mode_ == "figure8" && !trajectory_started_) {
      trajectory_started_ = true;
      trajectory_start_ = std::chrono::steady_clock::now();
      rmse_window_start_ = trajectory_start_;
      next_rmse_log_ = trajectory_start_ + std::chrono::seconds(10);
    }
  }

  void on_goal(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    if (!has_goal_ || std::hypot(msg->x - goal_.x, msg->y - goal_.y) > 1e-4 ||
      std::abs(wrap(msg->theta - goal_.theta)) > 1e-4)
    {
      goal_ = *msg;
      has_goal_ = true;
      maneuver_phase_ = 0;
      reset_pids();
    }
  }

  void publish_state()
  {
    if (has_goal_) {
      desired_pub_->publish(goal_);
    }
    geometry_msgs::msg::Pose2D actual;
    actual.x = x_;
    actual.y = y_;
    actual.theta = yaw_;
    actual_pub_->publish(actual);
  }

  void publish_cmd(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(linear, -max_linear_, max_linear_);
    cmd.angular.z = std::clamp(angular, -max_angular_, max_angular_);
    cmd_pub_->publish(cmd);
  }

  bool inside_waypoint_radius() const
  {
    return std::hypot(goal_.x - x_, goal_.y - y_) <= waypoint_radius_;
  }

  void set_mission_goal()
  {
    if (waypoint_index_ >= waypoint_x_.size()) {
      mission_done_ = true;
      return;
    }
    goal_.x = waypoint_x_[waypoint_index_];
    goal_.y = waypoint_y_[waypoint_index_];
    goal_.theta = waypoint_theta_[waypoint_index_];
    has_goal_ = true;
  }

  void step()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    if (dt <= 0.0 || dt > 0.25) {
      dt = 0.02;
    }
    if (!have_odom_) {
      return;
    }

    if (mode_ == "figure8") {
      step_figure8(now, dt);
      return;
    }

    if (mode_ == "mission") {
      if (mission_done_) {
        publish_cmd(0.0, 0.0);
        publish_state();
        return;
      }
      if (!has_goal_) {
        set_mission_goal();
      }
      if (!dwelling_ && inside_waypoint_radius()) {
        dwelling_ = true;
        dwell_start_ = now;
        std_msgs::msg::String status;
        status.data = "Aguardando no waypoint " + std::to_string(waypoint_index_ + 1);
        status_pub_->publish(status);
      }
      if (dwelling_) {
        publish_cmd(0.0, 0.0);
        publish_state();
        const auto dwell = std::chrono::duration<double>(now - dwell_start_).count();
        if (dwell < waypoint_dwell_[waypoint_index_]) {
          return;
        }
        dwelling_ = false;
        ++waypoint_index_;
        maneuver_phase_ = 0;
        reset_pids();
        if (waypoint_index_ >= waypoint_x_.size()) {
          std_msgs::msg::String status;
          if (loop_mission_) {
            waypoint_index_ = 0;
            ++mission_cycles_;
            set_mission_goal();
            status.data = "Ciclo " + std::to_string(mission_cycles_) +
              " concluido; reiniciando no waypoint 1";
            RCLCPP_INFO(get_logger(), "Ciclo %zu concluido; reiniciando missao", mission_cycles_);
          } else {
            mission_done_ = true;
            status.data = "Mission completed";
            RCLCPP_INFO(get_logger(), "Missao concluida: %zu waypoints", waypoint_index_);
            status_pub_->publish(status);
            publish_cmd(0.0, 0.0);
            publish_state();
            return;
          }
          status_pub_->publish(status);
        } else {
          set_mission_goal();
          std_msgs::msg::String status;
          status.data = "Waypoint " + std::to_string(waypoint_index_ + 1) + "/" +
            std::to_string(waypoint_x_.size());
          status_pub_->publish(status);
        }
      }
    }

    if (!has_goal_) {
      publish_cmd(0.0, 0.0);
      return;
    }
    publish_state();
    if (pose_method_ == "three_maneuvers") {
      step_three_maneuvers(dt);
    } else {
      step_continuous(dt);
    }
  }

  void step_continuous(double dt)
  {
    const double dx = goal_.x - x_;
    const double dy = goal_.y - y_;
    const double distance = std::hypot(dx, dy);
    double linear = 0.0;
    double angular = 0.0;
    if (distance > position_tolerance_) {
      const double bearing_error = wrap(std::atan2(dy, dx) - yaw_);
      linear = pid(linear_pid_, distance, dt) * std::max(0.0, std::cos(bearing_error));
      angular = pid(heading_pid_, bearing_error, dt);
    } else {
      const double heading_error = wrap(goal_.theta - yaw_);
      angular = pid(heading_pid_, heading_error, dt);
    }
    publish_cmd(linear, angular);
  }

  void step_three_maneuvers(double dt)
  {
    const double dx = goal_.x - x_;
    const double dy = goal_.y - y_;
    const double distance = std::hypot(dx, dy);
    const double bearing_error = distance > position_tolerance_ ?
      wrap(std::atan2(dy, dx) - yaw_) : 0.0;
    double linear = 0.0;
    double angular = 0.0;

    if (maneuver_phase_ == 0) {
      if (distance <= position_tolerance_ || std::abs(bearing_error) <= heading_tolerance_) {
        maneuver_phase_ = distance <= position_tolerance_ ? 2 : 1;
        heading_pid_.reset();
      } else {
        angular = pid(heading_pid_, bearing_error, dt);
      }
    }
    if (maneuver_phase_ == 1) {
      if (distance <= position_tolerance_) {
        maneuver_phase_ = 2;
        heading_pid_.reset();
        linear_pid_.reset();
      } else {
        linear = pid(linear_pid_, distance, dt);
      }
    }
    if (maneuver_phase_ == 2) {
      const double final_error = wrap(goal_.theta - yaw_);
      if (std::abs(final_error) > heading_tolerance_) {
        angular = pid(heading_pid_, final_error, dt);
      }
    }
    publish_cmd(linear, angular);
  }

  void step_figure8(SteadyTime now, double dt)
  {
    if (!trajectory_started_) {
      return;
    }
    const double t = std::chrono::duration<double>(now - trajectory_start_).count();
    const double wt = omega_ * t;
    const double local_x = a_ * std::sin(wt);
    const double local_y = b_ * std::sin(2.0 * wt);
    const double local_dx = a_ * omega_ * std::cos(wt);
    const double local_dy = 2.0 * b_ * omega_ * std::cos(2.0 * wt);
    const double local_ddx = -a_ * omega_ * omega_ * std::sin(wt);
    const double local_ddy = -4.0 * b_ * omega_ * omega_ * std::sin(2.0 * wt);

    // Keep the figure eight centered at the fixed world origin, independent of startup pose.
    const double dx = local_dx;
    const double dy = local_dy;
    const double ddx = local_ddx;
    const double ddy = local_ddy;

    goal_.x = local_x;
    goal_.y = local_y;
    goal_.theta = std::atan2(local_dy, local_dx);
    has_goal_ = true;
    publish_state();

    const double speed_ff = std::hypot(dx, dy);
    const double denom = std::max(1e-8, dx * dx + dy * dy);
    const double turn_ff = (dx * ddy - dy * ddx) / denom;
    double linear = kff_ * speed_ff;
    double angular = kff_ * turn_ff;

    const double world_ex = goal_.x - x_;
    const double world_ey = goal_.y - y_;
    const double forward_error = std::cos(yaw_) * world_ex + std::sin(yaw_) * world_ey;
    const double lateral_error = -std::sin(yaw_) * world_ex + std::cos(yaw_) * world_ey;
    const double heading_error = wrap(goal_.theta - yaw_);

    if (tracking_mode_ == "feedback") {
      linear += pid(linear_pid_, forward_error, dt);
      angular += pid(lateral_pid_, lateral_error, dt) + pid(heading_pid_, heading_error, dt);
    }
    publish_cmd(linear, angular);

    rmse_position_sq_sum_ += world_ex * world_ex + world_ey * world_ey;
    ++rmse_samples_;
    if (now >= next_rmse_log_ && rmse_samples_ > 0) {
      const double rmse = std::sqrt(rmse_position_sq_sum_ / static_cast<double>(rmse_samples_));
      RCLCPP_INFO(get_logger(), "RMSE xy (%s, Kff=%.2f, Omega=%.3f; janela 10s): %.4f m",
        tracking_mode_.c_str(), kff_, omega_, rmse);
      rmse_position_sq_sum_ = 0.0;
      rmse_samples_ = 0;
      rmse_window_start_ = now;
      next_rmse_log_ = now + std::chrono::seconds(10);
    }
  }

  std::string mode_;
  std::string pose_method_;
  std::string tracking_mode_;
  bool loop_mission_{true};
  double waypoint_radius_{0.35};
  double kff_{1.0}, a_{1.0}, b_{2.0}, omega_{0.5};
  double max_linear_{0.5}, max_angular_{1.5};
  double position_tolerance_{0.10}, heading_tolerance_{0.10};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  size_t waypoint_index_{0};
  size_t mission_cycles_{0};
  int maneuver_phase_{0};
  bool have_odom_{false}, has_goal_{false}, mission_done_{false}, dwelling_{false};
  bool trajectory_started_{false};
  geometry_msgs::msg::Pose2D goal_;
  std::vector<double> waypoint_x_, waypoint_y_, waypoint_theta_, waypoint_dwell_;
  control_toolbox::Pid linear_pid_, heading_pid_, lateral_pid_;
  SteadyTime last_tick_, trajectory_start_, rmse_window_start_, next_rmse_log_, dwell_start_;
  double rmse_position_sq_sum_{0.0};
  size_t rmse_samples_{0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr desired_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr actual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TrajectoryController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
