#include "uav_navigation_ros/local_navigation_node.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;
std::shared_ptr<const core::IFlightRegion> LoadPriorFlightRegion(rclcpp::Node&);

LocalNavigationNode::LocalNavigationNode()
      : Node("local_navigation"), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
    obstacle_timeout_s_ = declare_parameter<double>("obstacle_timeout_s", 1.0);
    max_obstacle_source_age_s_ =
        declare_parameter<double>("max_obstacle_source_age_s", 0.0);
    max_compute_time_ms_ =
        declare_parameter<double>("max_compute_time_ms", 80.0);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.5);
    goal_speed_tolerance_m_s_ =
        declare_parameter<double>("goal_speed_tolerance_m_s", 0.5);
    const auto obstacle_max_points =
        declare_parameter<int64_t>("obstacle_max_points", 800);
    const auto min_obstacle_points =
        declare_parameter<int64_t>("min_obstacle_points", 0);
    visualization_rate_hz_ =
        declare_parameter<double>("visualization.publish_rate_hz", 5.0);
    visualization_enabled_ =
        declare_parameter<bool>("visualization.enabled", true);
    const auto visualization_max_samples =
        declare_parameter<int64_t>("visualization.max_mppi_samples", 80);

    const double dt = declare_parameter<double>("mppi.dt_s", 0.1);
    const auto horizon = declare_parameter<int64_t>("mppi.horizon_steps", 30);
    const auto samples = declare_parameter<int64_t>("mppi.samples", 80);
    const double lambda = declare_parameter<double>("mppi.lambda", 1.0);
    const auto seed = declare_parameter<int64_t>("mppi.seed", 7);
    const double vmax =
        declare_parameter<double>("mppi.max_speed_xy_m_s", 10.0);
    const double vzmax = declare_parameter<double>("mppi.max_speed_z_m_s", 0.6);
    const double yaw_rate_max =
        declare_parameter<double>("mppi.max_yaw_rate_rad_s", 0.6);
    const double accel_xy =
        declare_parameter<double>("mppi.max_accel_xy_m_s2", 3.0);
    const double accel_z =
        declare_parameter<double>("mppi.max_accel_z_m_s2", 0.4);
    const double yaw_accel =
        declare_parameter<double>("mppi.max_yaw_accel_rad_s2", 0.6);
    const double command_alpha =
        declare_parameter<double>("conditioner.alpha", 0.30);
    const double noise_xy = declare_parameter<double>("mppi.noise_xy", 0.8);
    const double noise_z = declare_parameter<double>("mppi.noise_z", 0.3);
    const double noise_yaw = declare_parameter<double>("mppi.noise_yaw", 0.3);
    proactive_proposals_ =
        declare_parameter<bool>("mppi.proactive_proposals", true);

    if (planning_frame_.empty() || body_frame_.empty() ||
        control_rate_hz_ <= 0 || state_timeout_s_ <= 0 ||
        obstacle_timeout_s_ <= 0 || max_obstacle_source_age_s_ < 0 ||
        max_compute_time_ms_ <= 0 || horizon <= 0 ||
        samples <= 0 || obstacle_max_points <= 0 || min_obstacle_points < 0 ||
        min_obstacle_points > obstacle_max_points ||
        visualization_rate_hz_ <= 0 || visualization_max_samples < 0)
      throw std::invalid_argument(
          "invalid local-navigation timing or size parameter");
    dt_s_ = dt;
    horizon_ = static_cast<std::size_t>(horizon);
    samples_ = static_cast<std::size_t>(samples);
    obstacle_max_points_ = static_cast<std::size_t>(obstacle_max_points);
    min_obstacle_points_ = static_cast<std::size_t>(min_obstacle_points);
    visualization_max_samples_ =
        static_cast<std::size_t>(visualization_max_samples);

    mppi::MppiDynamicsConfig dynamics;
    dynamics.tau_s = declare_parameter<double>("response.tau_s", 0.5);
    dynamics.yaw_response_model = declare_parameter<bool>("response.yaw_enabled", false);
    dynamics.yaw_tau_s = declare_parameter<double>("response.yaw_tau_s", 0.5);
    dynamics.command_alpha = command_alpha;
    dynamics.max_accel_xy_m_s2 = accel_xy;
    dynamics.max_accel_z_m_s2 = accel_z;
    dynamics.max_yaw_accel_rad_s2 = yaw_accel;
    dynamics.vmax_m_s = vmax;
    dynamics.vzmax_m_s = vzmax;
    dynamics.yaw_rate_max_rad_s = yaw_rate_max;
    dynamics.response_accel_model =
        declare_parameter<bool>("response.enabled", true);
    dynamics.response_accel_xy_m_s2 =
        declare_parameter<double>("response.accel_xy_m_s2", accel_xy);
    dynamics.response_jerk_xy_m_s3 =
        declare_parameter<double>("response.jerk_xy_m_s3", 4.0);
    motion_model_ = std::make_unique<mppi::MultirotorMotionModel>(dynamics);

    mppi::MppiCostConfig cost;
    const auto profile =
        declare_parameter<std::string>("mppi.cost_profile", "project");
    if (profile == "paper")
      cost.profile = mppi::CostProfile::kPaper;
    else if (profile != "project")
      throw std::invalid_argument("mppi.cost_profile must be project or paper");
    cost.path_progress_objective =
        declare_parameter<bool>("mppi.path_progress_objective", true);
    cost.w_goal = declare_parameter<double>("mppi.goal_weight", 0.0);
    cost.w_terminal = declare_parameter<double>("mppi.terminal_weight", 0.0);
    cost.w_obstacle = declare_parameter<double>("mppi.obstacle_weight", 300.0);
    cost.margin_m = declare_parameter<double>("mppi.obstacle_margin_m", 4.0);
    cost.w_collision =
        declare_parameter<double>("mppi.collision_weight", 1.0e6);
    cost.collision_radius_m =
        declare_parameter<double>("safety.collision_radius_m", 1.5);
    cost.collision_cost_buffer_m =
        declare_parameter<double>("mppi.collision_cost_buffer_m", 0.0);
    cost.w_effort = declare_parameter<double>("mppi.effort_weight", 0.05);
    cost.w_smoothness =
        declare_parameter<double>("mppi.smoothness_weight", 0.2);
    cost.w_yaw = declare_parameter<double>("mppi.yaw_weight", 0.2);
    cost.w_path = declare_parameter<double>("mppi.path_weight", 400.0);
    cost.path_scale_m = declare_parameter<double>("mppi.path_scale_m", 1.0);
    cost.w_progress = declare_parameter<double>("mppi.progress_weight", 5000.0);
    cost.w_speed_limit =
        declare_parameter<double>("mppi.speed_limit_weight", 10000.0);
    cost.vmax_m_s = vmax;
    cost.w_stopping = declare_parameter<double>("mppi.stopping_weight", 0.0);
    cost.stopping_margin_m =
        declare_parameter<double>("mppi.stopping_margin_m", 1.5);
    cost.stopping_delay_s =
        declare_parameter<double>("safety.stopping_delay_s", 0.25);
    // A 3D braking tube must respect the weakest controlled axis.
    cost.braking_acceleration_m_s2 = declare_parameter<double>(
        "safety.braking_acceleration_m_s2", std::min(accel_xy,accel_z));
    if (!Finite(cost.braking_acceleration_m_s2) || cost.braking_acceleration_m_s2<=0 ||
        cost.braking_acceleration_m_s2>std::min(accel_xy,accel_z))
      throw std::invalid_argument("3D braking acceleration exceeds weakest axis limit");
    cost_evaluator_ = std::make_unique<mppi::CostEvaluator>(cost);

    mppi::MppiOptimizerConfig optimizer;
    optimizer.lambda = lambda;
    optimizer.noise_sigma = {{noise_xy, noise_xy, noise_z, noise_yaw}};
    optimizer.minimum = {{-vmax, -vmax, -vzmax, -yaw_rate_max}};
    optimizer.maximum = {{vmax, vmax, vzmax, yaw_rate_max}};
    optimizer_ = std::make_unique<mppi::MppiOptimizer>(optimizer);
    noise_sampler_ = std::make_unique<mppi::GaussianNoiseSampler>(
        optimizer.noise_sigma, seed);

    core::TrajectorySafetyConfig safety;
    safety.collision_radius_m = cost.collision_radius_m;
    safety.braking_acceleration_m_s2 = cost.braking_acceleration_m_s2;
    safety.stopping_delay_s = cost.stopping_delay_s;
    safety.stopping_clearance_m =
        declare_parameter<double>("safety.stopping_clearance_m", 1.5);
    safety.stopping_uncertainty_m =
        declare_parameter<double>("safety.stopping_uncertainty_m", 0.0);
    safety_checker_ = std::make_unique<core::TrajectorySafetyChecker>(
        safety, nullptr, LoadPriorFlightRegion(*this));
    route_block_hold_enabled_ =
        declare_parameter<bool>("safety.route_block_hold_enabled", false);
    route_block_lookahead_m_ =
        declare_parameter<double>("safety.route_block_lookahead_m", 3.0);
    if (!Finite(route_block_lookahead_m_) || route_block_lookahead_m_ <= 0.0)
      throw std::invalid_argument("invalid route blockage lookahead");
    route_block_radius_m_ = safety.collision_radius_m;
    blocked_route_detour_enabled_ =
        declare_parameter<bool>("mppi.blocked_route_detour_enabled", false);
    blocked_route_path_scale_ =
        declare_parameter<double>("mppi.blocked_route_path_scale", 0.0);
    if (!Finite(blocked_route_path_scale_) ||
        blocked_route_path_scale_ < 0.0 || blocked_route_path_scale_ > 1.0)
      throw std::invalid_argument("invalid blocked route path scale");
    stopping_delay_s_ = safety.stopping_delay_s;
    braking_accel_m_s2_ = cost.braking_acceleration_m_s2;

    speed_shaping_enabled_ =
        declare_parameter<bool>("stabilization.speed_shaping.enabled", false);
    speed_shaping_nominal_speed_m_s_ = declare_parameter<double>(
        "stabilization.speed_shaping.nominal_speed_m_s", vmax);
    speed_shaping_turn_speed_m_s_ = declare_parameter<double>(
        "stabilization.speed_shaping.turn_speed_m_s", 3.0);
    speed_shaping_min_speed_m_s_ = declare_parameter<double>(
        "stabilization.speed_shaping.min_speed_m_s", 0.5);
    speed_shaping_lateral_accel_m_s2_ = declare_parameter<double>(
        "stabilization.speed_shaping.lateral_accel_m_s2", 2.0);
    speed_shaping_braking_accel_m_s2_ = declare_parameter<double>(
        "stabilization.speed_shaping.braking_accel_m_s2", accel_xy);
    speed_shaping_reaction_delay_s_ = declare_parameter<double>(
        "stabilization.speed_shaping.reaction_delay_s", safety.stopping_delay_s);
    speed_shaping_turn_angle_rad_ = declare_parameter<double>(
        "stabilization.speed_shaping.turn_angle_rad", kTurnAngle);
    speed_shaping_lookahead_m_ = declare_parameter<double>(
        "stabilization.speed_shaping.lookahead_m", 2.0);
    stopping_recovery_enabled_ =
        declare_parameter<bool>("stabilization.stopping_recovery.enabled",
                                false);
    stopping_recovery_deceleration_m_s2_ = declare_parameter<double>(
        "stabilization.stopping_recovery.deceleration_m_s2", accel_xy);
    if (speed_shaping_nominal_speed_m_s_ <= 0.0 ||
        speed_shaping_turn_speed_m_s_ < 0.0 ||
        speed_shaping_min_speed_m_s_ < 0.0 ||
        speed_shaping_min_speed_m_s_ > speed_shaping_turn_speed_m_s_ ||
        speed_shaping_turn_speed_m_s_ > speed_shaping_nominal_speed_m_s_ ||
        speed_shaping_lateral_accel_m_s2_ <= 0.0 ||
        speed_shaping_braking_accel_m_s2_ <= 0.0 ||
        speed_shaping_reaction_delay_s_ < 0.0 ||
        speed_shaping_turn_angle_rad_ <= 0.0 ||
        speed_shaping_turn_angle_rad_ >= kPi ||
        speed_shaping_lookahead_m_ <= 0.0 ||
        stopping_recovery_deceleration_m_s2_ <= 0.0)
      throw std::invalid_argument("invalid stabilization parameter");
    speed_shaping_config_.enabled = speed_shaping_enabled_;
    speed_shaping_config_.nominal_speed_m_s = speed_shaping_nominal_speed_m_s_;
    speed_shaping_config_.turn_speed_m_s = speed_shaping_turn_speed_m_s_;
    speed_shaping_config_.min_speed_m_s = speed_shaping_min_speed_m_s_;
    speed_shaping_config_.lateral_accel_m_s2 = speed_shaping_lateral_accel_m_s2_;
    speed_shaping_config_.braking_accel_m_s2 = speed_shaping_braking_accel_m_s2_;
    speed_shaping_config_.reaction_delay_s = speed_shaping_reaction_delay_s_;
    speed_shaping_config_.turn_angle_rad = speed_shaping_turn_angle_rad_;
    speed_shaping_config_.lookahead_m = speed_shaping_lookahead_m_;
    stopping_recovery_tracker_.Configure(stopping_recovery_enabled_);

    core::VelocityCommandConditionerConfig conditioner;
    conditioner.dt_s = dt;
    conditioner.alpha = command_alpha;
    conditioner.max_speed_xy_m_s = vmax;
    conditioner.max_accel_xy_m_s2 = accel_xy;
    conditioner.max_accel_z_m_s2 = accel_z;
    conditioner.max_yaw_accel_rad_s2 = yaw_accel;
    conditioner.minimum = optimizer.minimum;
    conditioner.maximum = optimizer.maximum;
    conditioner_ =
        std::make_unique<core::VelocityCommandConditioner>(conditioner);
    nominal_.resize(horizon_);

    const auto odometry_topic = declare_parameter<std::string>(
        "odometry_topic", "/localization/odometry");
    const auto path_topic = declare_parameter<std::string>(
        "global_path_topic", "/planning/global_path");
    const auto obstacles_topic = declare_parameter<std::string>(
        "obstacles_topic", "/perception/obstacles");
    const auto output_topic = declare_parameter<std::string>(
        "output_command_topic", "/control/safe_velocity_command");
    const auto raw_output_topic = declare_parameter<std::string>(
        "raw_command_topic", "/control/raw_velocity_command");
    const auto predicted_topic = declare_parameter<std::string>(
        "predicted_path_topic", "/planning/mppi/predicted_path");
    const auto samples_topic = declare_parameter<std::string>(
        "cost_samples_topic", "/planning/mppi/cost_samples");
    const auto diagnostics_topic =
        declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    command_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 1);
    raw_command_publisher_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(raw_output_topic, 1);
    predicted_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(predicted_topic, 1);
    sample_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        samples_topic, 1);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic, 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          OnOdometry(*message);
        });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::Path::ConstSharedPtr message) {
          OnPath(*message);
        });
    obstacle_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          OnObstacles(*message);
        });

    control_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_hz_),
                          [this] { ControlCycle(); });
    if (visualization_enabled_) {
      visualization_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / visualization_rate_hz_),
          [this] { PublishVisualization(); });
    }
    RCLCPP_INFO(get_logger(),
                "local navigation ready: %zu samples x %zu steps, %.1f Hz, "
                "deadline %.1f ms",
                samples_, horizon_, control_rate_hz_, max_compute_time_ms_);
  }
}
