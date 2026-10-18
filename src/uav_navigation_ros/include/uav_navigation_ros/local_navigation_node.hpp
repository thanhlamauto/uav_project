#pragma once
#include "uav_navigation_ros/navigation_helpers.hpp"
namespace uav_navigation_ros {
class LocalNavigationNode final : public rclcpp::Node {
public:
  LocalNavigationNode();


private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  // Diagnostic bookkeeping for one control cycle. None of these
  // values feed back into control or safety decisions.
  struct CycleDiagnostics {
    std::int64_t command_stamp_ns{0};
    double scheduling_delay_ms{0.0};
    double wall_cycle_ms{0.0};
    double input_snapshot_ms{0.0};
    double path_reference_ms{0.0};
    double sampling_ms{0.0};
    double optimizer_update_ms{0.0};
    double publish_ms{0.0};
    double speed_total_m_s{0.0};
    double speed_xy_m_s{0.0};
    double speed_z_m_s{0.0};
    double yaw_rad{0.0};
    double yaw_rate_rad_s{0.0};
    double path_tangent_heading_rad{0.0};
    double heading_error_rad{0.0};
    double path_curvature_per_m{0.0};
    double distance_to_next_turn_m{
        std::numeric_limits<double>::infinity()};
    double stopping_distance_m{0.0};
    double reference_speed_cap_m_s{std::numeric_limits<double>::infinity()};
    double path_progress_m{0.0};
    double minimum_collision_clearance_m{
        std::numeric_limits<double>::infinity()};
    double minimum_stopping_clearance_m{
        std::numeric_limits<double>::infinity()};
    std::array<double, 4> warm_start_u{};
    std::array<double, 4> selected_u{};
    int selected_source{static_cast<int>(mppi::ProposalSource::kGaussian)};
    std::uint64_t local_active_path_id{0};
    std::uint64_t previous_path_id{0};
    std::int64_t active_path_stamp_ns{0};
    std::uint64_t solves_on_active_path{0};
    bool first_solve_on_active_path{false};
    bool first_safe_command_on_active_path{false};
  };

  void OnOdometry(const nav_msgs::msg::Odometry &message);


  void OnPath(const nav_msgs::msg::Path &message);


  void OnObstacles(const sensor_msgs::msg::PointCloud2 &message);


  void ResetController();


  // A deadline hold still reports the partial compute time that missed the
  // deadline, so timing attribution can separate scheduler latency from MPPI
  // computation overrun. Other holds report zeros.
  void Hold(navigation_detail::Mode mode, const std::string &reason, double total_ms = 0.0,
            double rollout_ms = 0.0, double cost_ms = 0.0,
            double safety_ms = 0.0);


  void ControlCycle();


  static double WrapAngle(double angle);


  double PathTangentHeading(const mppi::PathReference &path,
                            double progress_m) const;


  double DistanceToNextTurn(const mppi::PathReference &path, double progress_m,
                            double turn_angle = navigation_detail::kTurnAngle) const;


  void FillMotionDiagnostics(const core::Control &warm_start);


  void FillSelectedControl(const core::Control &selected);


  static std::array<double, 4> ComponentsOf(const core::Control &control);


  // Reference conservative reference-speed shaping delegated to the
  // core stabilization module (unit-tested there). Inert when disabled.
  double ComputeReferenceSpeedCap() const;


  void ExitStoppingRecovery();


  // Recovery: when all samples are rejected and the stopping-distance
  // predicate dominates, command a bounded braking action only if the existing
  // safety checker certifies its rollout. Otherwise fail closed as before.
  bool TryStoppingRecovery(const mppi::MppiOptimizationResult &result,
                           SteadyTime cycle_started);


  // The second rollout replaces ONLY the first action with the published command;
  // future actions remain the selected nominal plan, not future executed commands.


  void PublishDiagnostic(double total_ms, double rollout_ms, double cost_ms,
                         double safety_ms, std::size_t samples,
                         std::size_t safe_samples, double ess, double clearance,
                         double conditioner_ms = 0.0,
                         double best_feasible_cost =
                             std::numeric_limits<double>::infinity());


  void PublishVisualization();


  std::string planning_frame_;
  std::string body_frame_;
  double control_rate_hz_{10.0};
  bool route_block_hold_enabled_{false};
  bool route_blocked_latched_{false};
  bool blocked_route_detour_enabled_{false};
  int detour_side_{0};
  double blocked_route_path_scale_{0.0};
  double route_block_lookahead_m_{3.0};
  double route_block_radius_m_{1.5};
  double dt_s_{0.1};
  double state_timeout_s_{1.0};
  double obstacle_timeout_s_{1.0};
  double max_obstacle_source_age_s_{0.0};
  double max_compute_time_ms_{80.0};
  double goal_tolerance_m_{0.5};
  double goal_speed_tolerance_m_s_{0.5};
  double visualization_rate_hz_{5.0};
  std::size_t horizon_{30};
  std::size_t samples_{80};
  std::size_t obstacle_max_points_{800};
  std::size_t min_obstacle_points_{0};
  std::size_t visualization_max_samples_{80};
  bool proactive_proposals_{true};
  bool visualization_enabled_{true};
  bool have_state_{false};
  bool have_obstacles_{false};
  navigation_detail::Mode mode_{navigation_detail::Mode::kWaitingForState};
  std::string reason_{"startup"};
  std::string odometry_error_;
  std::string path_error_;
  std::string obstacle_error_;
  SteadyTime last_state_received_{};
  SteadyTime last_obstacles_received_{};
  core::Vec3 goal_{};
  mppi::MppiState state_{};
  core::ObstacleMap obstacles_{};
  mppi::ControlSequence nominal_{};
  std::shared_ptr<const mppi::PathReference> reference_path_{};
  std::unique_ptr<mppi::MultirotorMotionModel> motion_model_;
  std::unique_ptr<mppi::CostEvaluator> cost_evaluator_;
  std::unique_ptr<mppi::MppiOptimizer> optimizer_;
  std::unique_ptr<mppi::GaussianNoiseSampler> noise_sampler_;
  std::unique_ptr<core::TrajectorySafetyChecker> safety_checker_;
  std::unique_ptr<core::VelocityCommandConditioner> conditioner_;
  std::optional<mppi::MppiOptimizationResult> last_result_;
  std::optional<mppi::MppiTrajectory> selected_trajectory_;
  CycleDiagnostics cycle_diag_{};
  mppi::RejectionCounts last_rejection_{};
  std::array<mppi::ProposalSourceStats,
             static_cast<std::size_t>(mppi::ProposalSource::kCount)>
      last_proposal_stats_{};
  double stopping_delay_s_{0.25};
  double braking_accel_m_s2_{3.0};
  bool speed_shaping_enabled_{false};
  double speed_shaping_nominal_speed_m_s_{10.0};
  double speed_shaping_turn_speed_m_s_{3.0};
  double speed_shaping_min_speed_m_s_{0.5};
  double speed_shaping_lateral_accel_m_s2_{2.0};
  double speed_shaping_braking_accel_m_s2_{3.0};
  double speed_shaping_reaction_delay_s_{0.25};
  double speed_shaping_turn_angle_rad_{navigation_detail::kTurnAngle};
  double speed_shaping_lookahead_m_{2.0};
  bool stopping_recovery_enabled_{false};
  double stopping_recovery_deceleration_m_s2_{3.0};
  core::SpeedShapingConfig speed_shaping_config_{};
  core::StoppingRecoveryTracker stopping_recovery_tracker_{};
  SteadyTime previous_cycle_start_{};
  bool have_previous_cycle_{false};
  std::uint64_t cycle_counter_{0};
  std::uint64_t local_active_path_id_{0};
  std::uint64_t previous_path_id_{0};
  std::int64_t active_path_stamp_ns_{0};
  SteadyTime path_received_monotonic_{};
  std::uint64_t solves_on_active_path_{0};
  bool have_first_solve_on_path_{false};
  bool have_first_safe_on_path_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      raw_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      sample_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};
}
