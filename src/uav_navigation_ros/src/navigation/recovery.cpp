#include "uav_navigation_ros/local_navigation_node.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;



void LocalNavigationNode::ResetController(){
    std::fill(nominal_.begin(), nominal_.end(), core::Control{});
    conditioner_->Reset();
    last_result_.reset();
    selected_trajectory_.reset();
  }

void LocalNavigationNode::Hold(Mode mode, const std::string &reason, double total_ms,
            double rollout_ms, double cost_ms,
            double safety_ms){
    ExitStoppingRecovery();
    mode_ = mode;
    reason_ = reason;
    ResetController();
    PublishDiagnostic(total_ms, rollout_ms, cost_ms, safety_ms, 0, 0, 0.0,
                      std::numeric_limits<double>::infinity());
  }

double LocalNavigationNode::WrapAngle(double angle){
    while (angle > kPi)
      angle -= 2.0 * kPi;
    while (angle < -kPi)
      angle += 2.0 * kPi;
    return angle;
  }

double LocalNavigationNode::PathTangentHeading(const mppi::PathReference &path,
                            double progress_m) const{
    const double length = path.length_m();
    const double lower = std::clamp(progress_m - 0.5, 0.0, length);
    const double upper = std::clamp(progress_m + 0.5, 0.0, length);
    if (upper - lower < 1e-6)
      return state_.yaw_enu_rad;
    const core::Vec3 a = path.Sample(lower);
    const core::Vec3 b = path.Sample(upper);
    return std::atan2(b.y - a.y, b.x - a.x);
  }

double LocalNavigationNode::DistanceToNextTurn(const mppi::PathReference &path, double progress_m,
                            double turn_angle) const{
    const double length = path.length_m();
    const double step = 0.5;
    const double base = PathTangentHeading(path, progress_m);
    for (double s = progress_m + step; s <= length; s += step)
      if (std::abs(WrapAngle(PathTangentHeading(path, s) - base)) > turn_angle)
        return s - progress_m;
    return std::numeric_limits<double>::infinity();
  }

void LocalNavigationNode::FillMotionDiagnostics(const core::Control &warm_start){
    if (!reference_path_)
      return;
    const double progress = reference_path_->Progress(state_.position_enu_m);
    const double tangent = PathTangentHeading(*reference_path_, progress);
    const double speed_total = Norm(state_.velocity_enu_m_s);
    cycle_diag_.speed_total_m_s = speed_total;
    cycle_diag_.speed_xy_m_s =
        std::hypot(state_.velocity_enu_m_s.x, state_.velocity_enu_m_s.y);
    cycle_diag_.speed_z_m_s = state_.velocity_enu_m_s.z;
    cycle_diag_.yaw_rad = state_.yaw_enu_rad;
    cycle_diag_.yaw_rate_rad_s = state_.applied_control.yaw_rate_enu_rad_s;
    cycle_diag_.path_progress_m = progress;
    cycle_diag_.path_tangent_heading_rad = tangent;
    cycle_diag_.heading_error_rad = WrapAngle(state_.yaw_enu_rad - tangent);
    const double lookahead = std::min(2.0, reference_path_->length_m() - progress);
    if (lookahead > 1e-3) {
      const double ahead =
          PathTangentHeading(*reference_path_, progress + lookahead);
      cycle_diag_.path_curvature_per_m =
          std::abs(WrapAngle(ahead - tangent)) / lookahead;
    }
    cycle_diag_.distance_to_next_turn_m =
        DistanceToNextTurn(*reference_path_, progress);
    cycle_diag_.stopping_distance_m =
        speed_total * stopping_delay_s_ +
        speed_total * speed_total / (2.0 * braking_accel_m_s2_);
    const auto warm = ComponentsOf(warm_start);
    for (std::size_t i = 0; i < 4; ++i)
      cycle_diag_.warm_start_u[i] = warm[i];
    cycle_diag_.local_active_path_id = local_active_path_id_;
    cycle_diag_.active_path_stamp_ns = active_path_stamp_ns_;
    cycle_diag_.previous_path_id = previous_path_id_;
    cycle_diag_.solves_on_active_path = solves_on_active_path_;
  }

void LocalNavigationNode::FillSelectedControl(const core::Control &selected){
    const auto values = ComponentsOf(selected);
    for (std::size_t i = 0; i < 4; ++i)
      cycle_diag_.selected_u[i] = values[i];
  }

std::array<double, 4> LocalNavigationNode::ComponentsOf(const core::Control &control){
    return {control.velocity_enu_m_s.x, control.velocity_enu_m_s.y,
            control.velocity_enu_m_s.z, control.yaw_rate_enu_rad_s};
  }

double LocalNavigationNode::ComputeReferenceSpeedCap() const{
    if (!reference_path_)
      return std::numeric_limits<double>::infinity();
    return core::ComputeReferenceSpeedCap(
        speed_shaping_config_, *reference_path_, state_.position_enu_m,
        state_.velocity_enu_m_s);
  }

void LocalNavigationNode::ExitStoppingRecovery(){
    stopping_recovery_tracker_.RecordOtherCycle();
  }

bool LocalNavigationNode::TryStoppingRecovery(const mppi::MppiOptimizationResult &result,
                           SteadyTime cycle_started){
    if (!stopping_recovery_enabled_ ||
        !core::StoppingDominant(result.rejection_counts))
      return false;
    const auto recovery_started = std::chrono::steady_clock::now();
    const mppi::ControlSequence braking = core::BuildBrakingSequence(
        state_.velocity_enu_m_s, stopping_recovery_deceleration_m_s2_,
        horizon_, dt_s_);
    if (braking.empty())
      return false;
    const mppi::MppiTrajectory trajectory =
        mppi::Rollout(*motion_model_, state_, braking, dt_s_);
    const auto safety = safety_checker_->Evaluate(
        ToCoreState(state_), ToCoreTrajectory(trajectory), obstacles_);
    const double recovery_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - recovery_started)
            .count();
    if (!safety.safe)
      return false;
    const double total_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_started)
            .count();
    if (total_ms > max_compute_time_ms_)
      return false;

    const core::Control requested = braking.front();
    geometry_msgs::msg::TwistStamped raw_command;
    raw_command.header.stamp = now();
    raw_command.header.frame_id = planning_frame_;
    raw_command.twist.linear.x = requested.velocity_enu_m_s.x;
    raw_command.twist.linear.y = requested.velocity_enu_m_s.y;
    raw_command.twist.linear.z = requested.velocity_enu_m_s.z;
    raw_command.twist.angular.z = requested.yaw_rate_enu_rad_s;
    raw_command_publisher_->publish(raw_command);

    const auto conditioner_started = std::chrono::steady_clock::now();
    const auto safe_control =
        conditioner_->Apply(ToCoreState(state_), requested);
    const double conditioner_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - conditioner_started)
            .count();
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    command.twist.linear.x = safe_control.velocity_enu_m_s.x;
    command.twist.linear.y = safe_control.velocity_enu_m_s.y;
    command.twist.linear.z = safe_control.velocity_enu_m_s.z;
    command.twist.angular.z = safe_control.yaw_rate_enu_rad_s;
    cycle_diag_.command_stamp_ns = rclcpp::Time(command.header.stamp).nanoseconds();
    command_publisher_->publish(command);

    nominal_ = braking;
    selected_trajectory_ = trajectory;
    mode_ = Mode::kStoppingRecovery;
    reason_ = "stopping-dominant infeasibility: verified bounded braking";
    stopping_recovery_tracker_.RecordRecoveryCycle();
    PublishDiagnostic(
        total_ms, result.rollout_time_ms, result.cost_time_ms,
        result.safety_time_ms + recovery_ms, result.weights.size(), 0, 0.0,
        std::min(safety.minimum_collision_clearance_m,
                 safety.minimum_stopping_clearance_m),
        conditioner_ms);
    return true;
  }
}
