#include "uav_navigation_ros/local_navigation_node.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;



void LocalNavigationNode::PublishDiagnostic(double total_ms, double rollout_ms, double cost_ms,
                         double safety_ms, std::size_t samples,
                         std::size_t safe_samples, double ess, double clearance,
                         double conditioner_ms,
                         double best_feasible_cost){
    const auto diagnostics_started = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "local_navigation";
    status.hardware_id = "uav_navigation_core";
    status.level = mode_ == Mode::kActive || mode_ == Mode::kGoalReached
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : (mode_ == Mode::kWaitingForState ||
                                  mode_ == Mode::kWaitingForPath ||
                                  mode_ == Mode::kWaitingForObstacles
                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                              : diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    const double diagnostics_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() -
                                      diagnostics_started)
                                      .count();
    const auto source_stats = [&](mppi::ProposalSource source) {
      return last_proposal_stats_[static_cast<std::size_t>(source)];
    };
    const auto gaussian = source_stats(mppi::ProposalSource::kGaussian);
    const auto reference = source_stats(mppi::ProposalSource::kReferenceProposal);
    const auto braking = source_stats(mppi::ProposalSource::kBrakingProposal);
    const auto recovery = source_stats(mppi::ProposalSource::kRecoveryProposal);
    const auto specific = source_stats(mppi::ProposalSource::kSpecificAction);
    status.message = std::string(ModeName(mode_)) + ": " + reason_;
    status.values = {
        KeyValue("mode", ModeName(mode_)),
        // Single-threaded executor: this is the cloud used by this cycle.
        KeyValue("obstacle_source_stamp_ns", std::to_string(obstacles_.stamp_ns)),
        KeyValue("snapshot_state_stamp_ns", std::to_string(state_.stamp_ns)),
        KeyValue("safe_command_stamp_ns", std::to_string(cycle_diag_.command_stamp_ns)),
        KeyValue("reason", reason_),
        KeyValue("t_total_ms", std::to_string(total_ms)),
        KeyValue("t_rollout_ms", std::to_string(rollout_ms)),
        KeyValue("t_cost_ms", std::to_string(cost_ms)),
        KeyValue("t_safety_ms", std::to_string(safety_ms)),
        KeyValue("t_conditioner_ms", std::to_string(conditioner_ms)),
        KeyValue("t_scheduling_delay_ms",
                 std::to_string(cycle_diag_.scheduling_delay_ms)),
        KeyValue("t_wall_cycle_ms", std::to_string(cycle_diag_.wall_cycle_ms)),
        KeyValue("t_input_snapshot_ms",
                 std::to_string(cycle_diag_.input_snapshot_ms)),
        KeyValue("t_path_reference_ms",
                 std::to_string(cycle_diag_.path_reference_ms)),
        KeyValue("t_sampling_ms", std::to_string(cycle_diag_.sampling_ms)),
        KeyValue("t_optimizer_update_ms",
                 std::to_string(cycle_diag_.optimizer_update_ms)),
        KeyValue("t_publish_ms", std::to_string(cycle_diag_.publish_ms)),
        KeyValue("t_diagnostics_ms", std::to_string(diagnostics_ms)),
        KeyValue("samples", std::to_string(samples)),
        KeyValue("safe_samples", std::to_string(safe_samples)),
        KeyValue("obstacle_points",
                 std::to_string(obstacles_.points_enu_m.size())),
        KeyValue("min_obstacle_points", std::to_string(min_obstacle_points_)),
        KeyValue("ess", std::to_string(ess)),
        KeyValue("minimum_clearance_m", std::to_string(clearance)),
        KeyValue("best_feasible_cost", std::to_string(best_feasible_cost)),
        KeyValue("cross_track_error_m",
                 reference_path_
                     ? std::to_string(reference_path_->Distance(
                           state_.position_enu_m))
                     : "0"),
        KeyValue("deadline_miss",
                 mode_ == Mode::kPlannerTimeout ? "true" : "false"),
        KeyValue("path_progress_m",
                 reference_path_ ? std::to_string(reference_path_->Progress(
                                       state_.position_enu_m))
                                 : "0"),
        KeyValue("speed_total_m_s", std::to_string(cycle_diag_.speed_total_m_s)),
        KeyValue("speed_xy_m_s", std::to_string(cycle_diag_.speed_xy_m_s)),
        KeyValue("speed_z_m_s", std::to_string(cycle_diag_.speed_z_m_s)),
        KeyValue("yaw_rad", std::to_string(cycle_diag_.yaw_rad)),
        KeyValue("measured_yaw_rate_rad_s", std::to_string(state_.yaw_rate_enu_rad_s)),
        KeyValue("yaw_rate_rad_s", std::to_string(cycle_diag_.yaw_rate_rad_s)),
        KeyValue("path_tangent_heading_rad",
                 std::to_string(cycle_diag_.path_tangent_heading_rad)),
        KeyValue("heading_error_rad",
                 std::to_string(cycle_diag_.heading_error_rad)),
        KeyValue("path_curvature_per_m",
                 std::to_string(cycle_diag_.path_curvature_per_m)),
        KeyValue("distance_to_next_turn_m",
                 std::to_string(cycle_diag_.distance_to_next_turn_m)),
        KeyValue("stopping_distance_m",
                 std::to_string(cycle_diag_.stopping_distance_m)),
        KeyValue("reference_speed_cap_m_s",
                 std::to_string(cycle_diag_.reference_speed_cap_m_s)),
        KeyValue("minimum_collision_clearance_m",
                 std::to_string(cycle_diag_.minimum_collision_clearance_m)),
        KeyValue("minimum_stopping_clearance_m",
                 std::to_string(cycle_diag_.minimum_stopping_clearance_m)),
        KeyValue("warm_start_u_x", std::to_string(cycle_diag_.warm_start_u[0])),
        KeyValue("warm_start_u_y", std::to_string(cycle_diag_.warm_start_u[1])),
        KeyValue("warm_start_u_z", std::to_string(cycle_diag_.warm_start_u[2])),
        KeyValue("warm_start_u_yaw",
                 std::to_string(cycle_diag_.warm_start_u[3])),
        KeyValue("selected_u_x", std::to_string(cycle_diag_.selected_u[0])),
        KeyValue("selected_u_y", std::to_string(cycle_diag_.selected_u[1])),
        KeyValue("selected_u_z", std::to_string(cycle_diag_.selected_u[2])),
        KeyValue("selected_u_yaw", std::to_string(cycle_diag_.selected_u[3])),
        KeyValue("selected_source",
                 ProposalSourceName(static_cast<mppi::ProposalSource>(
                     cycle_diag_.selected_source))),
        KeyValue("N_total", std::to_string(last_rejection_.total)),
        KeyValue("N_safe", std::to_string(last_rejection_.safe)),
        KeyValue("reject_non_finite",
                 std::to_string(last_rejection_.reject_non_finite)),
        KeyValue("reject_swept_collision",
                 std::to_string(last_rejection_.reject_swept_collision)),
        KeyValue("reject_static_collision",
                 std::to_string(last_rejection_.reject_static_collision)),
        KeyValue("reject_dynamic_collision",
                 std::to_string(last_rejection_.reject_dynamic_collision)),
        KeyValue("reject_stopping_distance",
                 std::to_string(last_rejection_.reject_stopping_distance)),
        KeyValue("reject_other",
                 std::to_string(last_rejection_.reject_other)),
        KeyValue("gaussian_generated", std::to_string(gaussian.generated)),
        KeyValue("gaussian_safe", std::to_string(gaussian.safe)),
        KeyValue("gaussian_best_cost", std::to_string(gaussian.best_cost)),
        KeyValue("reference_generated", std::to_string(reference.generated)),
        KeyValue("reference_safe", std::to_string(reference.safe)),
        KeyValue("reference_best_cost", std::to_string(reference.best_cost)),
        KeyValue("braking_generated", std::to_string(braking.generated)),
        KeyValue("braking_safe", std::to_string(braking.safe)),
        KeyValue("braking_best_cost", std::to_string(braking.best_cost)),
        KeyValue("recovery_generated", std::to_string(recovery.generated)),
        KeyValue("recovery_safe", std::to_string(recovery.safe)),
        KeyValue("recovery_best_cost", std::to_string(recovery.best_cost)),
        KeyValue("specific_generated", std::to_string(specific.generated)),
        KeyValue("specific_safe", std::to_string(specific.safe)),
        KeyValue("specific_best_cost", std::to_string(specific.best_cost)),
        KeyValue("local_active_path_id",
                 std::to_string(cycle_diag_.local_active_path_id)),
        KeyValue("previous_path_id",
                 std::to_string(cycle_diag_.previous_path_id)),
        KeyValue("active_path_stamp_ns",
                 std::to_string(cycle_diag_.active_path_stamp_ns)),
        KeyValue("solves_on_active_path",
                 std::to_string(cycle_diag_.solves_on_active_path)),
        KeyValue("first_solve_on_active_path",
                 cycle_diag_.first_solve_on_active_path ? "true" : "false"),
        KeyValue("first_safe_command_on_active_path",
                 cycle_diag_.first_safe_command_on_active_path ? "true"
                                                              : "false"),
        KeyValue("stopping_recovery",
                 stopping_recovery_tracker_.active() ? "true" : "false"),
        KeyValue("stopping_recovery_entries",
                 std::to_string(stopping_recovery_tracker_.entries())),
        KeyValue("stopping_recovery_exits",
                 std::to_string(stopping_recovery_tracker_.exits())),
        KeyValue("stopping_recovery_cycles",
                 std::to_string(stopping_recovery_tracker_.cycles())),
        KeyValue("stopping_recovery_episode_cycles",
                 std::to_string(
                     stopping_recovery_tracker_.episode_cycles()))};
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }
}
