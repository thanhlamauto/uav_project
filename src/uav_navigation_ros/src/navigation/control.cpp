#include "uav_navigation_ros/local_navigation_node.hpp"

using namespace std::chrono_literals;
namespace uav_navigation_ros {
using namespace navigation_detail;

void LocalNavigationNode::ControlCycle(){
    const auto cycle_started = std::chrono::steady_clock::now();
    const auto ms_since = [](SteadyTime start) {
      return std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - start)
          .count();
    };
    ++cycle_counter_;
    cycle_diag_ = CycleDiagnostics{};
    last_rejection_ = mppi::RejectionCounts{};
    last_proposal_stats_ = {};
    const double period_ms = 1000.0 / control_rate_hz_;
    if (have_previous_cycle_) {
      cycle_diag_.wall_cycle_ms = std::chrono::duration<double, std::milli>(
                                      cycle_started - previous_cycle_start_)
                                      .count();
      cycle_diag_.scheduling_delay_ms = cycle_diag_.wall_cycle_ms - period_ms;
      if (std::abs(cycle_diag_.scheduling_delay_ms) > 3.0 * period_ms)
        cycle_diag_.scheduling_delay_ms = 0.0;
    }
    previous_cycle_start_ = cycle_started;
    have_previous_cycle_ = true;

    const auto age = [&](SteadyTime stamp) {
      return std::chrono::duration<double>(cycle_started - stamp).count();
    };
    const std::string input_error =
        !odometry_error_.empty()
            ? odometry_error_
            : (!path_error_.empty() ? path_error_ : obstacle_error_);
    if (!input_error.empty()) {
      Hold(Mode::kInvalidInput, input_error);
      return;
    }
    if (!have_state_) {
      Hold(Mode::kWaitingForState, "no odometry received");
      return;
    }
    if (!reference_path_) {
      Hold(Mode::kWaitingForPath, "no global path received");
      return;
    }
    if (!have_obstacles_) {
      Hold(Mode::kWaitingForObstacles, "no obstacle observation received");
      return;
    }
    if (age(last_state_received_) > state_timeout_s_ ||
        age(last_obstacles_received_) > obstacle_timeout_s_) {
      Hold(Mode::kHoldStale, "odometry or obstacle observation is stale");
      return;
    }
    if (max_obstacle_source_age_s_ > 0.0) {
      const double obstacle_source_age_s =
          static_cast<double>(now().nanoseconds() - obstacles_.stamp_ns) * 1e-9;
      if (obstacles_.stamp_ns <= 0 || obstacle_source_age_s < -0.05 ||
          obstacle_source_age_s > max_obstacle_source_age_s_) {
        Hold(Mode::kHoldStale, "obstacle source timestamp is stale or invalid");
        return;
      }
    }
    if (obstacles_.points_enu_m.size() < min_obstacle_points_) {
      Hold(Mode::kWaitingForObstacles,
           "only " + std::to_string(obstacles_.points_enu_m.size()) +
               " finite obstacle points; require " +
               std::to_string(min_obstacle_points_));
      return;
    }
    if (Distance(state_.position_enu_m, goal_) <= goal_tolerance_m_ &&
        Norm(state_.velocity_enu_m_s) <= goal_speed_tolerance_m_s_) {
      Hold(Mode::kGoalReached, "terminal position and speed reached");
      return;
    }
    const bool route_blocked =
        (route_block_hold_enabled_ || blocked_route_detour_enabled_) &&
        mppi::RouteBlockedByCloud(*reference_path_, state_.position_enu_m,
                                  obstacles_.points_enu_m,
                                  route_block_lookahead_m_,
                                  route_block_radius_m_);
    if (route_block_hold_enabled_) {
      if (!route_blocked_latched_ && route_blocked) {
        route_blocked_latched_ = true;
        RCLCPP_WARN(get_logger(),
                    "Observed obstacle blocks the active global route; holding until a new route is received");
      }
      if (route_blocked_latched_) {
        Hold(Mode::kRouteBlocked,
             "occupied point intersects active route; replan required");
        return;
      }
    }

    const auto input_snapshot_started = std::chrono::steady_clock::now();
    if (const auto &previous = conditioner_->previous())
      state_.applied_control = *previous;
    else
      state_.applied_control = {state_.velocity_enu_m_s, state_.yaw_rate_enu_rad_s};
    mppi::CostContext context;
    context.goal_enu_m = goal_;
    context.obstacles_enu_m = obstacles_.points_enu_m;
    context.reference_path = reference_path_;
    context.initial_state = state_;
    if (blocked_route_detour_enabled_ && route_blocked)
      context.path_weight_scale = blocked_route_path_scale_;
    const double reference_speed_cap = ComputeReferenceSpeedCap();
    context.reference_speed_limit_m_s = reference_speed_cap;
    cycle_diag_.reference_speed_cap_m_s = reference_speed_cap;
    const core::Control warm_start = nominal_.front();
    FillMotionDiagnostics(warm_start);
    cycle_diag_.input_snapshot_ms = ms_since(input_snapshot_started);

    const auto sampling_started = std::chrono::steady_clock::now();
    const auto noise = noise_sampler_->Sample(samples_, horizon_);
    cycle_diag_.sampling_ms = ms_since(sampling_started);

    mppi::ControlBatch proposals;
    std::vector<mppi::ProposalSource> proposal_sources;
    const auto reference_started = std::chrono::steady_clock::now();
    if (proactive_proposals_) {
      mppi::RecoveryProposalConfig proposal_config;
      proposal_config.horizon = horizon_;
      proposal_config.max_samples = samples_;
      proposal_config.dt_s = dt_s_;
      proposal_config.reference_speed_m_s =
          std::min(optimizer_->config().maximum[0], reference_speed_cap);
      proposal_config.minimum = optimizer_->config().minimum;
      proposal_config.maximum = optimizer_->config().maximum;
      proposal_config.proactive = true;
      auto recovered = mppi::BuildRecoveryProposalsWithSources(
          reference_path_.get(),
          reference_path_->Progress(state_.position_enu_m),
          std::hypot(state_.velocity_enu_m_s.x, state_.velocity_enu_m_s.y),
          proposal_config);
      proposals = std::move(recovered.proposals);
      proposal_sources = std::move(recovered.sources);
    }
    if (blocked_route_detour_enabled_ && route_blocked &&
        proposals.size() + 2 <= samples_) {
      const double progress = reference_path_->Progress(state_.position_enu_m);
      const auto a = reference_path_->Sample(progress);
      const auto b = reference_path_->Sample(
          std::min(reference_path_->length_m(), progress + 0.5));
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      if (length > 1e-6) {
        const core::Vec3 lateral{-(b.y - a.y) / length,
                                 (b.x - a.x) / length, 0.0};
        if (detour_side_ == 0) {
          double side_occupancy = 0.0;
          for (const auto &point : obstacles_.points_enu_m) {
            const double distance = Distance(point, state_.position_enu_m);
            if (distance < 5.0)
              side_occupancy +=
                  (point.x - state_.position_enu_m.x) * lateral.x +
                  (point.y - state_.position_enu_m.y) * lateral.y;
          }
          detour_side_ = side_occupancy >= 0.0 ? -1 : 1;
          RCLCPP_WARN(get_logger(),
                      "Blocked route: testing lateral MPPI proposals, side=%d",
                      detour_side_);
        }
        const double speed = std::min(reference_speed_cap,
                                      optimizer_->config().maximum[0]);
        auto lateral_proposals = mppi::BuildLateralDetourProposals(
            *reference_path_, state_.position_enu_m, detour_side_, speed,
            horizon_);
        for (auto &actions : lateral_proposals) {
          proposals.push_back(std::move(actions));
          proposal_sources.push_back(mppi::ProposalSource::kRecoveryProposal);
        }
      }
    }
    cycle_diag_.path_reference_ms = ms_since(reference_started);

    const auto optimizer_started = std::chrono::steady_clock::now();
    mppi::MppiOptimizationResult result;
    try {
      result = optimizer_->OptimizeInjected(
          *motion_model_, *cost_evaluator_, context, state_, nominal_, {},
          noise, dt_s_, proposals, safety_checker_.get(), &obstacles_,
          proposal_sources.empty() ? nullptr : &proposal_sources);
    } catch (const std::exception &error) {
      Hold(Mode::kInvalidInput,
           std::string("planner exception: ") + error.what());
      return;
    }
    const double optimizer_total_ms = ms_since(optimizer_started);
    cycle_diag_.optimizer_update_ms =
        std::max(0.0, optimizer_total_ms - result.rollout_time_ms -
                          result.cost_time_ms - result.safety_time_ms);
    last_rejection_ = result.rejection_counts;
    last_proposal_stats_ = result.proposal_source_stats;
    const double planner_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_started)
            .count();
    if (planner_ms > max_compute_time_ms_) {
      Hold(Mode::kPlannerTimeout, "MPPI output exceeded control deadline",
           planner_ms, result.rollout_time_ms, result.cost_time_ms,
           result.safety_time_ms);
      return;
    }

    mppi::ControlSequence selected_actions = result.updated_nominal;
    mppi::MppiTrajectory selected =
        mppi::Rollout(*motion_model_, state_, selected_actions, dt_s_);
    auto safety_started = std::chrono::steady_clock::now();
    auto final_safety = safety_checker_->Evaluate(
        ToCoreState(state_), ToCoreTrajectory(selected), obstacles_);
    double final_safety_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - safety_started)
            .count();
    cycle_diag_.minimum_collision_clearance_m =
        final_safety.minimum_collision_clearance_m;
    cycle_diag_.minimum_stopping_clearance_m =
        final_safety.minimum_stopping_clearance_m;
    if (!final_safety.safe) {
      std::optional<std::size_t> best;
      for (std::size_t i = 0; i < result.safe.size(); ++i)
        if (result.safe[i] &&
            (!best || result.total_costs[i] < result.total_costs[*best]))
          best = i;
      if (!best) {
        if (TryStoppingRecovery(result, cycle_started))
          return;
        ExitStoppingRecovery();
        last_result_ = std::move(result);
        mode_ = Mode::kNoSafeTrajectory;
        reason_ = final_safety.reason;
        std::fill(nominal_.begin(), nominal_.end(), core::Control{});
        conditioner_->Reset();
        selected_trajectory_.reset();
        PublishDiagnostic(planner_ms, last_result_->rollout_time_ms,
                          last_result_->cost_time_ms,
                          last_result_->safety_time_ms + final_safety_ms,
                          last_result_->weights.size(), 0, 0.0,
                          std::min(final_safety.minimum_collision_clearance_m,
                                   final_safety.minimum_stopping_clearance_m));
        return;
      }
      selected_actions = result.perturbed_actions[*best];
      selected = result.trajectories[*best];
      cycle_diag_.selected_source = static_cast<int>(
          result.sample_sources.empty() ? mppi::ProposalSource::kGaussian
                                        : result.sample_sources[*best]);
      final_safety = safety_checker_->Evaluate(
          ToCoreState(state_), ToCoreTrajectory(selected), obstacles_);
      cycle_diag_.minimum_collision_clearance_m =
          final_safety.minimum_collision_clearance_m;
      cycle_diag_.minimum_stopping_clearance_m =
          final_safety.minimum_stopping_clearance_m;
      if (!final_safety.safe) {
        Hold(Mode::kNoSafeTrajectory, "feasible sample failed final predicate");
        return;
      }
    }
    FillSelectedControl(selected_actions.front());

    const auto publish_started = std::chrono::steady_clock::now();
    geometry_msgs::msg::TwistStamped raw_command;
    raw_command.header.stamp = now();
    raw_command.header.frame_id = planning_frame_;
    raw_command.twist.linear.x = selected_actions.front().velocity_enu_m_s.x;
    raw_command.twist.linear.y = selected_actions.front().velocity_enu_m_s.y;
    raw_command.twist.linear.z = selected_actions.front().velocity_enu_m_s.z;
    raw_command.twist.angular.z = selected_actions.front().yaw_rate_enu_rad_s;
    raw_command_publisher_->publish(raw_command);

    const auto conditioner_started = std::chrono::steady_clock::now();
    const auto safe_control =
        conditioner_->Apply(ToCoreState(state_), selected_actions.front());
    const double conditioner_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - conditioner_started)
            .count();
    const double total_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cycle_started)
            .count();
    if (total_ms > max_compute_time_ms_) {
      Hold(Mode::kPlannerTimeout, "safe command became late before publish",
           total_ms, result.rollout_time_ms, result.cost_time_ms,
           result.safety_time_ms + final_safety_ms);
      return;
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    command.twist.linear.x = safe_control.velocity_enu_m_s.x;
    command.twist.linear.y = safe_control.velocity_enu_m_s.y;
    command.twist.linear.z = safe_control.velocity_enu_m_s.z;
    command.twist.angular.z = safe_control.yaw_rate_enu_rad_s;
    cycle_diag_.command_stamp_ns = rclcpp::Time(command.header.stamp).nanoseconds();
    command_publisher_->publish(command);
    cycle_diag_.publish_ms = ms_since(publish_started);

    nominal_ = std::move(selected_actions);
    ExitStoppingRecovery();
    mode_ = Mode::kActive;
    reason_ = "safe command published";
    selected_trajectory_ = std::move(selected);
    const auto safe_count = static_cast<std::size_t>(
        std::count(result.safe.begin(), result.safe.end(), true));
    double squared_weights = 0.0;
    for (double weight : result.weights)
      squared_weights += weight * weight;
    const double ess = squared_weights > 0 ? 1.0 / squared_weights : 0.0;
    last_result_ = std::move(result);
    double best_feasible_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < last_result_->total_costs.size(); ++i)
      if (last_result_->safe[i])
        best_feasible_cost =
            std::min(best_feasible_cost, last_result_->total_costs[i]);
    ++solves_on_active_path_;
    if (!have_first_solve_on_path_) {
      have_first_solve_on_path_ = true;
      cycle_diag_.first_solve_on_active_path = true;
    }
    if (!have_first_safe_on_path_) {
      have_first_safe_on_path_ = true;
      cycle_diag_.first_safe_command_on_active_path = true;
    }
    PublishDiagnostic(total_ms, last_result_->rollout_time_ms,
                      last_result_->cost_time_ms,
                      last_result_->safety_time_ms + final_safety_ms,
                      last_result_->weights.size(), safe_count, ess,
                      std::min(final_safety.minimum_collision_clearance_m,
                               final_safety.minimum_stopping_clearance_m),
                      conditioner_ms, best_feasible_cost);
  }
}
