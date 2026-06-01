/**
 * @file bt_nodes.cpp
 * @brief Implementation of Behavior Tree nodes for the RoboSub pre-qualification mission.
 * @license Apache-2.0
 */

#include "bt_nodes.h"

/**
 * @brief Retrieves the shared context from the blackboard.
 */
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    if (!cfg.blackboard->get("robot_context", ctx)) {
        throw BT::RuntimeError("MISSING robot_context on blackboard.");
    }
    return ctx;
}

// --- AllSystemsOK -----------------------------------------------------------

BT::NodeStatus AllSystemsOK::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000, 
                             "[AllSystemsOK] Awaiting IMU telemetry...");
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[AllSystemsOK] All sub-systems nominal.");
    return BT::NodeStatus::SUCCESS;
}

// --- DiveToDepth ------------------------------------------------------------

BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in) throw BT::RuntimeError("DiveToDepth: missing [target_depth]");
    target_z_ = depth_in.value();
    staystill_ = getInput<double>("staystill").value_or(0.0);
    in_stay_still_ = false;

    auto ctx = getCtx(config());
    ctx->target_depth = target_z_;
    RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Initiating descent to target depth: %.2f m", target_z_);
    
    ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (in_stay_still_) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stay_still_start_).count() >= staystill_) {
            return BT::NodeStatus::SUCCESS;
        }
        ctx->stopMotion();
        return BT::NodeStatus::RUNNING;
    }

    double current_z = ctx->getCurrentPose().z;
    if (std::abs(target_z_ - current_z) < depth_tolerance_) {
        if (staystill_ > 0.01) {
            in_stay_still_ = true;
            stay_still_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Target depth attained. Maintaining station for %.1f s.", staystill_);
            ctx->stopMotion();
            return BT::NodeStatus::RUNNING;
        }
        RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Target depth attained.");
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { getCtx(config())->stopMotion(); }

// --- IsObjectSeen -----------------------------------------------------------

BT::NodeStatus IsObjectSeen::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("IsObjectSeen: missing [object]");

    return ctx->isObjectSeen(obj.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --- Do360Turn --------------------------------------------------------------

BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj) throw BT::RuntimeError("Do360Turn: missing [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_ = ctx->getCurrentPose().yaw;
    accumulated_yaw_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Initiating 360-degree scan for: %s", target_object_.c_str());
    ctx->publishToPico(0.5f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Visual confirmation: %s detected.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    double current_yaw = ctx->getCurrentPose().yaw;
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] Scan complete. Target %s not detected in sector.", target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishToPico(0.5f, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() { getCtx(config())->stopMotion(); }

// --- DriveThruGate ----------------------------------------------------------

BT::NodeStatus DriveThruGate::onStart() {
    gate_depth_ = getInput<double>("gate_depth").value_or(3.0);
    staystill_ = getInput<double>("staystill").value_or(0.0);
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    entry_pose_ = ctx->getCurrentPose();
    phase_ = Phase::ALIGN;
    gate_lost_time_ = 0.0;
    align_started_ = false;
    gate_lost_started_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveThruGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (phase_ == Phase::STAY_STILL) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stay_still_start_).count() >= staystill_) {
            setOutput("entry_pose", entry_pose_);
            return BT::NodeStatus::SUCCESS;
        }
        ctx->stopMotion();
        return BT::NodeStatus::RUNNING;
    }

    double ox, oy, oz;
    bool gate_seen = ctx->getObjectPosition("GATE", ox, oy, oz);

    if (phase_ == Phase::ALIGN) {
        if (!gate_seen) {
            // Consistent search rotation (negative -> turn right)
            ctx->publishToPico(-0.3f, 0.0f, (float)ctx->target_depth, 0); 
            align_started_ = false;
            return BT::NodeStatus::RUNNING;
        }

        double norm_x = ox / std::max(oz, 0.5);
        if (std::abs(norm_x) < 0.1) { // Wider deadband for stability
            if (!align_started_) {
                align_start_time_ = ctx->node->get_clock()->now().seconds();
                align_started_ = true;
            }
            if (ctx->node->get_clock()->now().seconds() - align_start_time_ >= 0.4) { 
                phase_ = Phase::DRIVE;
                start_time_ = ctx->node->get_clock()->now().seconds();
                gate_drive_time_ = (oz + 15.0) / 0.5; // Safety timeout
                gate_lost_started_ = false;
                entry_pose_ = ctx->getCurrentPose(); 
                RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Alignment confirmed. Executing transit through gate.");
            } else {
                ctx->stopMotion();
            }
        } else {
            align_started_ = false;
            // Smoother proportional gain (1.5)
            float yaw_cmd = -(float)norm_x * 1.5f;
            ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    double elapsed = ctx->node->get_clock()->now().seconds() - start_time_;
    
    if (!gate_seen) {
        if (!gate_lost_started_) {
            gate_lost_time_ = ctx->node->get_clock()->now().seconds();
            gate_lost_started_ = true;
            RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Visual contact lost. Commencing 3.0s clearing surge.");
        }
        
        if (ctx->node->get_clock()->now().seconds() - gate_lost_time_ >= 3.0) {
            if (staystill_ > 0.01) {
                phase_ = Phase::STAY_STILL;
                stay_still_start_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Gate cleared. Maintaining station for %.1f s.", staystill_);
                ctx->stopMotion();
                return BT::NodeStatus::RUNNING;
            }
            ctx->stopMotion();
            setOutput("entry_pose", entry_pose_);
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        gate_lost_started_ = false;
    }

    if (elapsed >= gate_drive_time_) {
        if (staystill_ > 0.01) {
            phase_ = Phase::STAY_STILL;
            stay_still_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Safety timeout reached. Maintaining station for %.1f s.", staystill_);
            ctx->stopMotion();
            return BT::NodeStatus::RUNNING;
        }
        ctx->stopMotion();
        setOutput("entry_pose", entry_pose_);
        return BT::NodeStatus::SUCCESS;
    }

    double yaw_err = normalizeAngle(entry_pose_.yaw - ctx->getCurrentPose().yaw);
    ctx->publishToPico((float)yaw_err, 10.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void DriveThruGate::onHalted() { getCtx(config())->stopMotion(); }

// --- NavigateTo -------------------------------------------------------------

BT::NodeStatus NavigateTo::onStart() {
    auto to = getInput<Pose>("to");
    auto rev = getInput<bool>("reverse");
    auto dur = getInput<double>("duration");
    if (!to) throw BT::RuntimeError("NavigateTo: missing target [to]");
    
    target_ = to.value();
    if (rev && rev.value()) target_.yaw = normalizeAngle(target_.yaw + M_PI);
    duration_ = dur ? dur.value() : 20.0;
    
    auto ctx = getCtx(config());
    surge_started_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();
    double yaw_err = normalizeAngle(target_.yaw - cur.yaw);

    if (!surge_started_) {
        if (std::abs(yaw_err) < 0.1) {
            start_time_ = ctx->node->get_clock()->now().seconds();
            surge_started_ = true;
            RCLCPP_INFO(ctx->node->get_logger(), "[NavigateTo] Alignment attained. Executing timed surge to target waypoint.");
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }
    }

    if (ctx->node->get_clock()->now().seconds() - start_time_ >= duration_) { 
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico((float)yaw_err * 1.2f, 10.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void NavigateTo::onHalted() { getCtx(config())->stopMotion(); }

// --- SurgeForward -------------------------------------------------------------

BT::NodeStatus SurgeForward::onStart() {
    duration_ = getInput<double>("duration").value_or(5.0);
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    locked_yaw_ = ctx->getCurrentPose().yaw;
    start_time_ = ctx->node->get_clock()->now().seconds();
    RCLCPP_INFO(ctx->node->get_logger(), "[SurgeForward] Executing open-loop surge for %.1f seconds.", duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurgeForward::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->node->get_clock()->now().seconds() - start_time_ >= duration_) {
        ctx->stopMotion();
        return BT::NodeStatus::SUCCESS;
    }

    double yaw_err = normalizeAngle(locked_yaw_ - ctx->getCurrentPose().yaw);
    ctx->publishToPico((float)yaw_err * 1.2f, 10.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void SurgeForward::onHalted() { getCtx(config())->stopMotion(); }

// --- OrbitPole --------------------------------------------------------------

BT::NodeStatus OrbitPole::onStart() {
    auto obj = getInput<std::string>("object");
    auto thr = getInput<double>("threshold");
    if (!obj || !thr) throw BT::RuntimeError("OrbitPole: missing ports");
    target_object_ = obj.value();
    threshold_ = thr.value();
    staystill_ = getInput<double>("staystill").value_or(0.0);
    surge_duration_ = getInput<double>("surge_duration").value_or(4.0);
    
    steps_completed_ = 0;
    phase_ = Phase::ALIGN;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OrbitPole::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    Pose cur = ctx->getCurrentPose();

    if (phase_ == Phase::STAY_STILL) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - stay_still_start_).count() >= staystill_) {
            return BT::NodeStatus::SUCCESS;
        }
        ctx->stopMotion();
        return BT::NodeStatus::RUNNING;
    }

    if (steps_completed_ >= 8) {
        if (staystill_ > 0.01) {
            phase_ = Phase::STAY_STILL;
            stay_still_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Orbit protocol complete. Maintaining station for %.1f s.", staystill_);
            ctx->stopMotion();
            return BT::NodeStatus::RUNNING;
        }
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Orbit protocol complete.");
        return BT::NodeStatus::SUCCESS;
    }

    double ox, oy, oz;
    bool seen = ctx->getObjectPosition(target_object_, ox, oy, oz);
    if (phase_ == Phase::ALIGN) {
        if (!seen) {
            // Match Do360Turn direction (positive -> turn left) to prevent jitter
            ctx->publishToPico(0.3f, 0.0f, (float)ctx->target_depth, 0); 
            return BT::NodeStatus::RUNNING;
        }

        double norm_x = ox / std::max(oz, 0.5);
        
        // Continuous proportional control to eliminate derivative kick
        float yaw_cmd = -(float)norm_x * 1.2f;
        ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);

        if (std::abs(norm_x) < (steps_completed_ == 0 ? 0.05 : 0.15)) {
            if (steps_completed_ == 0) {
                phase_ = Phase::APPROACH;
                locked_yaw_ = cur.yaw;
                RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Precision alignment attained. Initiating approach to %.1f m.", threshold_);
            } else {
                phase_ = Phase::TURN;
                double correction = clampVal((threshold_ - oz) * 0.5, -0.4, 0.4); 
                target_yaw_ = normalizeAngle(cur.yaw - (85.0 * M_PI / 180.0) - correction); 
                RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Executing orbit sector %d/8: Aligning to tangent.", steps_completed_ + 1);
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::APPROACH) {
        if (!seen || oz < 0.5) { // Ignore if lost OR if depth drops to an impossibly small noise value
            // Maintain the straight-line surge and wait for valid depth data.
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico((float)yaw_err * 1.0f, 6.0f, (float)ctx->target_depth, 0);
            return BT::NodeStatus::RUNNING;
        }

        if (oz <= threshold_ + 0.1) {
            phase_ = Phase::TURN;
            double correction = clampVal((threshold_ - oz) * 0.5, -0.4, 0.4); 
            target_yaw_ = normalizeAngle(cur.yaw - (85.0 * M_PI / 180.0) - correction); 
            RCLCPP_INFO(ctx->node->get_logger(), "[OrbitPole] Proximity threshold reached. Sector %d/8: Aligning to tangent.", steps_completed_ + 1);
        } else {
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico((float)yaw_err * 1.0f, 6.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::TURN) {
        double yaw_err = normalizeAngle(target_yaw_ - cur.yaw);
        if (std::abs(yaw_err) < 0.08) {
            phase_ = Phase::SURGE;
            start_time_ = ctx->node->get_clock()->now().seconds();
            locked_yaw_ = cur.yaw;
        } else {
            ctx->publishToPico((float)yaw_err * 2.0f, 0.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::SURGE) {
        if (ctx->node->get_clock()->now().seconds() - start_time_ >= surge_duration_) {
            phase_ = Phase::ALIGN;
            steps_completed_++;
        } else {
            double yaw_err = normalizeAngle(locked_yaw_ - cur.yaw);
            ctx->publishToPico((float)yaw_err * 2.0f, 10.0f, (float)ctx->target_depth, 0);
        }
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
}

void OrbitPole::onHalted() { getCtx(config())->stopMotion(); }
