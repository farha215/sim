#include "bt_nodes.h"

// Retrieve the shared RobotContext from the blackboard
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    cfg.blackboard->get("robot_context", ctx);
    return ctx;
}

// ─── 1. AllSystemsOK ─────────────────────────────────────────────────────────
BT::NodeStatus AllSystemsOK::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (!ctx->odom_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(),
                             *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] Waiting for /odom ...");
        return BT::NodeStatus::RUNNING;
    }
    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(),
                             *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] Waiting for /imu ...");
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[AllSystemsOK] All systems nominal.");
    return BT::NodeStatus::SUCCESS;
}

// ─── 2. SaveToBlackboard ──────────────────────────────────────────────────────
BT::NodeStatus SaveToBlackboard::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    Pose current = ctx->getCurrentPose();
    setOutput("key", current);

    RCLCPP_INFO(ctx->node->get_logger(),
                "[SaveToBlackboard] Saved (%.2f, %.2f, %.2f, yaw=%.2f rad)",
                current.x, current.y, current.z, current.yaw);
    return BT::NodeStatus::SUCCESS;
}

// ─── 3. DiveToDepth ───────────────────────────────────────────────────────────
BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in)
        throw BT::RuntimeError("DiveToDepth: missing required port [target_depth]");
    target_z_ = depth_in.value();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(),
                "[DiveToDepth] Commanding dive to z = %.2f m", target_z_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_z = ctx->getCurrentPose().z;
    double dz        = target_z_ - current_z;

    if (std::abs(dz) < depth_tolerance_) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[DiveToDepth] Reached target depth (current z = %.2f m)", current_z);
        return BT::NodeStatus::SUCCESS;
    }

    double heave = clampVal(K_DEPTH * dz, -MAX_HEAVE, MAX_HEAVE);
    ctx->publishCmdVel(0.0, 0.0, heave, 0.0, 0.0, 0.0);
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[DiveToDepth] Halted.");
}

// ─── 4. IsObjectSeen ──────────────────────────────────────────────────────────
BT::NodeStatus IsObjectSeen::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    auto obj = getInput<std::string>("object");
    if (!obj)
        throw BT::RuntimeError("IsObjectSeen: missing required port [object]");

    bool seen = ctx->isObjectSeen(obj.value());
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "[IsObjectSeen] %s → %s",
                 obj.value().c_str(), seen ? "SEEN" : "not seen");
    return seen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── 5. Do360Turn ─────────────────────────────────────────────────────────────
BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj)
        throw BT::RuntimeError("Do360Turn: missing required port [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_        = ctx->getCurrentPose().yaw;
    accumulated_yaw_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(),
                "[Do360Turn] Searching for %s (360° sweep)", target_object_.c_str());
    ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, TURN_RATE);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    // Early success when target object appears
    if (ctx->isObjectSeen(target_object_)) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[Do360Turn] %s found — stopping rotation.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Accumulate absolute yaw change using normalised delta
    double current_yaw = ctx->getCurrentPose().yaw;
    double delta       = std::abs(normalizeAngle(current_yaw - prev_yaw_));
    accumulated_yaw_  += delta;
    prev_yaw_          = current_yaw;

    if (accumulated_yaw_ >= FULL_CIRCLE) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(),
                    "[Do360Turn] Full rotation complete — %s not found.",
                    target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, TURN_RATE);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] Halted mid-rotation.");
}

// ─── 6. NavigateTo ────────────────────────────────────────────────────────────
BT::NodeStatus NavigateTo::onStart() {
    auto to = getInput<Pose>("to");
    if (!to)
        throw BT::RuntimeError("NavigateTo: missing required port [to]");
    target_ = to.value();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(),
                "[NavigateTo] Heading to (%.2f, %.2f, %.2f)",
                target_.x, target_.y, target_.z);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    Pose cur = ctx->getCurrentPose();

    double dx      = target_.x - cur.x;
    double dy      = target_.y - cur.y;
    double dz      = target_.z - cur.z;
    double dist_xy = dist2D(dx, dy);

    // Arrival check
    if (dist_xy < ARRIVE_XY && std::abs(dz) < ARRIVE_Z) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[NavigateTo] Arrived at (%.2f, %.2f, %.2f).",
                    target_.x, target_.y, target_.z);
        return BT::NodeStatus::SUCCESS;
    }

    // Turn toward target
    double desired_yaw = std::atan2(dy, dx);
    double yaw_err     = normalizeAngle(desired_yaw - cur.yaw);
    double yaw_rate    = clampVal(K_YAW * yaw_err, -MAX_YAW_R, MAX_YAW_R);

    // Surge forward only when roughly aligned
    double surge = 0.0;
    if (std::abs(yaw_err) < ALIGN_RAD) {
        surge = clampVal(K_SURGE * dist_xy, 0.0, MAX_SURGE);
    }

    double heave = clampVal(K_DEPTH * dz, -MAX_HEAVE, MAX_HEAVE);

    ctx->publishCmdVel(surge, 0.0, heave, 0.0, 0.0, yaw_rate);
    return BT::NodeStatus::RUNNING;
}

void NavigateTo::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[NavigateTo] Halted.");
}

// ─── 7. NavigateAround ────────────────────────────────────────────────────────
BT::NodeStatus NavigateAround::onStart() {
    auto obj = getInput<std::string>("object");
    auto ret = getInput<Pose>("return_point");
    auto thr = getInput<double>("threshold");
    if (!obj || !ret || !thr)
        throw BT::RuntimeError("NavigateAround: missing required ports");

    target_object_ = obj.value();
    return_point_  = ret.value();
    threshold_     = thr.value();
    phase_         = Phase::APPROACH;
    orbit_yaw_acc_ = 0.0;

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    prev_yaw_ = ctx->getCurrentPose().yaw;

    RCLCPP_INFO(ctx->node->get_logger(),
                "[NavigateAround] Starting orbit of %s at %.2f m",
                target_object_.c_str(), threshold_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateAround::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    Pose cur = ctx->getCurrentPose();

    // ── APPROACH ─────────────────────────────────────────────────────────────
    if (phase_ == Phase::APPROACH) {
        double ox, oy, oz;
        if (!ctx->getObjectPosition(target_object_, ox, oy, oz)) {
            // Object lost — rotate slowly to reacquire
            ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, 0.2);
            return BT::NodeStatus::RUNNING;
        }

        // oz = depth to object in camera frame
        double dist_error = oz - threshold_;

        if (std::abs(dist_error) < APPROACH_TOL) {
            phase_         = Phase::ORBIT;
            prev_yaw_      = cur.yaw;
            orbit_yaw_acc_ = 0.0;
            RCLCPP_INFO(ctx->node->get_logger(),
                        "[NavigateAround] Approach done — starting orbit.");
            return BT::NodeStatus::RUNNING;
        }

        // ox: positive = object is right of centre → yaw left to centre it
        double yaw_corr = clampVal(K_CENTER * (ox / std::max(oz, 0.5)), -1.5, 1.5);
        double surge    = clampVal(K_RADIAL * dist_error, -3.0, 3.0);
        ctx->publishCmdVel(surge, 0.0, 0.0, 0.0, 0.0, yaw_corr);
        return BT::NodeStatus::RUNNING;
    }

    // ── ORBIT ─────────────────────────────────────────────────────────────────
    if (phase_ == Phase::ORBIT) {
        // Track accumulated yaw to detect a full revolution
        double delta_yaw = std::abs(normalizeAngle(cur.yaw - prev_yaw_));
        orbit_yaw_acc_  += delta_yaw;
        prev_yaw_        = cur.yaw;

        if (orbit_yaw_acc_ >= 2.0 * M_PI) {
            phase_ = Phase::RETURN;
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(),
                        "[NavigateAround] Orbit complete — returning to start point.");
            return BT::NodeStatus::RUNNING;
        }

        // Hold radial distance while rotating
        double radial_correction = 0.0;
        double yaw_correction    = 0.0;
        double ox, oy, oz;
        if (ctx->getObjectPosition(target_object_, ox, oy, oz)) {
            radial_correction = clampVal(K_RADIAL * (oz - threshold_), -2.0, 2.0);
            yaw_correction    = clampVal(K_CENTER * (ox / std::max(oz, 0.5)), -1.5, 1.5);
        }

        ctx->publishCmdVel(radial_correction, 0.0, 0.0, 0.0, 0.0,
                           ORBIT_RATE + yaw_correction);
        return BT::NodeStatus::RUNNING;
    }

    // ── RETURN ────────────────────────────────────────────────────────────────
    if (phase_ == Phase::RETURN) {
        double dx      = return_point_.x - cur.x;
        double dy      = return_point_.y - cur.y;
        double dz      = return_point_.z - cur.z;
        double dist_xy = dist2D(dx, dy);

        if (dist_xy < RETURN_TOL && std::abs(dz) < 0.4) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(),
                        "[NavigateAround] Returned to start point. SUCCESS.");
            return BT::NodeStatus::SUCCESS;
        }

        double desired_yaw = std::atan2(dy, dx);
        double yaw_err     = normalizeAngle(desired_yaw - cur.yaw);
        double yaw_rate    = clampVal(3.5 * yaw_err, -2.0, 2.0);
        double surge       = (std::abs(yaw_err) < 0.4)
                             ? clampVal(2.0 * dist_xy, 0.0, 5.0) : 0.0;
        double heave       = clampVal(2.5 * dz, -3.0, 3.0);

        ctx->publishCmdVel(surge, 0.0, heave, 0.0, 0.0, yaw_rate);
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;
}

void NavigateAround::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[NavigateAround] Halted.");
}

// ─── 8. NavigateBelowObject ───────────────────────────────────────────────────
BT::NodeStatus NavigateBelowObject::onStart() {
    auto obj = getInput<std::string>("object");
    auto off = getInput<double>("vertical_offset");
    if (!obj || !off)
        throw BT::RuntimeError("NavigateBelowObject: missing required ports");

    target_object_ = obj.value();
    offset_        = off.value();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(),
                "[NavigateBelowObject] Targeting point %.2f m below %s",
                offset_, target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateBelowObject::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz;
    if (!ctx->getObjectPosition(target_object_, ox, oy, oz)) {
        // Object lost - rotate slowly to reacquire
        ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, 0.3);
        return BT::NodeStatus::RUNNING;
    }

    // oz: depth to object (metres, forward)
    // ox: horizontal offset (right = positive)
    // oy: vertical offset (down = positive)

    // We want to reach the object's X, Z, but Y = object_Y + offset
    // Target position in camera frame: (ox, oy + offset, oz)
    
    double dist_xyz = std::sqrt(ox*ox + (oy + offset_)*(oy + offset_) + oz*oz);

    if (dist_xyz < ARRIVE_XYZ) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[NavigateBelowObject] Arrived below %s.", target_object_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Control:
    // 1. Yaw to center the object (ox)
    double yaw_rate = clampVal(K_CENTER * (ox / std::max(oz, 0.5)), -1.5, 1.5);
    
    // 2. Surge to reach depth (oz)
    double surge = clampVal(K_SURGE * oz, -2.0, 3.0);
    
    // 3. Heave to reach vertical offset (oy + offset)
    double heave = clampVal(K_HEAVE * (oy + offset_), -2.0, 2.0);

    ctx->publishCmdVel(surge, 0.0, heave, 0.0, 0.0, yaw_rate);
    return BT::NodeStatus::RUNNING;
}

void NavigateBelowObject::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
    RCLCPP_WARN(ctx->node->get_logger(), "[NavigateBelowObject] Halted.");
}

// ─── 9. PlanPathTo ────────────────────────────────────────────────────────────
BT::NodeStatus PlanPathTo::onStart() {
    auto target_in = getInput<Pose>("target");
    if (!target_in)
        throw BT::RuntimeError("PlanPathTo: missing required port [target]");

    auto ctx = getCtx(config());
    if (!ctx->path_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(ctx->node->get_logger(), "PlanPath service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<custom_interfaces::srv::PlanPath::Request>();
    request->target.position.x = target_in->x;
    request->target.position.y = target_in->y;
    request->target.position.z = target_in->z;
    request->target.orientation.w = 1.0;

    future_ = ctx->path_client->async_send_request(request);
    request_sent_ = true;
    RCLCPP_INFO(ctx->node->get_logger(), "[PlanPathTo] Request sent to (%.2f, %.2f, %.2f)",
                target_in->x, target_in->y, target_in->z);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanPathTo::onRunning() {
    if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto response = future_.get();
        auto ctx = getCtx(config());
        if (response->success) {
            RCLCPP_INFO(ctx->node->get_logger(), "[PlanPathTo] Path planned successfully.");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(ctx->node->get_logger(), "[PlanPathTo] Path planning failed: %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void PlanPathTo::onHalted() {
    request_sent_ = false;
}

// ─── 10. WaitUntilReached ─────────────────────────────────────────────────────
BT::NodeStatus WaitUntilReached::tick() {
    auto target_in = getInput<Pose>("target");
    auto tol_in = getInput<double>("tolerance");
    if (!target_in || !tol_in)
        throw BT::RuntimeError("WaitUntilReached: missing required ports");

    auto ctx = getCtx(config());
    Pose cur = ctx->getCurrentPose();
    double dist = std::sqrt(std::pow(cur.x - target_in->x, 2) + 
                            std::pow(cur.y - target_in->y, 2) + 
                            std::pow(cur.z - target_in->z, 2));

    if (dist < tol_in.value()) {
        RCLCPP_INFO(ctx->node->get_logger(), "[WaitUntilReached] Target reached (dist=%.2f)", dist);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

// ─── 11. CalculateObjectTarget ────────────────────────────────────────────────
BT::NodeStatus CalculateObjectTarget::tick() {
    auto obj_pose_in = getInput<Pose>("object_pose");
    auto off_f = getInput<double>("offset_forward");
    auto off_l = getInput<double>("offset_lateral");
    auto off_v = getInput<double>("offset_vertical");
    
    if (!obj_pose_in || !off_f || !off_l || !off_v)
        throw BT::RuntimeError("CalculateObjectTarget: missing required ports");

    auto ctx = getCtx(config());
    Pose cur = ctx->getCurrentPose();
    Pose obj_pose = obj_pose_in.value();

    // Offset in global frame: Forward is along Y, Lateral is along X
    // Target Z is set to the current AUV depth per user instruction.
    Pose target = obj_pose;
    target.y -= off_f.value(); // Subtract forward offset (along Y)
    target.x -= off_l.value(); // Subtract lateral offset (along X)
    target.z  = cur.z;         // MAINTAIN current depth for future waypoints

    setOutput("target", target);
    RCLCPP_INFO(ctx->node->get_logger(), "[CalculateObjectTarget] SUCCESS: Planned target at (%.2f, %.2f, %.2f)",
                target.x, target.y, target.z);
    return BT::NodeStatus::SUCCESS;
}

// ─── 12. MoveStraight ──────────────────────────────────────────────────────────
BT::NodeStatus MoveStraight::onStart() {
    auto speed_in = getInput<double>("speed");
    if (!speed_in)
        throw BT::RuntimeError("MoveStraight: missing required port [speed]");

    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[MoveStraight] Starting forward movement at %.2f m/s", speed_in.value());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveStraight::onRunning() {
    auto speed_in = getInput<double>("speed");
    auto ctx = getCtx(config());
    ctx->publishCmdVel(speed_in.value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    return BT::NodeStatus::RUNNING;
}

void MoveStraight::onHalted() {
    getCtx(config())->stopMotion();
}

// ─── 13. Full360Scan ──────────────────────────────────────────────────────────
BT::NodeStatus Full360Scan::onStart() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);
    prev_yaw_        = ctx->getCurrentPose().yaw;
    accumulated_yaw_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(), "[Full360Scan] Starting mapping scan (360°)...");
    ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, TURN_RATE);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Full360Scan::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_yaw = ctx->getCurrentPose().yaw;
    double delta       = std::abs(normalizeAngle(current_yaw - prev_yaw_));
    accumulated_yaw_  += delta;
    prev_yaw_          = current_yaw;

    if (accumulated_yaw_ >= FULL_CIRCLE) {
        ctx->stopMotion();
        RCLCPP_INFO(ctx->node->get_logger(), "[Full360Scan] Mapping scan complete. SUCCESS.");
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishCmdVel(0.0, 0.0, 0.0, 0.0, 0.0, TURN_RATE);
    return BT::NodeStatus::RUNNING;
}

void Full360Scan::onHalted() {
    auto ctx = getCtx(config());
    ctx->stopMotion();
}

// ─── 14. TrackObject ──────────────────────────────────────────────────────────
BT::NodeStatus TrackObject::onStart() {
    auto obj = getInput<std::string>("object");
    if (!obj)
        throw BT::RuntimeError("TrackObject: missing required port [object]");
    
    RCLCPP_INFO(getCtx(config())->node->get_logger(), "[TrackObject] Looking for %s to save its global pose...", obj.value().c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackObject::onRunning() {
    auto obj = getInput<std::string>("object");
    auto ctx = getCtx(config());
    Pose obj_pose;
    if (ctx->getGlobalObjectPose(obj.value(), obj_pose)) {
        setOutput("pose", obj_pose);
        RCLCPP_INFO(ctx->node->get_logger(), "[TrackObject] SUCCESS: Captured global pose of %s at (%.2f, %.2f, %.2f)",
                    obj.value().c_str(), obj_pose.x, obj_pose.y, obj_pose.z);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void TrackObject::onHalted() {
}
