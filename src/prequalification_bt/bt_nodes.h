#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_interfaces/srv/plan_path.hpp"

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

// ─── Waypoint type ────────────────────────────────────────────────────────────
struct Pose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

namespace BT {
template <> inline Pose convertFromString(StringView str) {
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
        throw RuntimeError("invalid Pose: expected 'x;y;z;yaw'");
    Pose p;
    p.x   = convertFromString<double>(parts[0]);
    p.y   = convertFromString<double>(parts[1]);
    p.z   = convertFromString<double>(parts[2]);
    p.yaw = convertFromString<double>(parts[3]);
    return p;
}
}  // namespace BT

// ─── Shared robot interface (stored on the blackboard) ────────────────────────
//
//  All BT nodes access sensors and actuators through this struct.
//  Created once in main() and injected via:
//      tree.rootBlackboard()->set("robot_context", ctx)
//
struct RobotContext {
    rclcpp::Node::SharedPtr node;

    std::mutex mtx;

    // Latest sensor readings (updated by ROS2 callbacks)
    nav_msgs::msg::Odometry::SharedPtr             latest_odom;
    sensor_msgs::msg::Imu::SharedPtr               latest_imu;
    double                                         latest_altimeter = 0.0;
    vision_msgs::msg::Detection3DArray::SharedPtr  latest_detections;

    bool odom_received = false;
    bool imu_received  = false;

    // Actuator publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    // Service Client for Global Planner
    rclcpp::Client<custom_interfaces::srv::PlanPath>::SharedPtr path_client;

    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Subscriptions (kept alive here so they are not destroyed)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             alt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub;

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Current AUV pose: position from /odom, yaw from /imu
    Pose getCurrentPose() {
        std::lock_guard<std::mutex> g(mtx);
        Pose p;
        if (latest_odom) {
            p.x = latest_odom->pose.pose.position.x;
            p.y = latest_odom->pose.pose.position.y;
            p.z = latest_odom->pose.pose.position.z;
        }
        if (latest_imu) {
            tf2::Quaternion q(latest_imu->orientation.x,
                              latest_imu->orientation.y,
                              latest_imu->orientation.z,
                              latest_imu->orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            p.yaw = yaw;
        }
        return p;
    }

    // Returns true when the named object is present in /detections_3d.
    //   "GATE" matches: left_gate_pole | right_gate_pole
    //   "POLE" matches: red_pole | white_pole
    bool isObjectSeen(const std::string& object) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) {
            RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "isObjectSeen: No detections received yet.");
            return false;
        }
        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& id = det.results[0].hypothesis.class_id;
            
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, 
                                 "isObjectSeen: Checking [%s] against [%s]", id.c_str(), object.c_str());

            if (object == "GATE" &&
                (id == "left_gate_pole" || id == "right_gate_pole" || id == "preq_gate")) return true;
            if (object == "POLE" &&
                (id == "red_pole"       || id == "white_pole"      || id == "preq_pole"))       return true;
            if (object == "SHARK" && id == "shark")                   return true;
        }
        return false;
    }

    // Fills (ox, oy, oz) with the object's 3-D position in the front camera
    // optical frame produced by data_distance_node:
    //   oz = depth (metres, forward)
    //   ox = horizontal offset (right = positive)
    //   oy = vertical offset   (down  = positive)
    // Returns false when the object is not detected.
    bool getObjectPosition(const std::string& object,
                           double& ox, double& oy, double& oz) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) return false;
        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& id = det.results[0].hypothesis.class_id;
            bool match = (object == "GATE" &&
                          (id == "left_gate_pole" || id == "right_gate_pole" || id == "preq_gate")) ||
                         (object == "POLE" &&
                          (id == "red_pole" || id == "white_pole" || id == "preq_pole")) ||
                         (object == "SHARK" && id == "shark");
            if (match) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                return true;
            }
        }
        return false;
    }

    // New helper: transform camera-frame object position to map frame
    bool getGlobalObjectPose(const std::string& object, Pose& out_pose) {
        double ox, oy, oz;
        rclcpp::Time stamp;
        
        {
            std::lock_guard<std::mutex> g(mtx);
            if (!getObjectPositionInternal(object, ox, oy, oz)) return false;
            stamp = latest_detections->header.stamp;
        }

        try {
            // Transform from camera optical frame to map
            geometry_msgs::msg::PoseStamped cam_pose;
            cam_pose.header.frame_id = "zed_camera_front_link_optical_frame";
            cam_pose.header.stamp = stamp;
            cam_pose.pose.position.x = ox;
            cam_pose.pose.position.y = oy;
            cam_pose.pose.position.z = oz;
            cam_pose.pose.orientation.w = 1.0;

            // Wait for transform to be available (use simulation time)
            if (!tf_buffer->canTransform("map", cam_pose.header.frame_id, tf2_ros::fromMsg(stamp), std::chrono::milliseconds(200))) {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Waiting for transform from %s to map (sim time %f)", cam_pose.header.frame_id.c_str(), stamp.seconds());
                return false;
            }

            geometry_msgs::msg::PoseStamped map_pose = tf_buffer->transform(cam_pose, "map");
            out_pose.x = map_pose.pose.position.x;
            out_pose.y = map_pose.pose.position.y;
            out_pose.z = map_pose.pose.position.z;
            
            Pose cur = getCurrentPose();
            out_pose.yaw = cur.yaw; // Default to current heading
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "TF error in getGlobalObjectPose: %s", ex.what());
            return false;
        }
    }

    // Internal version of getObjectPosition without mutex (to be called inside a lock)
    bool getObjectPositionInternal(const std::string& object,
                                   double& ox, double& oy, double& oz) {
        if (!latest_detections) return false;
        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& id = det.results[0].hypothesis.class_id;
            bool match = (object == "GATE" &&
                          (id == "left_gate_pole" || id == "right_gate_pole" || id == "preq_gate")) ||
                         (object == "POLE" &&
                          (id == "red_pole" || id == "white_pole" || id == "preq_pole")) ||
                         (object == "SHARK" && id == "shark");
            if (match) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                return true;
            }
        }
        return false;
    }

    // Publishes a 6-DOF velocity command to /cmd_vel (consumed by allocation_matrix)
    void publishCmdVel(double surge, double sway, double heave,
                       double roll_r, double pitch_r, double yaw_r) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = surge;
        cmd.linear.y  = sway;
        cmd.linear.z  = heave;
        cmd.angular.x = roll_r;
        cmd.angular.y = pitch_r;
        cmd.angular.z = yaw_r;
        cmd_vel_pub->publish(cmd);
    }

    void stopMotion() { publishCmdVel(0, 0, 0, 0, 0, 0); }
};

// ─── Math utilities ───────────────────────────────────────────────────────────
inline double clampVal(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
inline double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
inline double dist2D(double dx, double dy) {
    return std::sqrt(dx * dx + dy * dy);
}

// ─── Node declarations ────────────────────────────────────────────────────────

// 1. AllSystemsOK
//    Condition that passes only once /odom and /imu have been received.
class AllSystemsOK : public BT::ConditionNode {
public:
    AllSystemsOK(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 2. SaveToBlackboard
//    Reads current pose from /odom + /imu and writes it to the
//    blackboard entry named by the "key" output port.
//    XML usage:  <Action ID="SaveToBlackboard" key="{T0}"/>
class SaveToBlackboard : public BT::SyncActionNode {
public:
    SaveToBlackboard(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::OutputPort<Pose>("key", "Blackboard key to store current pose") };
    }
    BT::NodeStatus tick() override;
};

// 3. DiveToDepth
//    Commands heave (cmd_vel.linear.z) until odom.z reaches target_depth.
class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("target_depth", "Target z in metres") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    double target_z_        = 0.0;
    double depth_tolerance_ = 0.15;
    static constexpr double K_DEPTH   = 3.0;
    static constexpr double MAX_HEAVE = 5.0;
};

// 4. IsObjectSeen
//    Queries /detections_3d for "GATE" or "POLE".
class IsObjectSeen : public BT::ConditionNode {
public:
    IsObjectSeen(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("object", "GATE or POLE") };
    }
    BT::NodeStatus tick() override;
};

// 5. Do360Turn
//    Spins at constant yaw rate.
//    Returns SUCCESS early if the target object is detected.
//    Returns FAILURE after a full 360° revolution.
class Do360Turn : public BT::StatefulActionNode {
public:
    Do360Turn(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("success_when_seen",
                     "Return SUCCESS when this object appears: GATE or POLE") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::string target_object_;
    double      prev_yaw_        = 0.0;
    double      accumulated_yaw_ = 0.0;
    static constexpr double TURN_RATE   = 0.4;
    static constexpr double FULL_CIRCLE = 2.0 * M_PI;
};

// 6. NavigateTo
//    P-controller: turns to face target, surges forward, controls depth.
//    Returns SUCCESS when within arrival tolerance.
class NavigateTo : public BT::StatefulActionNode {
public:
    NavigateTo(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<Pose>("from", "Start waypoint (informational)"),
            BT::InputPort<Pose>("to",   "Target waypoint")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    Pose target_;
    static constexpr double ARRIVE_XY = 0.4;
    static constexpr double ARRIVE_Z  = 0.3;
    static constexpr double K_YAW     = 3.5;
    static constexpr double K_SURGE   = 2.0;
    static constexpr double K_DEPTH   = 2.5;
    static constexpr double MAX_SURGE = 5.0;
    static constexpr double MAX_YAW_R = 2.0;
    static constexpr double MAX_HEAVE = 3.0;
    static constexpr double ALIGN_RAD = 0.4;   // rad — must align before surging
};

// 7. NavigateAround
//    Three-phase orbit controller around a detected object:
//
//    APPROACH: surge toward the object until depth ≈ threshold
//    ORBIT:    rotate at constant yaw rate while holding radial distance;
//              completes after a full 360° accumulation
//    RETURN:   navigate back to return_point
class NavigateAround : public BT::StatefulActionNode {
public:
    NavigateAround(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object",       "Object to orbit: POLE"),
            BT::InputPort<Pose>       ("return_point", "Pose to return to after orbit"),
            BT::InputPort<double>     ("threshold",    "Orbit radius in metres")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    enum class Phase { APPROACH, ORBIT, RETURN };
    Phase       phase_         = Phase::APPROACH;
    std::string target_object_;
    Pose        return_point_;
    double      threshold_     = 1.5;
    double      prev_yaw_      = 0.0;
    double      orbit_yaw_acc_ = 0.0;
    static constexpr double ORBIT_RATE   = 0.35;
    static constexpr double K_RADIAL     = 2.0;
    static constexpr double K_CENTER     = 2.5;
    static constexpr double APPROACH_TOL = 0.3;
    static constexpr double RETURN_TOL   = 0.4;
};

// 8. NavigateBelowObject
//    Targets a point directly below a detected object.
class NavigateBelowObject : public BT::StatefulActionNode {
public:
    NavigateBelowObject(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object", "Object to pass below (e.g. SHARK)"),
            BT::InputPort<double>("vertical_offset", "Distance below object in metres (e.g. 0.5)")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    std::string target_object_;
    double      offset_ = 0.5;
    static constexpr double K_CENTER   = 2.5;
    static constexpr double K_SURGE    = 1.5;
    static constexpr double K_HEAVE    = 2.0;
    static constexpr double ARRIVE_XYZ = 0.5;
};

// 9. PlanPathTo
//    Calls the /plan_path service to generate a trajectory to a Pose.
class PlanPathTo : public BT::StatefulActionNode {
public:
    PlanPathTo(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<Pose>("target", "Target pose in map frame") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    rclcpp::Client<custom_interfaces::srv::PlanPath>::SharedFuture future_;
    bool request_sent_ = false;
};

// 10. WaitUntilReached
//     Returns SUCCESS when current pose is within tolerance of target pose.
class WaitUntilReached : public BT::ConditionNode {
public:
    WaitUntilReached(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<Pose>("target", "Target pose"),
            BT::InputPort<double>("tolerance", "Arrival tolerance in metres")
        };
    }
    BT::NodeStatus tick() override;
};

// 11. CalculateObjectTarget
//     Calculates a global Pose relative to a reference object pose.
class CalculateObjectTarget : public BT::SyncActionNode {
public:
    CalculateObjectTarget(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<Pose>("object_pose", "Reference global pose of the object"),
            BT::InputPort<double>("offset_forward", "Forward offset in metres"),
            BT::InputPort<double>("offset_lateral", "Lateral offset in metres (left positive)"),
            BT::InputPort<double>("offset_vertical", "Vertical offset in metres (up positive)"),
            BT::OutputPort<Pose>("target", "Calculated global pose")
        };
    }
    BT::NodeStatus tick() override;
};

// 12. MoveStraight
//     Commands constant surge velocity.
class MoveStraight : public BT::StatefulActionNode {
public:
    MoveStraight(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("speed", "Surge speed m/s") };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
};

// 13. Full360Scan
//     Spins a full 360 degrees for mapping/SLAM purposes.
//     Always returns SUCCESS after the full rotation.
class Full360Scan : public BT::StatefulActionNode {
public:
    Full360Scan(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
private:
    double prev_yaw_        = 0.0;
    double accumulated_yaw_ = 0.0;
    static constexpr double TURN_RATE   = 0.4;
    static constexpr double FULL_CIRCLE = 2.0 * M_PI;
};

// 14. TrackObject
//     Looks for an object and saves its map-frame Pose once found.
class TrackObject : public BT::StatefulActionNode {
public:
    TrackObject(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("object", "Object to find"),
            BT::OutputPort<Pose>("pose", "Global pose of the object")
        };
    }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;
};

// ─── Factory registration ──────────────────────────────────────────────────────
inline void registerAllNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<SaveToBlackboard>("SaveToBlackboard");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<NavigateTo>("NavigateTo");
    factory.registerNodeType<NavigateAround>("NavigateAround");
    factory.registerNodeType<NavigateBelowObject>("NavigateBelowObject");
    factory.registerNodeType<PlanPathTo>("PlanPathTo");
    factory.registerNodeType<WaitUntilReached>("WaitUntilReached");
    factory.registerNodeType<CalculateObjectTarget>("CalculateObjectTarget");
    factory.registerNodeType<MoveStraight>("MoveStraight");
    factory.registerNodeType<Full360Scan>("Full360Scan");
    factory.registerNodeType<TrackObject>("TrackObject");
}
