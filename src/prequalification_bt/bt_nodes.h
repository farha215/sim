/**
 * @file bt_nodes.h
 * @brief Behavior Tree node declarations for the RoboSub pre-qualification mission.
 * @license Apache-2.0
 */

#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "custom_interfaces/msg/to_pico.hpp"

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

/**
 * @brief Simple Pose structure for waypoint management.
 */
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
}

/**
 * @brief Shared context for Behavior Tree nodes to access ROS 2 interfaces and sensor data.
 */
struct RobotContext {
    rclcpp::Node::SharedPtr node;
    std::mutex mtx;

    sensor_msgs::msg::Imu::SharedPtr               latest_imu;
    double                                         latest_altimeter = 0.0;
    double                                         target_depth = 0.0;
    vision_msgs::msg::Detection3DArray::SharedPtr  latest_detections;

    bool imu_received  = false;

    rclcpp::Publisher<custom_interfaces::msg::ToPico>::SharedPtr pico_pub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             alt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub;

    /**
     * @brief Retrieves the current pose estimated from sensors.
     */
    Pose getCurrentPose() {
        std::lock_guard<std::mutex> g(mtx);
        Pose p;
        p.x = 0.0; p.y = 0.0;
        p.z = latest_altimeter;

        if (latest_imu) {
            tf2::Quaternion q(latest_imu->orientation.x, latest_imu->orientation.y,
                              latest_imu->orientation.z, latest_imu->orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            p.yaw = yaw;
        }
        return p;
    }

    /**
     * @brief Checks if a specific object is currently detected.
     */
    bool isObjectSeen(const std::string& object) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) return false;
        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& hyp = det.results[0].hypothesis;
            const auto& id = hyp.class_id;
            const auto& score = hyp.score;

            if (object == "GATE" && id == "preq_gate" && score >= 0.6) return true;
            if (object == "POLE" && id == "preq_pole" && score >= 0.3) return true;
        }
        return false;
    }

    /**
     * @brief Gets the 3D position of a detected object.
     */
    bool getObjectPosition(const std::string& object, double& ox, double& oy, double& oz) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections) return false;

        for (const auto& det : latest_detections->detections) {
            if (det.results.empty()) continue;
            const auto& hyp = det.results[0].hypothesis;
            const auto& id = hyp.class_id;
            const auto& score = hyp.score;

            if (((object == "GATE" && id == "preq_gate" && score >= 0.6) ||
                 (object == "POLE" && id == "preq_pole" && score >= 0.3))) {
                ox = det.bbox.center.position.x;
                oy = det.bbox.center.position.y;
                oz = det.bbox.center.position.z;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Publishes control setpoints to the Pico controller.
     */
    void publishToPico(float delta_yaw, float delta_d, float target_depth_val, uint8_t stop_bit) {
        custom_interfaces::msg::ToPico msg;
        msg.delta_yaw = delta_yaw;
        msg.delta_d = delta_d;
        msg.target_depth = target_depth_val;
        msg.stop_bit = stop_bit;
        pico_pub->publish(msg);
    }

    /**
     * @brief Commands the robot to stop all horizontal motion.
     */
    void stopMotion() { 
        publishToPico(0.0f, 0.0f, (float)target_depth, 1); 
    }
};

// --- Math Utilities --------------------------------------------------------

inline double clampVal(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
inline double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// --- Condition Nodes -------------------------------------------------------

class AllSystemsOK : public BT::ConditionNode {
public:
    AllSystemsOK(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class IsObjectSeen : public BT::ConditionNode {
public:
    IsObjectSeen(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("object") }; }
    BT::NodeStatus tick() override;
};

// --- Action Nodes ----------------------------------------------------------

class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<double>("target_depth"), BT::InputPort<double>("staystill") }; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    double target_z_ = 0.0, depth_tolerance_ = 0.15;
    double staystill_ = 0.0;
    std::chrono::steady_clock::time_point stay_still_start_;
    bool in_stay_still_ = false;
};

class Do360Turn : public BT::StatefulActionNode {
public:
    Do360Turn(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("success_when_seen") }; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    std::string target_object_;
    double prev_yaw_ = 0.0, accumulated_yaw_ = 0.0;
};

class DriveThruGate : public BT::StatefulActionNode {
public:
    DriveThruGate(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("gate_depth"), BT::InputPort<double>("staystill"), BT::OutputPort<Pose>("entry_pose") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { ALIGN, DRIVE, STAY_STILL };
    Phase phase_ = Phase::ALIGN;
    Pose entry_pose_;
    double gate_depth_ = 6.0, start_time_ = 0.0, align_start_time_ = 0.0, gate_drive_time_ = 0.0, gate_lost_time_ = 0.0;
    bool align_started_ = false, gate_lost_started_ = false;
    double staystill_ = 0.0;
    std::chrono::steady_clock::time_point stay_still_start_;
};

class NavigateTo : public BT::StatefulActionNode {
public:
    NavigateTo(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<Pose>("to"), BT::InputPort<bool>("reverse"), BT::InputPort<double>("duration") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    Pose target_;
    double start_time_ = 0.0, duration_ = 15.0;
    bool surge_started_ = false;
};

class OrbitPole : public BT::StatefulActionNode {
public:
    OrbitPole(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("object"), 
                 BT::InputPort<double>("threshold"), 
                 BT::InputPort<double>("staystill"),
                 BT::InputPort<double>("surge_duration") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    enum class Phase { ALIGN, APPROACH, TURN, SURGE, STAY_STILL };
    Phase phase_ = Phase::ALIGN;
    std::string target_object_;
    double threshold_ = 1.5, target_yaw_ = 0.0, locked_yaw_ = 0.0, start_time_ = 0.0, surge_duration_ = 4.0;
    int steps_completed_ = 0;
    double staystill_ = 0.0;
    std::chrono::steady_clock::time_point stay_still_start_;
};

/**
 * @brief Registration helper for the Behavior Tree factory.
 */
inline void registerAllNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<DriveThruGate>("DriveThruGate");
    factory.registerNodeType<NavigateTo>("NavigateTo");
    factory.registerNodeType<OrbitPole>("OrbitPole");
}
