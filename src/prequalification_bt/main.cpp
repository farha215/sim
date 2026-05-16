#include "bt_nodes.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ── ROS2 node ──────────────────────────────────────────────────────────────
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);
    auto node = std::make_shared<rclcpp::Node>("prequalification_bt", options);

    // ── Shared robot context ───────────────────────────────────────────────────
    auto ctx       = std::make_shared<RobotContext>();
    ctx->node      = node;

    // TF infrastructure
    ctx->tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    ctx->tf_listener = std::make_shared<tf2_ros::TransformListener>(*ctx->tf_buffer);

    // Actuator publishers
    //   /cmd_vel  — legacy direct path (allocation_matrix subscribes)
    //   /to_pico  — preferred path (pico_controller closes PIDs, publishes /cmd_vel itself)
    ctx->cmd_vel_pub =
        node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    ctx->to_pico_pub =
        node->create_publisher<custom_interfaces::msg::ToPico>("/to_pico", 10);

    // Global Planner Client
    ctx->path_client = node->create_client<custom_interfaces::srv::PlanPath>("plan_path");

    // /odom  — position (from RTAB-Map visual odometry)
    ctx->odom_sub =
        node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [ctx](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_odom   = msg;
                ctx->odom_received = true;
            });

    // /imu   — orientation / yaw (BNO055, 10 Hz)
    ctx->imu_sub =
        node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [ctx](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_imu   = msg;
                ctx->imu_received = true;
            });

    // /altimeter — absolute depth (Float64, positive-down, from Gazebo bridge)
    ctx->alt_sub =
        node->create_subscription<std_msgs::msg::Float64>(
            "/altimeter", 10,
            [ctx](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_altimeter   = msg->data;
                ctx->altimeter_received = true;
            });

    // /detections_3d — 3-D bounding boxes from data_distance_node (YOLO + depth fusion)
    ctx->det_sub =
        node->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/detections_3d", 10,
            [ctx](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_detections = msg;
            });

    // ── BT factory and node registration ──────────────────────────────────────
    BT::BehaviorTreeFactory factory;
    registerAllNodes(factory);

    // ── Locate the XML tree file ───────────────────────────────────────────────
    std::string xml_path;
    if (argc > 1) {
        // Allow overriding path at runtime:  ros2 run prequalification_bt prequalification /path/to/tree.xml
        xml_path = argv[1];
    } else {
        try {
            xml_path = ament_index_cpp::get_package_share_directory("prequalification_bt")
                       + "/prequalification.xml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(node->get_logger(),
                         "Cannot find prequalification.xml: %s\n"
                         "Build and source the package, or pass the path as argv[1].",
                         e.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    RCLCPP_INFO(node->get_logger(), "Loading behaviour tree: %s", xml_path.c_str());

    auto tree = factory.createTreeFromFile(xml_path);

    // ── Inject shared context into the blackboard ──────────────────────────────
    tree.rootBlackboard()->set("robot_context", ctx);

    // ── Pre-set waypoints (replace with measured field coordinates) ───────────
    // T0 is saved dynamically by the SaveToBlackboard node at runtime.
    // T1: staging point between gate and pole (through the gate)
    // T2: exit waypoint on the far side (back through the gate toward home)
    Pose t1, t2;
    t1.x = 5.0;  t1.y = 0.0;  t1.z = 0.5;  t1.yaw = 0.0;
    t2.x = 1.5;  t2.y = 0.0;  t2.z = 0.5;  t2.yaw = 0.0;
    tree.rootBlackboard()->set("T1", t1);
    tree.rootBlackboard()->set("T2", t2);

    // ── Seed callbacks before first tick ──────────────────────────────────────
    rclcpp::spin_some(node);

    // ── Tick loop (10 Hz) ─────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "=== Starting Pre-Qualification Maneuver ===");

    constexpr auto TICK_PERIOD = std::chrono::milliseconds(100);
    BT::NodeStatus status      = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(TICK_PERIOD);
    }

    ctx->stopMotion();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "=== Pre-Qualification COMPLETE ===");
    } else {
        RCLCPP_WARN(node->get_logger(), "=== Pre-Qualification FAILED ===");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
