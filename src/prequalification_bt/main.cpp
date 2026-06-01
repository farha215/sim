/**
 * @file main.cpp
 * @brief Main entry point for the pre-qualification behavior tree mission.
 * @license Apache-2.0
 */

#include "bt_nodes.h"
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <fstream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("prequalification_bt");

    // --- Shared robot context -----------------------------------------------
    auto ctx       = std::make_shared<RobotContext>();
    ctx->node      = node;

    // Actuator publishers
    ctx->pico_pub = node->create_publisher<custom_interfaces::msg::ToPico>("/to_pico", 10);

    // IMU subscription (orientation / yaw)
    ctx->imu_sub =
        node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [ctx](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_imu   = msg;
                ctx->imu_received = true;
            });

    // Altimeter subscription (altitude above pool floor)
    ctx->alt_sub =
        node->create_subscription<std_msgs::msg::Float64>(
            "/altimeter", 10,
            [ctx](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_altimeter = msg->data;
            });

    // 3D Detections subscription (YOLO + depth fusion)
    ctx->det_sub =
        node->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/detections_3d", 10,
            [ctx](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                std::lock_guard<std::mutex> g(ctx->mtx);
                ctx->latest_detections = msg;
            });

    // --- Behavior Tree Initialization ---------------------------------------
    BT::BehaviorTreeFactory factory;
    registerAllNodes(factory);

    // Dump TreeNodesModel XML for Groot2 visualization
    {
        std::string xml_models = BT::writeTreeNodesModelXML(factory);
        std::ofstream model_file("bt_nodes_model.xml");
        model_file << xml_models;
        RCLCPP_INFO(node->get_logger(), "[main] Written bt_nodes_model.xml for Groot2");
    }

    // Locate the XML mission file
    std::string xml_path;
    if (argc > 1) {
        xml_path = argv[1];
    } else {
        try {
            xml_path = ament_index_cpp::get_package_share_directory("prequalification_bt")
                       + "/prequalification.xml";
        } catch (const std::exception& e) {
            RCLCPP_FATAL(node->get_logger(), "Cannot find prequalification.xml: %s", e.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", xml_path.c_str());
    auto tree = factory.createTreeFromFile(xml_path);

    // Start Groot2 Publisher on default port 1667
    RCLCPP_INFO(node->get_logger(), "Starting Groot2 Publisher on port 1667...");
    BT::Groot2Publisher publisher(tree, 1667);

    // Inject shared context into the blackboard
    tree.rootBlackboard()->set("robot_context", ctx);

    // Seed callbacks before first tick
    rclcpp::spin_some(node);

    // --- Mission Loop -------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "RoboSub Pre-Qualification Mission: COMMENCING EXECUTION");

    constexpr auto TICK_PERIOD = std::chrono::milliseconds(100);
    BT::NodeStatus status      = BT::NodeStatus::RUNNING;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(TICK_PERIOD);
    }

    ctx->stopMotion();

    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "RoboSub Pre-Qualification Mission: STATUS SUCCESS");
    } else {
        RCLCPP_WARN(node->get_logger(), "RoboSub Pre-Qualification Mission: STATUS FAILURE");
    }

    rclcpp::shutdown();
    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
