cmake_minimum_required(VERSION 3.16)
project(prequalification_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 / ament dependencies ─────────────────────────────────────────────────
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(vision_msgs      REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2              REQUIRED)
find_package(ament_index_cpp  REQUIRED)

# ── BehaviorTree.CPP (installed via apt or manually) ─────────────────────────
find_package(behaviortree_cpp REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(prequalification
    main.cpp
    bt_nodes.cpp
)

target_include_directories(prequalification PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(prequalification
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    vision_msgs
    custom_interfaces
    tf2
    ament_index_cpp
)

target_link_libraries(prequalification
    behaviortree_cpp::behaviortree_cpp
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS prequalification
    DESTINATION lib/${PROJECT_NAME}
)

# XML tree file installed to share/ so ament_index_cpp can locate it at runtime
install(FILES 
    prequalification.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
