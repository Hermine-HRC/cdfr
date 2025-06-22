cmake_minimum_required(VERSION 3.8)
project(herminebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hrc_interfaces REQUIRED)
find_package(hrc_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
    pluginlib
    nav2_costmap_2d
)

add_library(costmap_layers SHARED
    src/costmap_obstacle_layer.cpp
)

ament_target_dependencies(costmap_layers ${dependencies})

add_executable(map_modifier src/main_map_modifier.cpp)

add_library(map_modifier_lib SHARED src/map_modifier.cpp)

ament_target_dependencies(map_modifier rclcpp)
ament_target_dependencies(map_modifier_lib rclcpp nav2_util nav_msgs hrc_interfaces geometry_msgs rclcpp_components hrc_utils OpenCV)

target_link_libraries(map_modifier map_modifier_lib)
rclcpp_components_register_nodes(map_modifier_lib "hrc_map::MapModifier")

add_executable(robot_triangulation src/main_robot_triangulation.cpp)
add_library(robot_triangulation_lib SHARED src/robot_triangulation.cpp)
ament_target_dependencies(robot_triangulation_lib rclcpp hrc_interfaces nav_msgs sensor_msgs visualization_msgs rclcpp_components tf2_ros nav2_util hrc_utils)
target_link_libraries(robot_triangulation robot_triangulation_lib)
rclcpp_components_register_nodes(robot_triangulation_lib "hrc_localization::RobotTriangulation")

add_library(costmap_filters SHARED
  src/costmap_keepout_filter.cpp
)

ament_target_dependencies(costmap_filters ${dependencies})

add_library(hrc_regulated_rotation_controller SHARED src/hrc_regulated_rotation_controller.cpp)
ament_target_dependencies(hrc_regulated_rotation_controller ${dependencies} nav2_core nav2_util)

install(TARGETS costmap_layers map_modifier_lib costmap_filters hrc_regulated_rotation_controller robot_triangulation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS map_modifier robot_triangulation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
    DIRECTORY launch rviz params maps behavior_tree
    DESTINATION share/${PROJECT_NAME}
)

function(generate_from_erb)
  add_custom_command(
      OUTPUT ${OUTPUT_FILE}
      COMMAND ${ARGS} erb ${SOURCE_FILE} > ${OUTPUT_FILE}
      DEPENDS ${SOURCE_FILE}
      COMMENT "Generating file from ERB template"
      VERBATIM
  )
endfunction()

# Create output folders
file(MAKE_DIRECTORY
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/params
)

set(SOURCES
    params/nav2_params_herminebot_omni.erb
)
set(FILES
    params/nav2_params_herminebot_omni.yaml
)

foreach(SOURCE FILE IN ZIP_LISTS SOURCES FILES)
    set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE})
    set(OUTPUT_FILE ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${FILE})

    generate_from_erb()
    string(REPLACE "/" "_" RENAMED ${FILE})
    add_custom_target(generate_${RENAMED} ALL DEPENDS ${OUTPUT_FILE})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} costmap_layers map_modifier_lib costmap_filters hrc_regulated_rotation_controller)
ament_export_dependencies(${dependencies} rclcpp hrc_interfaces nav_msgs geometry_msgs sensor_msgs visualization_msgs nav2_util OpenCV nav2_core tf2_ros)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_description/costmap_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core plugin_description/hrc_regulated_rotation_controller.xml)

ament_package()
