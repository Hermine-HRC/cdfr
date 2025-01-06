cmake_minimum_required(VERSION 3.8)
project(herminebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hrc_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
    nav2_regulated_pure_pursuit_controller
    pluginlib
    nav2_costmap_2d
)

set(library_name hrc_regulated_pure_pursuit_controller)

add_library(${library_name} SHARED
    src/regulated_pure_pursuit_controller.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(costmap_layers SHARED
    src/costmap_obstacle_layer.cpp
)

ament_target_dependencies(costmap_layers ${dependencies})

add_executable(map_modifier src/main_map_modifier.cpp)

add_library(map_modifier_lib SHARED src/map_modifier.cpp)

ament_target_dependencies(map_modifier rclcpp)
ament_target_dependencies(map_modifier_lib rclcpp nav2_util nav_msgs hrc_interfaces geometry_msgs rclcpp_components OpenCV)

target_link_libraries(map_modifier map_modifier_lib)
rclcpp_components_register_nodes(map_modifier_lib "hrc_map::MapModifier")

add_library(costmap_filters SHARED
  src/costmap_keepout_filter.cpp
)

ament_target_dependencies(costmap_filters ${dependencies})

install(TARGETS ${library_name} costmap_layers map_modifier_lib costmap_filters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS map_modifier
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
    DIRECTORY launch rviz params maps behavior_tree
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(python_tests
    test/test_uncrustify.py
  )
  foreach(python_test ${python_tests})
      string(REPLACE "/" "_" python_test_name ${python_test})
      ament_add_pytest_test(${python_test_name} ${python_test}
          APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
          TIMEOUT 60
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
  endforeach()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} costmap_layers map_modifier_lib costmap_filters)
ament_export_dependencies(${dependencies} rclcpp hrc_interfaces nav_msgs geometry_msgs nav2_util OpenCV)

pluginlib_export_plugin_description_file(nav2_core plugin_description/hrc_regulated_pure_pursuit_controller.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_description/costmap_plugins.xml)

ament_package()
