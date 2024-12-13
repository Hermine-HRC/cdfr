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

install(TARGETS ${library_name} costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
    DIRECTORY launch rviz params maps behavior_tree
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} costmap_layers)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin_description/hrc_regulated_pure_pursuit_controller.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_description/costmap_plugins.xml)

ament_package()
