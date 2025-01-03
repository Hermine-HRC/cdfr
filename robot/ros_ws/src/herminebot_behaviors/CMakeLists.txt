cmake_minimum_required(VERSION 3.8)
project(herminebot_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(hrc_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_core REQUIRED)

include_directories(
  include
)

set(executable_name bt_navigator)
add_executable(${executable_name}
  src/main_bt_navigator.cpp
)

set(dependencies
  nav2_behaviors
  hrc_interfaces
  behaviortree_cpp_v3
)

add_library(hrc_move_elevator_behavior SHARED
  src/behavior/move_elevator.cpp
)

ament_target_dependencies(hrc_move_elevator_behavior
  ${dependencies}
)

add_library(hrc_move_elevator_action_bt_node SHARED src/bt_plugin/move_elevator_action.cpp)
list(APPEND plugin_libs hrc_move_elevator_action_bt_node)

add_library(hrc_manage_map_service_bt_node SHARED src/bt_plugin/manage_map_service.cpp)
list(APPEND plugin_libs hrc_manage_map_service_bt_node)

add_library(hrc_navigators SHARED 
  src/bt_navigator.cpp 
  src/navigator/navigator_preemption.cpp
)

target_link_libraries(${executable_name} hrc_navigators ${plugin_libs})

ament_target_dependencies(hrc_navigators nav2_bt_navigator hrc_interfaces)

rclcpp_components_register_nodes(hrc_navigators "hrc_bt_navigator::BtNavigator")

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS hrc_move_elevator_behavior
                ${plugin_libs}
                hrc_navigators
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY behavior_trees
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${executable_name} ${library_name})

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(
  hrc_move_elevator_behavior
  ${plugin_libs}
  hrc_navigators
)
ament_export_dependencies(${dependencies} nav2_bt_navigator)

ament_package()
