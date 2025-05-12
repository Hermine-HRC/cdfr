cmake_minimum_required(VERSION 3.8)
project(herminebot_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(hrc_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hrc_utils REQUIRED)

include_directories(
  include
)

set(dependencies
  nav2_behaviors
  hrc_interfaces
  behaviortree_cpp
  pluginlib
  hrc_utils
)

add_library(hrc_move_elevator_behavior SHARED
  src/behavior/move_elevator.cpp
)

ament_target_dependencies(hrc_move_elevator_behavior
  ${dependencies}
)

add_library(hrc_omni_drive_behavior SHARED
  src/behavior/omni_drive.cpp
)
ament_target_dependencies(hrc_omni_drive_behavior
  ${dependencies}
)

add_library(hrc_move_elevator_action_bt_node SHARED src/bt_plugin/move_elevator_action.cpp)
list(APPEND plugin_libs hrc_move_elevator_action_bt_node)

add_library(hrc_manage_map_service_bt_node SHARED src/bt_plugin/manage_map_service.cpp)
list(APPEND plugin_libs hrc_manage_map_service_bt_node)

add_library(hrc_omni_drive_action_bt_node SHARED src/bt_plugin/omni_drive_action.cpp)
list(APPEND plugin_libs hrc_omni_drive_action_bt_node)

add_library(hrc_navigator_preempt SHARED
  src/navigator/navigator_preemption.cpp
)
ament_target_dependencies(hrc_navigator_preempt ${dependencies})

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS hrc_move_elevator_behavior
                hrc_omni_drive_behavior
                ${plugin_libs}
                hrc_navigator_preempt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml navigator_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY behavior_trees
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(
  hrc_move_elevator_behavior
  hrc_omni_drive_behavior
  ${plugin_libs}
  hrc_navigator_preempt
)
ament_export_dependencies(${dependencies})

ament_package()
