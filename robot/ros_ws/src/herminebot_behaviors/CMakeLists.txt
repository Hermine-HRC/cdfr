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

include_directories(
  include
)

set(dependencies
  nav2_behaviors
  hrc_interfaces
  behaviortree_cpp_v3
)

add_library(hrc_move_elevator_behavior SHARED
  src/move_elevator.cpp
)

ament_target_dependencies(hrc_move_elevator_behavior
  ${dependencies}
)

add_library(hrc_move_elevator_action_bt_node SHARED src/move_elevator_action.cpp)
list(APPEND plugin_libs hrc_move_elevator_action_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS hrc_move_elevator_behavior
                ${plugin_libs}
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

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(
  hrc_move_elevator_behavior
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})

ament_package()
