cmake_minimum_required(VERSION 3.8)
project(herminebot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
    DIRECTORY worlds models launch herminebot_gazebo
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  herminebot_gazebo/laser_to_range_node.py
  DESTINATION lib/${PROJECT_NAME}
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

# make worlds
execute_process(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/worlds) # Create the world directory

set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/worlds/world.erb)

set(FILES full.world blue.world yellow.world)
set(ARGS_VAL "" USE_YELLOW=false USE_BLUE=false)
foreach(FILE ARGS IN ZIP_LISTS FILES ARGS_VAL)
    set(OUTPUT_FILE ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/worlds/${FILE})
    generate_from_erb()
    add_custom_target(generate_${FILE} ALL DEPENDS ${OUTPUT_FILE})
endforeach()


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

ament_package()
