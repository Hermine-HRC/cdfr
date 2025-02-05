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
    DIRECTORY worlds models launch herminebot_gazebo config
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  herminebot_gazebo/laser_to_range_node.py
  herminebot_gazebo/frame_switcher_node.py
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

# make worlds and sdf files
# Create output folders
file(MAKE_DIRECTORY 
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/worlds
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/herminebot_diff
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/herminebot_omni
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config
)

set(SOURCES
    worlds/world.erb
    worlds/world.erb
    worlds/world.erb
    worlds/world.erb
    models/herminebot_diff/herminebot.erb
    models/herminebot_omni/herminebot.erb
    config/ros_gz_config_herminebot_diff.erb
    config/ros_gz_config_herminebot_omni.erb
)
set(FILES
    worlds/full.sdf
    worlds/empty.sdf
    worlds/blue.sdf
    worlds/yellow.sdf
    models/herminebot_diff/model.sdf
    models/herminebot_omni/model.sdf
    config/ros_gz_config_herminebot_diff.yaml
    config/ros_gz_config_herminebot_omni.yaml
)
set(ARGS_VAL "" EMPTY=true USE_YELLOW=false USE_BLUE=false "" "")
foreach(SOURCE FILE ARGS IN ZIP_LISTS SOURCES FILES ARGS_VAL)
    set(SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE})
    set(OUTPUT_FILE ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${FILE})

    generate_from_erb()
    string(REPLACE "/" "_" RENAMED ${FILE})
    add_custom_target(generate_${RENAMED} ALL DEPENDS ${OUTPUT_FILE})
endforeach()


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
