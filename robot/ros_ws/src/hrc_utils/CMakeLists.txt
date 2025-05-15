cmake_minimum_required(VERSION 3.8)
project(hrc_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

set(library_name ${PROJECT_NAME}_core)
set(dependencies
    geometry_msgs
)

include_directories(
  include
)

install(
    DIRECTORY hrc_utils
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  hrc_utils/services_common_server.py
  hrc_utils/external_parameter_interface.py
  hrc_utils/utils.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(${library_name} SHARED src/utils.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
