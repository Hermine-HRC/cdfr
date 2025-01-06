cmake_minimum_required(VERSION 3.8)
project(herminebot_description)

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
    DIRECTORY launch models config herminebot_description
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  herminebot_description/services_common_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    set(python_tests
        test/test_flake8.py
        test/test_pep257.py
    )
    foreach(python_test ${python_tests})
        string(REPLACE "/" "_" python_test_name ${python_test})
        ament_add_pytest_test(${python_test_name} ${python_test}
            APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
            TIMEOUT 60
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    endforeach()
endif()

ament_package()
