set(python_tests
    test_flake8.py
    test_pep257.py
    test_services_common_server.py
    test_utils.py
)
foreach(python_test ${python_tests})
ament_add_pytest_test(${python_test} ${python_test}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
endforeach()

ament_add_gtest(test_utils test_utils.cpp)
target_link_libraries(test_utils
    ${library_name}
)
