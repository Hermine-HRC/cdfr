set(python_tests
    test_flake8.py
    test_pep257.py
    test_laser_to_range.py
)
foreach(python_test ${python_tests})
ament_add_pytest_test(${python_test} ${python_test}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
endforeach()
