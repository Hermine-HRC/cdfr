ament_add_gtest(keepout_filter_test keepout_filter_test.cpp)
target_link_libraries(keepout_filter_test
    costmap_filters
)

ament_add_gtest(obstacle_layer_test obstacle_layer_test.cpp)
target_link_libraries(obstacle_layer_test
    costmap_layers
)

ament_add_gtest(map_modifier_test test_map_modifier.cpp)
target_link_libraries(map_modifier_test
    map_modifier_lib
)

ament_add_gtest(regulated_rotation_controller_test test_RRC.cpp)
target_link_libraries(regulated_rotation_controller_test
    hrc_regulated_rotation_controller
)

set(python_tests
    test_uncrustify.py
)
foreach(python_test ${python_tests})
    string(REPLACE "/" "_" python_test_name ${python_test})
    ament_add_pytest_test(${python_test_name} ${python_test}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endforeach()
