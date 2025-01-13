set(python_tests
  test_uncrustify.py
)
foreach(python_test ${python_tests})
  ament_add_pytest_test(${python_test} ${python_test}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endforeach()

ament_add_gtest(test_action_move_elevator test_move_elevator_action.cpp)
target_link_libraries(test_action_move_elevator hrc_move_elevator_action_bt_node)
ament_target_dependencies(test_action_move_elevator ${dependencies})

ament_add_gtest(test_manage_map_service test_manage_map_service.cpp)
target_link_libraries(test_manage_map_service hrc_manage_map_service_bt_node)
ament_target_dependencies(test_manage_map_service ${dependencies})

ament_add_gtest(test_move_elevator_behavior test_move_elevator_behavior.cpp)
target_link_libraries(test_move_elevator_behavior hrc_move_elevator_behavior)
ament_target_dependencies(test_move_elevator_behavior ${dependencies})
