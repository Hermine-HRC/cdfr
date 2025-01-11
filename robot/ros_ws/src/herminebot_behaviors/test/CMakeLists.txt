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
