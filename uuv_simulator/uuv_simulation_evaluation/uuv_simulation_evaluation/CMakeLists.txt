cmake_minimum_required(VERSION 2.8.3)
project(uuv_simulation_evaluation)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/evaluate_bag
                               scripts/run_best_worst_comparison
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  find_package(rosunit)

  foreach(UNIT_TESTS
    test/test_constraint.py
    test/test_cost_function.py
    test/test_evaluation_trajectory.py
    test/test_kpis.py
    catkin_add_nosetests(${UNIT_TESTS}))
  endforeach()

endif()
