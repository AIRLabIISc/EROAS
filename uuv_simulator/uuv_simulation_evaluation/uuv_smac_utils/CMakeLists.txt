cmake_minimum_required(VERSION 2.8.3)
project(uuv_smac_utils)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  uuv_simulation_evaluation
  uuv_simulation_wrapper)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/smac_wrapper
                               scripts/run_smac
                               scripts/create_smac_config_files
                               scripts/evaluate_smac_best_results
                               scripts/smac
                               scripts/create_results_folder
                               scripts/generate_motion_primitives
                               scripts/sync_smac_files
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
