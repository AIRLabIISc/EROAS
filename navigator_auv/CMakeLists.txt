cmake_minimum_required(VERSION 2.8.3)
project(navigator_auv)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/sonar_heading_node.py
                               scripts/move_sonar.py
                               scripts/local_planner_heading.py
                               scripts/altitude_control.py
                               scripts/contour_heading.py
                               scripts/sonar_reconstruction.py
                               scripts/image_saver.py
                               scripts/euler_calculator.py
                               scripts/move_obstacle.py
                               scripts/cbf_implementation.py
                               scripts/just_cbf.py
                               scripts/obstacle_draw.py
                               scripts/only_gap.py
                               scripts/pose_plotter.py
                               scripts/spawner.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
