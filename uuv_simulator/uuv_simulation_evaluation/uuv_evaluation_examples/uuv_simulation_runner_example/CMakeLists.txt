cmake_minimum_required(VERSION 2.8.3)
project(uuv_simulation_runner_example)

find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        uuv_simulation_evaluation
        uuv_simulation_wrapper
        uuv_descriptions
)

catkin_install_python(PROGRAMS scripts/run_grid_search DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
