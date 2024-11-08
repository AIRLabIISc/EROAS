cmake_minimum_required(VERSION 2.8.3)
project(uuv_batch_run_example)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
