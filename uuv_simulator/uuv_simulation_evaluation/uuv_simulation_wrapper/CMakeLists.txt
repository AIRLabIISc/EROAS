cmake_minimum_required(VERSION 2.8.3)
project(uuv_simulation_wrapper)

find_package(catkin REQUIRED COMPONENTS
        rospy
        rostopic
        rosgraph
        nav_msgs
        geometry_msgs
        gazebo_msgs
        visualization_msgs
        tf)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/set_simulation_timer
                               scripts/unpause_simulation
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  foreach(T
      test/test_run_task.test)
     add_rostest(${T})
  endforeach()
endif()
