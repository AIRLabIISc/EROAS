source devel/setup.bash 
rostopic pub -r 10 /rexrov2/cmd_vel geometry_msgs/Twist "linear:
rosrun uuv_control_utils contour_heading.py 
roslaunch rexrov2_gazebo start_demo_pid_controller.launch teleop_on:=true
pkill gzclient && pkill gzserver
rostopic echo /rexrov2/blueview_p900/sonar_image_raw