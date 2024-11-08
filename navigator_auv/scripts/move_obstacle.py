#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def move_obstacle():
    rospy.init_node('move_obstacle', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    state = ModelState()
    state.model_name = 'moving_obs'
    state.pose.position.x = 5.0
    state.pose.position.y = 5.0
    state.pose.position.z = -85.0
    state.pose.orientation.x = 0.0
    state.pose.orientation.y = 0.0
    state.pose.orientation.z = 0.0
    state.pose.orientation.w = 1.0
    
    rate = rospy.Rate(10)  # 1 Hz
    while not rospy.is_shutdown():
        state.pose.position.x += 0.1
        set_state(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass
