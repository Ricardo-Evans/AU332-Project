#!/usr/bin/python3

import rospy
import arm_controller
from std_srvs.srv import Empty

if __name__ == '__main__':
    # Unpause the physics
    rospy.init_node("initialize")
    rospy.delete_param("is_initialized")
    rospy.loginfo("Unpause Gazebo...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
    rospy.loginfo("Unpause Gazebo successfully.")
    rospy.loginfo("initialize robot")
    arm_controller = arm_controller.ArmController()
    rospy.loginfo("send the robot to home")
    arm_controller.home()
    rospy.set_param("is_initialized", True)
    rospy.loginfo("robot initialized successfully")
