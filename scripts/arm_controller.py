#!/usr/bin/python3
import time

import rospy

from kortex_driver.srv import *
from kortex_driver.msg import *


class ArmController:
    def __init__(self):
        try:
            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom " +
                          "and the president of gripper is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_subscriber = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.action_topic_callback)
            self.last_action_notification_type = None

            # Wait for the driver to be initialised
            while not rospy.has_param("/" + self.robot_name + "/is_initialized"):
                time.sleep(0.1)

            # Init the services
            clear_faults_fullname = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_fullname)
            self.clear_faults = rospy.ServiceProxy(clear_faults_fullname, Base_ClearFaults)

            read_action_fullname = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_fullname)
            self.read_action = rospy.ServiceProxy(read_action_fullname, ReadAction)

            execute_action_fullname = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_fullname)
            self.execute_action = rospy.ServiceProxy(execute_action_fullname, ExecuteAction)

            set_cartesian_reference_frame_fullname = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_fullname)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_fullname, SetCartesianReferenceFrame)

            play_cartesian_trajectory_fullname = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_fullname)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_fullname, PlayCartesianTrajectory)

            play_joint_trajectory_fullname = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_fullname)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_fullname, PlayJointTrajectory)

            send_gripper_command_fullname = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_fullname)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_fullname, SendGripperCommand)

            activate_publishing_of_action_notification_fullname = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_fullname)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_fullname,
                                                                                 OnNotificationActionTopic)
        except Exception as e:
            rospy.logerr("initialize robot arm controller failed", e)
            raise e
        else:
            rospy.loginfo("initialize robot arm controller successfully")

    def action_topic_callback(self, notification):
        self.last_action_notification_type = notification.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notification_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notification_type == ActionEvent.ACTION_ABORT:
                rospy.logerr("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def reset_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call ClearFaults", e)
            raise e
        else:
            rospy.loginfo("Cleared the faults successfully")

    def home(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notification_type = None
        request = ReadActionRequest()
        request.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            response = self.read_action(request)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call ReadAction", e)
            raise e
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            request = ExecuteActionRequest()
            request.input = response.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(request)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call ExecuteAction", e)
                raise e
            else:
                if not self.wait_for_action_end_or_abort():
                    raise Exception("home robot arm aborted")


def main():
    rospy.init_node("arm_controller")
    rospy.delete_param("is_initialized")
    rospy.loginfo("initialize robot arm controller")
    arm_controller = ArmController()
    rospy.loginfo("reset faults of the robot arm")
    arm_controller.reset_faults()
    rospy.loginfo("move robot arm to home position")
    arm_controller.home()
    rospy.set_param("is_initialized", True)


if __name__ == '__main__':
    main()
