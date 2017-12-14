#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from giskard_msgs.msg import ControllerListGoal, Controller, ControllerListAction
from sensor_msgs.msg import JointState

from std_msgs.msg import String
from tf.transformations import quaternion_about_axis, quaternion_from_euler


class MoveArm(object):
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.client = SimpleActionClient('/qp_controller/command', ControllerListAction)
        rospy.loginfo('connecting to giskard')
        self.client.wait_for_server()
        rospy.loginfo('connected to giskard')
        self.tip = 'gripper_tool_frame'
        self.root = 'base_link'
        self.joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'elbow_joint',
                            'wrist_1_joint',
                            'wrist_2_joint',
                            'wrist_3_joint', ]

    def send_cart_goal(self, goal_pose):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER

            # translation
            controller = Controller()
            controller.type = Controller.TRANSLATION_3D
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_pose = goal_pose

            controller.p_gain = 3
            controller.enable_error_threshold = True
            controller.threshold_value = 0.05
            controller.weight = 1.0
            goal.controllers.append(controller)

            # rotation
            controller = Controller()
            controller.type = Controller.ROTATION_3D
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_pose = goal_pose

            controller.p_gain = 3
            controller.enable_error_threshold = True
            controller.threshold_value = 0.2
            controller.weight = 1.0
            goal.controllers.append(controller)

            self.client.send_goal(goal)
            result = self.client.wait_for_result(rospy.Duration(10))
            print('finished in 10s?: {}'.format(result))

    def relative_goal(self, position, orientation):
        p = PoseStamped()
        p.header.frame_id = self.tip
        p.pose.position = Point(*position)
        p.pose.orientation = Quaternion(*orientation)
        self.send_cart_goal(p)

    def send_joint_goal(self, joint_state):
        if self.enabled:
            goal = ControllerListGoal()
            goal.type = ControllerListGoal.STANDARD_CONTROLLER

            # translation
            controller = Controller()
            controller.type = Controller.JOINT
            controller.tip_link = self.tip
            controller.root_link = self.root

            controller.goal_state = joint_state

            controller.p_gain = 3
            controller.enable_error_threshold = False
            controller.threshold_value = 0.01
            controller.weight = 1.0
            goal.controllers.append(controller)

            self.client.send_goal(goal)
            result = self.client.wait_for_result(rospy.Duration(10))
            print('finished in 10s?: {}'.format(result))

    # Definition der Start Pose
    def go_to_start_cutting(self):
        print ("Approach Start Pose")
        goal_joint_state = JointState()
        goal_joint_state.name = self.joint_names
        goal_joint_state.position = [-(1.5708+0.7854),
                                     -1.668,
                                     -(1.5708+0.7854),
                                     -2.26,
                                     -1.5708,
                                     1.5708]
        self.send_joint_goal(goal_joint_state)
        print ("Start Pose Approached")

    # Definition der End Pose
    def go_to_end_cutting(self):
        print ("Approach End Pose")
        goal_joint_state = JointState()
        goal_joint_state.name = self.joint_names
        goal_joint_state.position = [-(1.5708+0.7854),
                                     -0.7854,
                                     -2.8,
                                     -0.1,
                                     -1.5708,
                                     1.5708]
        self.send_joint_goal(goal_joint_state)
        print ("End Pose Approached")

    # Einfache Schnittbewegung entlang der y-Achse (in Bezug auf gripper_tool_frame) bei gleicher Orientierung des Grippers
    def straight_cut(self):
        print ("Begin straight cut")
        goal = PoseStamped()
        goal.header.frame_id = 'gripper_tool_frame'
        goal.pose.position.y = -0.08
        goal.pose.orientation.w = 1.0
        self.send_cart_goal(goal)


if __name__ == '__main__':

    rospy.init_node('move_group_python_interface_test',
                    anonymous=True)
    test = MoveArm()

    print "Please make sure that your robot can move freely before proceeding!"
    inp = raw_input("Continue? y/n: ")[0]
    if (inp == 'y'):
        print ("Start")
        test.go_to_start_cutting() # Aufruf der Start-Pose
        test.straight_cut() # Aufruf der einfachen Schnittbewegung
        test.go_to_end_cutting()  # Aufruf der End-Pose
        print ("End")
    else:
        print ("Halting program")


