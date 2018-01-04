#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL
import tf2_ros
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from giskard_msgs.msg import ControllerListGoal, Controller, ControllerListAction
from sensor_msgs.msg import JointState
from numpy import pi

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

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def send_cart_goal(self, goal_pose,translation_weight=1,rotation_weight=1):
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
            controller.weight = translation_weight
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
            controller.weight = rotation_weight
            goal.controllers.append(controller)

            self.client.send_goal(goal)
            result = self.client.wait_for_result(rospy.Duration(10))
            print('finished in 10s?: {}'.format(result))

    def relative_goal(self, position, orientation,translation_weight=1,rotation_weight=1):
        p = PoseStamped()
        p.header.frame_id = self.tip
        p.pose.position = Point(*position)
        p.pose.orientation = Quaternion(*orientation)
        self.send_cart_goal(p,translation_weight,rotation_weight)

    def move_tip_in_amp(self, x, y, z):  # Bewegung des Gripper Tool Frame in Bezug auf den Frame 'arm_mounting_plate' (amp)
        trans = self.tfBuffer.lookup_transform('arm_mounting_plate', self.tip,
                                               rospy.Time())  # Ermittelung der Position von Gripper Tool Frame in Bezug auf 'arm_mounting_plate'-Frame
        p = PoseStamped()
        p.header.frame_id = 'arm_mounting_plate'
        p.pose.position = trans.transform.translation  # Uebernahme der soeben ermittelten Werte
        p.pose.position.x += x  # Addition der Bewegung in x-Richtung
        p.pose.position.y += y
        p.pose.position.z += z
        p.pose.orientation = trans.transform.rotation
        test.send_cart_goal(p)

    def distance2table(self):
        trans = self.tfBuffer.lookup_transform('arm_mounting_plate', self.tip,rospy.Time())
        offset_tip = 0.04 #Offset
        distance2table = trans.transform.translation.z - offset_tip
        print(distance2table)
        return distance2table

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
    # def straight_cut(self):
    #     distance2table = test.distance2table()
    #     test.relative_goal([0,-distance2table,0],[0,0,0,1])

    # def straight_chop(self):
    #     max_step = 6
    #     for i in range(max_step):
    #         test.relative_goal([0,-0.02,0],[0,0,0,1])
    #         test.relative_goal([0,0.01,0], [0, 0, 0, 1])

    # def saw(self):
    #     max_step = 6
    #     for i in range(max_step):
    #         test.relative_goal([0, -0.005, 0.05], [0, 0, 0, 1])
    #         test.relative_goal([0, -0.005, -0.05], [0, 0, 0, 1])
    #
    # def roll_simple(self):
    #     q = quaternion_from_euler(0, 0.3, 0, 'ryxz')
    #     test.relative_goal([0,0,0],q,translation_weight=100) # Erhohung der Gewichtung der Translation, damit die Spitze genauer in Position bleibt
    #     test.move_tip_in_amp(0, 0, -0.08)
    #     q_1 = quaternion_from_euler(0, -0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0],q_1,translation_weight=100)
    #
    # def roll_advanced(self):
    #     q = quaternion_from_euler(0, 0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0], q, translation_weight=100)
    #     test.move_tip_in_amp(0, 0, -0.08)
    #     test.move_tip_in_amp(-0.05, 0, 0)
    #     q_1 = quaternion_from_euler(0, -0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0], q_1, translation_weight=100)
    #
    # def cross_cut(self):
    #     max_step = 5
    #     for i in range(max_step):
    #         q = quaternion_from_euler(0, 0.1, 0, 'ryxz')
    #         test.relative_goal([0, 0, 0], q, translation_weight=100)
    #         test.relative_goal([0, -0.01, 0.05], [0, 0, 0, 1])
    #         q_1 = quaternion_from_euler(0, -0.1, 0, 'ryxz')
    #         test.relative_goal([0, 0, 0], q_1,translation_weight=100)
    #         test.relative_goal([0, 0, -0.05], [0, 0, 0, 1])

    def straight_cut2(self):
        d2t = test.distance2table()
        while d2t > 0.001:
           d2t = test.distance2table()
           print("Distance to Table %s" % d2t)
           test.move_tip_in_amp(0, 0, -0.01)
           d2t = test.distance2table()
           if d2t <= 0.01:
                print("FINALE")
                test.move_tip_in_amp(0, 0, -d2t)

    def cross_cut2(self):
        max_step = 5
        for i in range(max_step):
            q = quaternion_from_euler(0, 0.1, 0, 'ryxz')
            test.relative_goal([0, 0, 0], q, translation_weight=100)
            test.relative_goal([0, -0.01, 0.05], [0, 0, 0, 1])
            q_1 = quaternion_from_euler(0, -0.1, 0, 'ryxz')
            test.relative_goal([0, 0, 0], q_1,translation_weight=100)
            test.relative_goal([0, 0, -0.05], [0, 0, 0, 1])


if __name__ == '__main__':

    rospy.init_node('move_group_python_interface_test',
                    anonymous=True)
    test = MoveArm()

    # test.relative_goal([0.,0,0.05],[0,0,0,1])

    # print "Please make sure that your robot can move freely before proceeding!"
    # inp = raw_input("Continue? y/n: ")[0]
    # # if (inp == 'y'):
    #     print ("Start")

    test.go_to_start_cutting() # Aufruf der Start-Pose
    test.saw2()# Aufruf der einfachen Schnittbewegung
    rospy.sleep(2)
    test.go_to_start_cutting()  # Aufruf der Start-Pose
    # test.go_to_end_cutting()  # Aufruf der End-Pose


    #     print ("End")
    # else:
    #     print ("Halting program")


