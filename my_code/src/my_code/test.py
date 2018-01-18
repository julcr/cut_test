#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
## END_SUB_TUTORIAL
import tf2_ros
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Point, WrenchStamped
from giskard_msgs.msg import ControllerListGoal, Controller, ControllerListAction
from sensor_msgs.msg import JointState
from numpy import pi
import numpy as np
from math import sqrt
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


        # This declares the subscription to the "/kms40/wrench_zeroed" topic which is of type WrenchStamped.
        # When new messages are recieved, ft_callback is invoked.
        self.ft_sub = rospy.Subscriber("/kms40/wrench_zeroed", WrenchStamped, self.ft_callback)
        self.ft_list = []


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

    def move_tip_in_amp(self, x, y, z, translation_weight=1,rotation_weight=1):  # Bewegung des Gripper Tool Frame in Bezug auf den Frame 'arm_mounting_plate' (amp)
        trans = self.tfBuffer.lookup_transform('arm_mounting_plate', self.tip,
                                               rospy.Time())  # Ermittelung der Position von Gripper Tool Frame in Bezug auf 'arm_mounting_plate'-Frame
        p = PoseStamped()
        p.header.frame_id = 'arm_mounting_plate'
        p.pose.position = trans.transform.translation  # Uebernahme der soeben ermittelten Werte
        p.pose.position.x += x  # Addition der Bewegung in x-Richtung
        p.pose.position.y += y
        p.pose.position.z += z
        p.pose.orientation = trans.transform.rotation
        test.send_cart_goal(p,translation_weight,rotation_weight)

    def distance2table(self):
        trans = self.tfBuffer.lookup_transform('arm_mounting_plate', self.tip,rospy.Time())
        offset_tip = 0.01 #Offset in cm
        distance2table = trans.transform.translation.z - offset_tip
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
    # def go_to_start_cutting(self):
    #     print ("Approach Start Pose")
    #     goal_joint_state = JointState()
    #     goal_joint_state.name = self.joint_names
    #     goal_joint_state.position = [-(1.5708+0.7854),
    #                                  -1.668,
    #                                  -(1.5708+0.7854),
    #                                  -2.26,
    #                                  -1.5708,
    #                                  1.5708]
    #     self.send_joint_goal(goal_joint_state)
    #     print ("Start Pose Approached")

    def go_to_home(self):
        print ("Approach Start Pose")
        goal_joint_state = JointState()
        goal_joint_state.name = self.joint_names
        goal_joint_state.position = [-2.3561944901923,
                                     -1.7453292519943,
                                     -1.3962634015955,
                                     -1.5707963267949,
                                     1.5707963267949,
                                     1.5707963267949]
        self.send_joint_goal(goal_joint_state)
        print ("Start Pose Approached")

    # # Definition der End Pose
    # def go_to_end_cutting(self):
    #     print ("Approach End Pose")
    #     goal_joint_state = JointState()
    #     goal_joint_state.name = self.joint_names
    #     goal_joint_state.position = [-(1.5708+0.7854),
    #                                  -0.7854,
    #                                  -2.8,
    #                                  -0.1,
    #                                  -1.5708,
    #                                  1.5708]
    #     self.send_joint_goal(goal_joint_state)
    #     print ("End Pose Approached")



    #Einfache Schnittbewegung entlang der y-Achse (in Bezug auf gripper_tool_frame) bei gleicher Orientierung des Grippers
    # def straight_cut(self):
    #     d2t = test.distance2table()
    #     test.relative_goal([0,-d2t,0],[0,0,0,1])
    #
    # def straight_chop(self):
    #     max_step = 6
    #     for i in range(max_step):
    #         test.relative_goal([0,-0.02,0],[0,0,0,1])
    #         test.relative_goal([0,0.01,0], [0, 0, 0, 1])
    #
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

    #Einfache Schnittbewegung entlang der y-Achse (in Bezug auf gripper_tool_frame) bei gleicher Orientierung des Grippers
    # def straight_cut(self):
    #     d2t = test.distance2table()
    #     test.relative_goal([0,-d2t,0],[0,0,0,1])
    #
    # def straight_cut3(self):
    #     d2t = test.distance2table()
    #     while d2t > 0:
    #        d2t = test.distance2table()
    #        #select down
    #        down = 0.01
    #        print("Distance to Table %s" % d2t)
    #        test.move_tip_in_amp(0, 0, -down)
    #        d2t = test.distance2table()
    #        if d2t <= down:
    #             test.move_tip_in_amp(0, 0, -d2t)
    #             print("Distance to Table %s" % d2t)
    #             return


    # def cross_cut2(self):
    #     d2t = test.distance2table()
    #     max_step = 5
    #     for i in range(max_step):
    #         q = quaternion_from_euler(0, 0.1, 0, 'ryxz')
    #         test.relative_goal([0, 0, 0], q, translation_weight=100)
    #         test.relative_goal([0, -0.01, 0.05], [0, 0, 0, 1])
    #         q_1 = quaternion_from_euler(0, -0.1, 0, 'ryxz')
    #         test.relative_goal([0, 0, 0], q_1,translation_weight=100)
    #         test.relative_goal([0, 0, -0.05], [0, 0, 0, 1])

    def master_cut(self):
        d2t = test.distance2table()
        while d2t > 0:
            # call calculation function
            down, side, final = test.calc_move()
            # exec move(s). second move primarily to return to initial position.
            if side == 0:
                test.move_tip_in_amp(0, 0, -down)
            else:
                test.move_tip_in_amp(-side, 0, 0)
                test.move_tip_in_amp(2*side, 0, -down)
                test.move_tip_in_amp(-side, 0, 0)

         # If blade has made it's final move (as calculated in calc_move), function is exited
            if final == True:
                d2t = test.distance2table()
                print("Final Distance to Table %s" % d2t)
                return

    def max_ft(self):
        # last_meas = np.array(self.ft_list)

        # Get max. FT value from list and empty list
        ft_max = max(self.ft_list)
        self.ft_list = []
        # ft_max = last_meas.mean()
        print("Max. FT: %s" % ft_max)
        return(ft_max)

    def calc_move(self):
        # Init
        final = False # init
        blade_length = 0.10 #Lenght of Blade
        ft_threshold = 3 # Threshold for ft
        ft_limit = 15 # Set maximum ft

        # Get values for computation
        # cur_ft = -self.ft # Get current ft
        cur_ft = test.max_ft()
        # cur_ft = 2
        d2t = test.distance2table() # Get current distance to table
        print("Distance to Table %s" % d2t)
        print("Current FT %s" %cur_ft)
        # If FT value is below a threshold, no further computation needed
        if cur_ft <= ft_threshold:
            down = 0.01
            side = 0
            # If current step size is smaller than the distance to the table,
            # the next step is set to the remaining distance to avoid collision
            # and final is set to true.
            if d2t <= down:
                down = d2t
                final = True
        else:
            # Calculation of movement, if the value from the ft-sensor exceeds the threshold
            # the higher the ft, the lower the step size on z-axis
            # movement on x axis is not taken into calculation
            down = (1-(float(cur_ft)/ft_limit))*0.01
            if cur_ft >= ft_limit:
                down = 0.001
            if d2t <= down:
                down = d2t
                final = True
            side = float(blade_length)/2

        print("Side %s" % side)
        print("Down %s" % down)
        return (down,side,final)

    def ft_callback(self,data):
        # calculate length of ft euclidean vector
        # ft = abs(data.wrench.force.x*data.wrench.force.y*data.wrench.force.z)
        # self.ft = sqrt(ft)
        ft = data.wrench.force.z
        # self.ft = ft
        # Add elements to list
        self.ft_list.append(-ft)


if __name__ == '__main__':

    rospy.init_node('move_group_python_interface_test',
                    anonymous=True)

    test = MoveArm()


    # print "Please make sure that your robot can move freely before proceeding!"
    # inp = raw_input("Continue? y/n: ")[0]
    # # if (inp == 'y'):
    #     print ("Start")



    test.go_to_home() # Aufruf der Start-Pose
    test.move_tip_in_amp(0,0,-0.10)
    test.master_cut()# Aufruf der Schnittbewegung
    rospy.sleep(2)
    test.go_to_home()  # Aufruf der Start-Pose



    #     print ("End")
    # else:
    #     print ("Halting program")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
