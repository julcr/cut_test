#!/usr/bin/env python
from __future__ import division
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
from std_srvs.srv import Trigger, TriggerRequest
from tf.transformations import quaternion_about_axis, quaternion_from_euler


class MoveArm(object):
    # Variablen die den Schnitt unmittelbar beeinflussen
    offset_tip = 0.103  # Offset in cm (Dicke des Schneidbretts + Abstand zw. Fingerspitze und Klingenunterseite)
    blade_len = 0.05 # Laenge der Klinge
    ft_limit = 50 # Kraft Grenzwert
    ft_threshold = 15 # Kraft Schwellwert
    step_down = 0.01 # Standard Schnitttiefe

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
        #
        self.ft_sub = rospy.Subscriber("/kms40/wrench_zeroed", WrenchStamped, self.ft_callback)
        self.ft_list = []
        # Service wartet auf ein Objekt vom Typ Trigger
        self.reset_ft = rospy.ServiceProxy("/ft_cleaner/update_offset",Trigger)
        rospy.sleep(1)
        self.reset_ft.call(TriggerRequest()) #Trigger Objekt

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
        test123.send_cart_goal(p)

    def distance2table(self):
        # Abfrage der Position des Frames 'gripper_tool_frame' in Bezug auf 'arm_mounting_plate'.
        # Das Frame 'arm_mounting_plate' entspricht dabei der Tischoberkante.
        trans = self.tfBuffer.lookup_transform('arm_mounting_plate', self.tip,rospy.Time())

        # Kalkulation des Abstandes von Klingen-Unterseite zum Schneidebrett
        distance2table = trans.transform.translation.z - self.offset_tip
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
    # Definition der Ausgangsposition-Position
    def go_to_home(self):
        print ("Approach Start Pose")
        goal_joint_state = JointState()
        goal_joint_state.name = self.joint_names
        goal_joint_state.position = [-2.417572323475973,
                                     -1.530511204396383,
                                     -1.6327641646014612,
                                     -1.5507991949664515,
                                     1.5708668231964111,
                                     1.509663701057434]
        self.send_joint_goal(goal_joint_state)
        print ("Start Pose Approached")

    # Einfache Schnittbewegung entlang der y-Achse (in Bezug auf gripper_tool_frame) bei gleicher Orientierung des Grippers
    # def straight_cut(self):
    #     d2t = test.distance2table()
    #     test.relative_goal([0,-d2t,0],[0,0,0,1])
    #

    # Hackende Bewegung
    # def straight_chop(self):
    #     max_step = 6
    #     for i in range(max_step):
    #         test.relative_goal([0,-0.02,0],[0,0,0,1])
    #         test.relative_goal([0,0.01,0], [0, 0, 0, 1])
    #
    # Saegende Bewegung
    # def saw(self):
    #     max_step = 6
    #     for i in range(max_step):
    #         test.relative_goal([0, -0.005, 0.05], [0, 0, 0, 1])
    #         test.relative_goal([0, -0.005, -0.05], [0, 0, 0, 1])
    #

    # Einfache rollende Schnittbewegung
    # def roll_simple(self):
    #     q = quaternion_from_euler(0, 0.3, 0, 'ryxz')
    #     test.relative_goal([0,0,0],q,translation_weight=100) # Erhohung der Gewichtung der Translation, damit die Spitze genauer in Position bleibt
    #     test.move_tip_in_amp(0, 0, -0.08)
    #     q_1 = quaternion_from_euler(0, -0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0],q_1,translation_weight=100)
    #
    # Erweiterte rollende Schnittbewegung
    # def roll_advanced(self):
    #     q = quaternion_from_euler(0, 0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0], q, translation_weight=100)
    #     test.move_tip_in_amp(0, 0, -0.08)
    #     test.move_tip_in_amp(-0.05, 0, 0)
    #     q_1 = quaternion_from_euler(0, -0.3, 0, 'ryxz')
    #     test.relative_goal([0, 0, 0], q_1, translation_weight=100)
    #
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


    def master_cut(self):
        # Abfrage des aktuellen Abstands von Klinge zu Schneidebrett
        d2t = test123.distance2table()
        while d2t > 0:
            # Aufruf der Funktion, die die Bewegung berechnet.
            down, side, final = test123.calc_move()
            # Ausfuehrung der Bewegung
            if side == 0:
                test123.move_tip_in_amp(0, 0, -down)
            # Wenn F/T-Wert den Grenzwert ueberschreitet, kommt eine Bewegung in x Richtung dazu.
            # Dabei wird zunaechst die Klinge ohne Bewegung zurueck gefahren, um von der vollen Klingenlaenge
            # zu profitieren. Anschliessend erfolgt eine diagonale Schnittbewegung ueber die gesamte Klingenlaenge.
            # Abschliessend eine weitere diagonale Bewegung, um wieder in die Ausgangsposition (x-Achse) zu gelangen.
            else:
                test123.move_tip_in_amp(-side, 0, -(1/4) * down)
                test123.move_tip_in_amp(2 * side, 0, -(2 / 4) * down)
                test123.move_tip_in_amp(-side, 0, -(1 / 4) * down)

         # Wenn die letze Bewegung ausgefuehrte wurde (also Final == True von calc_move() zurueckgegeben wird),
         # wird die Funktion beendet.
            if final == True:
                test123.move_tip_in_amp(self.blade_len/2, 0, 0)
                test123.move_tip_in_amp(-self.blade_len, 0, 0)
                test123.move_tip_in_amp(self.blade_len/2, 0, 0)
                test123.move_tip_in_amp(0, 0.05, 0.005)
                return

    # Funktion um den maximalen F/T waehrend der Bewegung auszulesen.
    def max_ft(self):
        # Abfrage des max. F/T aus der Liste der F/T Werte
        ft_max = max(self.ft_list)
        # Nach dem der Wert zwischengespeichert worden ist, wird die Liste fuer den naechsten Durchlauf zurueckgesetzt
        self.ft_list = []
        print("Max. FT: %s" % ft_max)
        return(ft_max)

    # Funktion um die Schnittbewegung zu berechnen
    def calc_move(self):
        """
        Return of three values necessary for cut-move execution.
        :return: 1.value:cutting depth; 2.value: lateral move; 3.value: final cut
        :type: (float,float,bool)
        """

        # Init
        final = False

        # Abfrage des maximalen F/T-Werts aus der letzten Bewegung
        cur_ft = test123.max_ft()

        # Abfrage des aktuellen Abstands von Klingenunterseite zu Schneidebrett
        d2t = test123.distance2table()
        print("Distance to Table %s" % d2t)
        print("Current FT %s" %cur_ft)

        # Wenn der Kraftwert den Schwellwert unterschreitet, wird nur entlang der z-Achse geschnitten
        if cur_ft < self.ft_threshold:
            down = self.step_down
            side = 0
            # Wenn die Schritttiefe kleiner als der Abstand zur Oberflaeche ist,
            # wird die Schritttiefe der naechsten Bewegung auf die verbleibende Distanz gesetzt,
            # um Kollisionen mit dem Tisch zu vermeiden.
            if d2t <= down:
                down = d2t
                final = True
        else:
            # Berechnung der Bewegung, wenn der Kraftschwellwert ueberschritten wird.

            # Je hoeher der gemessene Kraftwert, desto geringer die Schnitttiefe.
            # Die maximale berechnete Schnitttiefe entspricht der Standardschnitttiefe,
            # wenn die Kraft dem Schwellwert entspricht.
            down = (1-(cur_ft)/self.ft_limit)*(self.step_down)

            # Setzen einer Mindestschnitttiefe
            if down < 0.001:
                down = 0.001

            # Wenn die berechnete Schrittweite kleiner als der Abstand zur Oberflaeche ist,
            # wird die Schrittweite der naechsten Bewegung entsprechend angepasst,
            # um Kollisionen mit dem Tisch zu vermeiden.
            if d2t <= down:
                down = d2t
                final = True # Letzte Bewegung

            # Die seitliche Bewegung entspricht der Haelfte der Klingenlaenge.
            side = self.blade_len / 2

        print("Side %s" % side)
        print("Down %s" % down)
        return (down,side,final)

    def ft_callback(self,data):
        """
        Callback for force torque sensor
        :param data: sensor data
        :type: WrenchStamped
        """
        # Abbruch der Bewegung, wenn gemessene Kraft >60 Nm, um Sicherheitsabschaltung des Arms zuvorzukommen.
        if abs(data.wrench.force.z) > 60:
            print("Stop")
            self.client.cancel_all_goals()

        # Auslesen der Kraft in z-Richtung und hinzufuegen zu einer Liste.
        ft = abs(data.wrench.force.z)
        self.ft_list.append(ft)

    def align(self):
        """
        Alignment of the knife
        """
        # Ausrichtung des Messers
        q = quaternion_from_euler(0, -0.15, 0.02, 'ryxz')
        test123.relative_goal([0, 0, 0], q)



if __name__ == '__main__':

    rospy.init_node('move_group_python_interface_test',
                    anonymous=True)

    test123 = MoveArm()

    test123.go_to_home() # Aufruf der Ausgangs-Pose
    test123.align() # Ausrichtung der Klinge
    test123.move_tip_in_amp(0, 0,-0.03) # Positionierung ueber Schnittobjekt
    test123.master_cut()# Aufruf der Schnittbewegung
    test123.go_to_home()  # Aufruf der Ausgangs-Pose

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
