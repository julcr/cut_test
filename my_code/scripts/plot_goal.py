import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
import pylab as plt
import numpy as np

def cb(data):
    print(data)
    positions = []
    velocities = []
    for i, point in enumerate(data.goal.trajectory.points):
        positions.append(point.positions)
        velocities.append(point.velocities)
    positions = np.array(positions)
    velocities = np.array(velocities)
    plt.plot(positions)
    plt.show()
    plt.plot(velocities)
    plt.show()


if __name__ == '__main__':
    rospy.init_node('goal_plotter')
    goal_sub = rospy.Subscriber('/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, cb, queue_size=10)
    rospy.spin()
