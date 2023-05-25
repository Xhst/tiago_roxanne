#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class TiagoHeadController:

    def __init__(self):
        self.node_name = 'torso_controller'
        self.publisher = None


    def callback(self, message):
        data = message.data

        split = data.split(" ")

        position = float(split[0])
        duration = int(split[1])

        self.move_to(position, duration)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        self.publisher = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=3)
        rospy.Subscriber('torso_cmd', String, self.callback)
        rospy.spin()


    def move_to(self, position, duration):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)

        joint_trajectory.points.append(point)
        self.publisher.publish(joint_trajectory)    
        

if __name__ == '__main__':
    try:
        head_controller = TiagoHeadController()
        head_controller.start()
    except rospy.ROSInterruptException:
        pass