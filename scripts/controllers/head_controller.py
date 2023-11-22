#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

class TiagoHeadController:

    def __init__(self):
        self.node_name = 'head_controller'
        self.publisher = None


    def move_to(self, horizontal_position, vertical_position, duration):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [horizontal_position, vertical_position]
        point.time_from_start = rospy.Duration.from_sec(duration)

        joint_trajectory.points.append(point)
        self.publisher.publish(joint_trajectory)


    def callback(self, message):
        data = message.data
        print(data)
        split = data.split(" ")

        horizontal_position = float(split[1])
        vertical_position = float(split[2])
        duration = int(split[3])

        self.move_to(horizontal_position, vertical_position, duration)


    def connect_roxanne_nodes(self):
        if not bool(self.use_roxanne):
            return

        rospy.loginfo('Starting /roxanne/acting/feedback/base')
        self.roxanne_publisher = rospy.Publisher('/roxanne/acting/feedback/base', TokenExecutionFeedback, queue_size=1)
        rospy.loginfo('Subscribing to /roxanne/acting/dispatching/base')
        rospy.Subscriber('/roxanne/acting/dispatching/base', TokenExecution, self.roxanne_execution_callback)
        

    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        self.publisher = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=3)
        rospy.Subscriber('head_cmd', String, self.callback)
        rospy.spin()    
        

if __name__ == '__main__':
    try:
        head_controller = TiagoHeadController()
        head_controller.start()
    except rospy.ROSInterruptException:
        pass