#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from roxanne_rosjava_msgs.msg import TokenExecution, TokenExecutionFeedback
from std_msgs.msg import String


class TiagoTorsoController:

    def __init__(self):
        self.node_name = 'torso_controller'
        
        self.use_roxanne = rospy.get_param('use_roxanne', True)
        

    def move_to(self, position, duration):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration.from_sec(duration)

        joint_trajectory.points.append(point)
        self.publisher.publish(joint_trajectory)  


    def command_callback(self, message):
        data = message.data

        cmd = data.split(" ", 1)
        split = cmd[1].split(" ")

        position = float(split[0])
        duration = int(split[1])

        self.move_to(position, duration)


    def roxanne_execution_callback(self, execution):
        rospy.loginfo(execution)
        if execution.token.component == 'torso':

            if execution.token.predicate == 'Moving' and len(execution.token.parameters) == 1:

                position = float(execution.token.parameters[0]) / 35.0
                duration = 5.0

                self.move_to(position, duration)

                rospy.sleep(duration)
                self.send_roxanne_feedback(0, execution.tokenId)


    def send_roxanne_feedback(self, code, id):
        feedback = TokenExecutionFeedback()
        feedback.tokenId = id
        feedback.code = code

        self.roxanne_publisher.publish(feedback)
        

    def connect_roxanne_nodes(self):
        if not bool(self.use_roxanne):
            return

        rospy.loginfo('Starting /roxanne/acting/feedback/torso')
        self.roxanne_publisher = rospy.Publisher('/roxanne/acting/feedback/torso', TokenExecutionFeedback, queue_size=1)
        rospy.loginfo('Subscribing to /roxanne/acting/dispatching/torso')
        rospy.Subscriber('/roxanne/acting/dispatching/torso', TokenExecution, self.roxanne_execution_callback)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        self.publisher = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=3)
        rospy.Subscriber('/tiago_roxanne/cmd/torso', String, self.command_callback)

        self.connect_roxanne_nodes()

        rospy.spin()  
        

if __name__ == '__main__':
    try:
        torso_controller = TiagoTorsoController()
        torso_controller.start()
    except rospy.ROSInterruptException:
        pass