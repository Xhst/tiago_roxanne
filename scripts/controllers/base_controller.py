#!/usr/bin/env python
import sys
import os
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from roxanne_rosjava_msgs.msg import TokenExecution, TokenExecutionFeedback
from tiago_roxanne.srv import Position

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils.pose import posestamped_from_xy_and_cardinal_point


class TiagoBaseController:

    def __init__(self):
        self.node_name = 'base_controller'

        self.use_roxanne = rospy.get_param('use_roxanne', True)


    def get_movebasegoal_from_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        return goal


    def move_to(self, pose, timeout = rospy.Duration.from_sec(300.0)):
        goal = self.get_movebasegoal_from_pose(pose)

        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout)

        return result


    def move_to_model(self, model):
        try:
            poseResponse = self.model_pose_service(model)
            self.move_to(poseResponse.pose)
        except rospy.ServiceException:
            rospy.logwarn('Cannot find model with name: '+ model)
        except NameError:
            rospy.logwarn('Model pose service is disabled.')

    
    def move_to_cmd(self, data):
        split = data.split(" ")

        pose_stamped = posestamped_from_xy_and_cardinal_point(
            float(split[0]),
            float(split[1]),
            split[2]
        )

        self.move_to(pose_stamped)


    def command_callback(self, message):
        msg_data = message.data

        print(msg_data)
        split = msg_data.split(" ", 1)
        
        command = split[0]
        data = split[1]

        print(command)
        if command == 'moveto':
            self.move_to_cmd(data)
        if command == 'movetomodel':
            self.move_to_model(data)

    
    def roxanne_execution_callback(self, execution):
        try:
            rospy.loginfo(execution)
            if execution.token.component == 'base':

                if execution.token.predicate == '_MovingTo' and len(execution.token.parameters) == 1:

                    response = self.position_service(str(execution.token.parameters[0]))
                    
                    pose_stamped = posestamped_from_xy_and_cardinal_point(
                        response.x,
                        response.y,
                        response.cardinal_point
                    )

                    result = self.move_to(pose_stamped)
                    
                    self.send_roxanne_feedback(int(not result), execution.tokenId)
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))


    def send_roxanne_feedback(self, code, id):
        feedback = TokenExecutionFeedback()
        feedback.tokenId = id
        feedback.code = code

        self.roxanne_publisher.publish(feedback)

    
    def connect_roxanne_nodes(self):
        if not bool(self.use_roxanne):
            return

        rospy.loginfo('Starting /roxanne/acting/feedback/base')
        self.roxanne_publisher = rospy.Publisher('/roxanne/acting/feedback/base', TokenExecutionFeedback, queue_size=1)
        rospy.loginfo('Subscribing to /roxanne/acting/dispatching/base')
        rospy.Subscriber('/roxanne/acting/dispatching/base', TokenExecution, self.roxanne_execution_callback)


    def connect_position_service(self):
        rospy.loginfo('Connecting to position_service.')
        self.position_service = rospy.ServiceProxy('/tiago_roxanne/position_service', Position)
        self.position_service.wait_for_service()
        rospy.loginfo('Succesfully connected.')


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)

        self.connect_roxanne_nodes()
        self.connect_position_service()

        rospy.loginfo("Connecting to move_base Action Server.")
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('Succesfully connected.')

        rospy.Subscriber('/tiago_roxanne/cmd/base', String, self.command_callback)

        rospy.spin()
        

if __name__ == '__main__':
    try:
        base_controller = TiagoBaseController()
        base_controller.start()
    except rospy.ROSInterruptException:
        pass