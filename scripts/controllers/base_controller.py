#!/usr/bin/env python
import sys
import os
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tiago_hrc.srv import ModelPose


class TiagoBaseController:

    def __init__(self):
        self.node_name = 'base_controller'
        self.publisher = None

        self.use_model_pose_service = rospy.get_param('use_model_pose_service', False)


    def move_to(self, pose, timeout = rospy.Duration.from_sec(10.0)):
        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.client.send_goal(goal)
        self.client.wait_for_result(timeout)


    def move(self, linear, angular, duration):
        twist = Twist()
        twist.linear = linear
        twist.angular = angular

        now = rospy.Time.now()
        end_duration = now + rospy.Duration.from_sec(duration)

        rate = rospy.Rate(10)
        while rospy.Time.now() < end_duration:
            self.publisher.publish(twist)
            rate.sleep()


    def move_to_model(self, model):
        print("moveto")
        try:
            poseResponse = self.model_pose_service(model)
            self.move_to(poseResponse.pose)
        except rospy.ServiceException:
            rospy.logwarn('Cannot find model with name: '+ model)
        except NameError:
            rospy.logwarn('Model pose service is disabled.')


    def move_cmd(self, data):
        print("move")
        split = data.split(" ")
        
        linear = Vector3()
        linear.x = float(split[0])

        angular = Vector3()
        angular.z = float(split[1])

        duration = float(split[2])

        self.move(linear, angular, duration)


    def callback(self, message):
        msg_data = message.data

        print(msg_data)
        split = msg_data.split(" ", 1)
        
        command = split[0]
        data = split[1]

        if command == 'moveto':
            self.move_to_model(data)
        if command == 'base':
            self.move_cmd(data)


    def connect_model_pose_service(self):
        if not bool(self.use_model_pose_service):
            return
        
        rospy.loginfo('Connecting to model_pose_service.')
        self.model_pose_service = rospy.ServiceProxy('model_pose', ModelPose)
        self.model_pose_service.wait_for_service()
        rospy.loginfo('Succesfully connected.')


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)

        self.publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Connecting to move_base Action Server.")
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('Succesfully connected.')

        self.connect_model_pose_service()

        rospy.Subscriber('base_cmd', String, self.callback)

        rospy.spin()
        

if __name__ == '__main__':
    try:
        base_controller = TiagoBaseController()
        base_controller.start()
    except rospy.ROSInterruptException:
        pass