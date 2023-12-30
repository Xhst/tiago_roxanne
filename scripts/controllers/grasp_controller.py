#!/usr/bin/env python
import rospy
import sys
import os
from actionlib import SimpleActionClient
from std_msgs.msg import String
from moveit_msgs.msg import PickupAction, PlaceAction
from tiago_roxanne.srv import ModelPose

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils.grasp.spherical_grasps import SphericalGrasps
from utils.grasp.pick_and_place import create_pickup_goal, create_place_goal


class TiagoGraspController:

    def __init__(self):
        self.node_name = 'grasp_controller'

        self.grasped_model = ''

        self.use_model_pose_service = rospy.get_param('use_model_pose_service', False)


    def motion(self, name):
        self.play_motion_pub.publish(name)

    
    def pick_up(self, model, pose):
        rospy.loginfo('Picking up '+ model)
        grasps = self.spherical_grasp.create_grasps_from_object_pose(pose)
        goal = create_pickup_goal(grasps, model)

        self.pickup_ac.send_goal(goal)
        self.pickup_ac.wait_for_result()

        self.grasped_model = model


    def place(self, model, pose):
        rospy.loginfo('Placing '+ self.grasped_model +' on '+ model)
        placings  = self.spherical_grasp.create_placings_from_object_pose(pose)
        goal = create_place_goal(placings, model)

        self.pickup_ac.send_goal(goal)
        self.pickup_ac.wait_for_result()

        self.grasped_model = ''


    def callback(self, message):
        data = message.data

        split = data.split(' ', 1)

        command = split[0]
        model = split[1]

        try:
            poseResponse = self.model_pose_service(model)
        except rospy.ServiceException:
            rospy.logwarn('Cannot find model with name: '+ model)
            return
        except NameError:
            rospy.logwarn('Model pose service is disabled.')
            return

        if command == 'pick':
            self.pick_up(model, poseResponse.pose)
        if command == 'place':
            self.place(model, poseResponse.pose)
        

    def connect_pickup_and_place_action_servers(self):
        rospy.loginfo('Connecting to pickup Action Server.')
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo('Succesfully connected.')

        rospy.loginfo('Connecting to place Action Server.')
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo('Succesfully connected.')

    
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

        rospy.Subscriber('/tiago_roxanne/cmd/grasp', String, self.callback)

        self.play_motion_pub = rospy.Publisher('/tiago/roxanne/cmd/play_motion', String, queue_size=3)

        self.connect_model_pose_service()
        self.connect_pickup_and_place_action_servers()

        self.spherical_grasp = SphericalGrasps()

        rospy.spin()


if __name__ == '__main__':
    try:
        grasp_controller = TiagoGraspController()
        grasp_controller.start()
    except rospy.ROSInterruptException:
        pass