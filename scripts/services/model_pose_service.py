#!/usr/bin/env python
import rospy
import sys
import os
from tiago_roxanne.srv import ModelPose

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils.xml_parser import XMLParser
from utils.pose import posestamped_from_positions_and_euler_angles


class ModelPoseService:

    def __init__(self, world_parser):
        self.world_parser = world_parser
        self.world_parser.root = self.world_parser.find_element_from_root('state', {'world_name':'default'})


    def handler(self, request):
        try:
            rospy.loginfo('Requested pose for model named: '+ request.model_name)
            model = self.world_parser.find_element_from_root('model', {'name':request.model_name})

            pose_str = self.world_parser.find_element(model, 'pose').text
            pose_parsed = pose_str.split(' ')

            return posestamped_from_positions_and_euler_angles(
                float(pose_parsed[0]),
                float(pose_parsed[1]),
                float(pose_parsed[2]),
                float(pose_parsed[3]),
                float(pose_parsed[4]),
                float(pose_parsed[5]),
            )
        except:
            return None


    def start(self):
        rospy.init_node('model_pose_service')
        rospy.Service('/tiago_roxanne/model_pose', ModelPose, self.handler)
        rospy.loginfo('Service "model_pose_service" started')
        rospy.spin()


if __name__ == '__main__':
    
    if(len(sys.argv) < 1):
        rospy.logerr('model_pose_service require an argument with the world file path.')
        sys.exit()

    world_xml_file = sys.argv[1]
    world_parser = XMLParser(world_xml_file)

    model_pose_service = ModelPoseService(world_parser)
    model_pose_service.start()