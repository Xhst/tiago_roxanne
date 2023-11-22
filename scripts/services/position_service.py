#!/usr/bin/env python
import rospy
import sys
import os
import json
from tiago_hrc.srv import Position


class PositionService():
    def __init__(self, json_data):
        self.json_data = json_data

    def handler(self, request):

        home_position = Position()
        home_position.x = 0.0
        home_position.y = 0.0
        home_position.cardinal = 'north'

        try:
            json_pos = self.json_data.get(request.position_name)
            position = Position()
            position.x = json_pos.get('x', 0.0)
            position.y = json_pos.get('y', 0.0)
            position.cardinal = json_pos.get('cardinal', 'north')

            return position
        except:
            return home_position
        

    def start(self):
        rospy.init_node('object_position_service')
        rospy.Service('model_pose', Position, self.handler)
        rospy.loginfo('Service "model_pose_service" started')
        rospy.spin()


if __name__ == '__main__':
    
    try:
        file_path = os.path.dirname(__file__) + '/../../config/positions.json'
        file = open(file_path)

        data = json.load(file)

        position_service = PositionService(data)
        position_service.start()
    except:
        rospy.logerr("Positions file not found")
        sys.exit()
