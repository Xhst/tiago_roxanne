#!/usr/bin/env python
import rospy
import sys
import os
import json
from tiago_roxanne.srv import Position

class PositionService():
    def __init__(self, json_data):
        self.json_data = json_data


    def handler(self, request):

        try:
            json_pos = self.json_data.get(request.position_name)

            return (json_pos.get('x', 0.0), json_pos.get('y', 0.0), json_pos.get('cardinal', 'north'))
        except:
            return (0.0, 0.0, 'north')
        

    def start(self):
        rospy.init_node('position_service')
        rospy.Service('/tiago_roxanne/position_service', Position, self.handler)
        rospy.loginfo('Service "position_service" started')
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