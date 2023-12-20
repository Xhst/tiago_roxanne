#!/usr/bin/env python
import rospy
import os
from roxanne_rosjava_msgs.srv import ActingConfigurationService
from roxanne_rosjava_msgs.msg import ActingGoal
from utils.roxanne_plan_parser import RoxannePlanParser

class TiagoRoxanne:

    def __init__(self, plan, agent = 'agent.properties'):
        self.node_name = 'tiago_roxanne'
        self.log_level = rospy.INFO
        
        self.config_folder = os.path.dirname(__file__) + '/../config/roxanne/'
        self.plan_folder = self.config_folder + 'plans/'

        self.plan_file = plan
        self.agent_file = agent


    def config(self):
        rospy.loginfo('Connecting to /roxanne/acting/configuration service.')
        roxanne_config_service = rospy.ServiceProxy('/roxanne/acting/configuration', ActingConfigurationService)
        roxanne_config_service.wait_for_service()
        rospy.loginfo('Succesfully connected.')

        result = roxanne_config_service(self.config_folder + self.agent_file)
        rospy.loginfo(result.message)

    
    def start(self):
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)
        rospy.loginfo('Starting Tiago Roxanne node.')

        self.config()

        rospy.loginfo('Starting /roxanne/acting/goal')
        self.roxanne_topic = rospy.Publisher('/roxanne/acting/goal', ActingGoal, queue_size=1) 

        parser = RoxannePlanParser(self.plan_folder + self.plan_file)
        goals = parser.parse()

        rospy.loginfo(goals)

        rospy.sleep(1)
        
        for goal in goals:
            rospy.loginfo(goal.goalId)
            self.roxanne_topic.publish(goal)
            rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        tiago_input = TiagoRoxanne('test_plan.json')
        tiago_input.start()
    except rospy.ROSInterruptException:
        pass