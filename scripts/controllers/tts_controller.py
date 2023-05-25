#!/usr/bin/env python
import rospy
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class TiagoTTSController:

    def __init__(self):
        self.node_name = 'tts_controller'
        self.client = None

    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        self.client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
        self.client.wait_for_server()
        #
        rospy.sleep(2)
        goal = TtsGoal()
        text = "Text to speak"
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        self.client.send_goal_and_wait(goal)
        #
    
if __name__ == '__main__':
    try:
        tts_controller = TiagoTTSController()
        tts_controller.start()
    except rospy.ROSInterruptException:
        pass