#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class TiagoTTSController:

    def __init__(self):
        self.node_name = 'tts_controller'
        self.client = None

        self.lang_id = rospy.get_param('tts_lang_id', 'en_GB')
        self.wait_before_speaking = rospy.get_param('tts_wait_before_speaking', 0)


    def say(self, text, lang_id = 'en_GB'):
        goal = TtsGoal()
        goal.wait_before_speaking = self.wait_before_speaking
        goal.rawtext.text = text
        goal.rawtext.lang_id = lang_id

        self.client.send_goal_and_wait(goal)


    def callback(self, message):
        data = message.data
        split = data.split(" ", 1)
        self.say(split[1], self.lang_id)
        

    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        
        rospy.loginfo('Connecting to tts Action Server.')
        self.client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
        self.client.wait_for_server()
        rospy.loginfo("Succesfully connected.")

        rospy.Subscriber('tts_cmd', String, self.callback)
        rospy.spin()

    
if __name__ == '__main__':
    try:
        tts_controller = TiagoTTSController()
        tts_controller.start()
    except rospy.ROSInterruptException:
        pass