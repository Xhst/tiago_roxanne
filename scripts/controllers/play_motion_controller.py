#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

class TiagoTTSController:

    def __init__(self):
        self.node_name = 'play_motion_controller'
        self.client = None


    def motion(self, motion_name, timeout = rospy.Duration(30)):
        goal = PlayMotionGoal()
        goal.motion_name = motion_name

        self.client.send_goal_and_wait(goal, timeout)


    def callback(self, message):
        data = message.data

        split = data.split(" ", 1)

        self.motion(split[1])


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        
        rospy.loginfo("Connection to play_motion Action Server.")
        self.client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Succesfully connected.")
    
        rospy.Subscriber('play_motion_cmd', String, self.callback)

        rospy.wait_for_message("joint_states", JointState)
        
        rospy.spin()

    
if __name__ == '__main__':
    try:
        tts_controller = TiagoTTSController()
        tts_controller.start()
    except rospy.ROSInterruptException:
        pass