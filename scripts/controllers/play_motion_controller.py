#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from roxanne_rosjava_msgs.msg import TokenExecution, TokenExecutionFeedback
from tiago_roxanne.msg import TimerRequest

class TiagoTTSController:

    def __init__(self):
        self.node_name = 'play_motion_controller'
        
        self.use_roxanne = rospy.get_param('use_roxanne', True)


    def motion(self, motion_name, timeout = rospy.Duration(30)):
        goal = PlayMotionGoal()
        goal.motion_name = motion_name

        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout)

        return result


    def command_callback(self, message):
        data = message.data

        split = data.split(" ", 1)

        self.motion(split[1])


    def send_timer_request(self, execution, is_pending = True):
        timer_req = TimerRequest()
        timer_req.execution = execution
        timer_req.is_pending = is_pending

        self.timer_publisher.publish(timer_req)


    def roxanne_execution_callback(self, execution):
        rospy.loginfo(execution)
        if execution.token.component == 'play_motion':

            self.send_timer_request(execution)

            if execution.token.predicate == 'Playing' and len(execution.token.parameters) == 1:

                result = self.motion(str(execution.token.parameters[0]))
                
                self.send_roxanne_feedback(int(not result), execution.tokenId)
            
            self.send_timer_request(execution, False)


    def send_roxanne_feedback(self, code, id):
        feedback = TokenExecutionFeedback()
        feedback.tokenId = id
        feedback.code = code

        self.roxanne_publisher.publish(feedback)

    
    def connect_roxanne_nodes(self):
        if not bool(self.use_roxanne):
            return

        rospy.loginfo('Starting /roxanne/acting/feedback/play_motion')
        self.roxanne_publisher = rospy.Publisher('/roxanne/acting/feedback/play_motion', TokenExecutionFeedback, queue_size=1)
        rospy.loginfo('Subscribing to /roxanne/acting/dispatching/play_motion')
        rospy.Subscriber('/roxanne/acting/dispatching/play_motion', TokenExecution, self.roxanne_execution_callback)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        
        rospy.loginfo("Connection to play_motion Action Server.")
        self.client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Succesfully connected.")
    
        rospy.Subscriber('/tiago_roxanne/cmd/play_motion', String, self.command_callback)

        rospy.wait_for_message("joint_states", JointState)

        self.connect_roxanne_nodes()
        
        rospy.spin()

    
if __name__ == '__main__':
    try:
        tts_controller = TiagoTTSController()
        tts_controller.start()
    except rospy.ROSInterruptException:
        pass