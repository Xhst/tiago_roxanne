#!/usr/bin/env python
import rospy
import os
from tiago_roxanne.msg import TimerToken

class TimerResult():

    def __init__(self):
        self.node_name = 'result_log'
        self.log_level = rospy.INFO

        self.file_path = os.path.dirname(__file__) + '/../results/log.txt'

        self.pending_tokens = dict()
        self.first_token_time = None

        self.is_working = False


    def has_pending_token(self):
        return len(self.pending_tokens) > 0
    

    def new_token_execution(self, execution):
        token = execution.token

        rospy.loginfo("Received execution for token with id "+ str(token.id) +" ("+ str(token.component) + "." + str(token.predicate) +")")
        self.pending_tokens[token.id] = rospy.get_time()

        if not self.is_working:
            self.is_working = True
            self.first_token_time = rospy.get_time()


    def token_execution_finished(self, execution):
        token = execution.token
        exec_time = rospy.get_time() - self.pending_tokens[token.id]

        del self.pending_tokens[token.id]

        self.log_token(token, exec_time)

        if self.is_working and not self.has_pending_token() and len(execution.next) == 0:
            self.is_working = False
            self.log_total_exec_time(rospy.get_time() - self.first_token_time)


    def log_token(self, token, exec_time):
        rospy.loginfo("Loggin token "+ str(token.id) +" ("+ str(token.component) + "." + str(token.predicate) +"), exec time: "+ str(exec_time))
        
        file = open(self.file_path, "a")

        file.write("token id: " + str(token.id) + os.linesep)
        file.write("token component: " + str(token.component) + os.linesep)
        file.write("token predicate: " + str(token.predicate) + os.linesep)
        file.write("token parameters: " + str(token.parameters) + os.linesep)
        file.write("token execution time: " + str(exec_time) + os.linesep)
        file.write("___________________" + os.linesep)

        file.close()


    def log_total_exec_time(self, exec_time):
        file = open(self.file_path, "a")

        file.write("total execution time: " + str(exec_time) + os.linesep)

        file.close()
        
    
    def callback(self, message):
        if message.is_pending:
            self.new_token_execution(message.execution)
            return

        self.token_execution_finished(message.execution)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)

        rospy.loginfo('Starting TimerResult node.')

        rospy.Subscriber("/tiago_roxanne/timer/token_execution" , TimerToken, self.callback)

        rospy.loginfo('TimerResult node started.')

        rospy.spin()


if __name__ == '__main__':
    try:
        # plan, planner log

        timer_result = TimerResult()
        timer_result.start()
    except rospy.ROSInterruptException:
        pass