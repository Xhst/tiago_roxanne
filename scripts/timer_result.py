#!/usr/bin/env python
import rospy
import os
from tiago_roxanne.msg import TimerRequest

class TimerResult():

    def __init__(self):
        self.node_name = 'result_log'
        self.log_level = rospy.INFO

        self.file_path = os.path.dirname(__file__) + '/../../results/log.txt'

        self.pending_token = dict()
        self.first_token_time = None

        self.is_working = False



    def has_pending_token(self):
        return len(self.pendig_token) > 0
    
    def new_token_execution(self, exec):
        token = exec.token
        self.pending_token[token.id] = rospy.get_time()

        if not self.is_working:
            self.is_working = True
            self.first_token_time = rospy.get_time()


    def token_execution_finished(self, exec):
        token = exec.token
        exec_time = self.pending_token[token.id] - rospy.get_time()

        del self.pending_token[token.id]

        self.log_token(token, exec_time)

        if self.is_working and not self.has_pending_token() and len(exec.next) == 0:
            self.is_working = False
            self.log_total_exec_time(self.first_token_time - rospy.get_time())


    def log_token(self, token, exec_time):
        file = open(self.file_path, "a")

        file.write("token id: " + token.id + os.linesep)
        file.write("token component: " + token.component + os.linesep)
        file.write("token predicate: " + token.predicate + os.linesep)
        file.write("token parameter: " + token.parameter + os.linesep)
        file.write("token execution time: " + exec_time + os.linesep)
        file.write("___________________" + os.linesep)

        file.close()


    def log_total_exec_time(self, exec_time):
        file = open(self.file_path, "a")

        file.write("total execution time: " + exec_time + os.linesep)

        file.close()
        
    
    def callback(self, message):
        if message.is_pending:
            self.new_token_execution(message.execution)
            return

        self.token_execution_finished(message.execution)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)

        rospy.Subscriber("/tiago_roxanne/timer/token_execution" , TimerRequest, self.callback)

        rospy.spin()


if __name__ == '__main__':
    try:
        # plan, planner log

        timer_result = TimerResult()
        timer_result.start()
    except rospy.ROSInterruptException:
        pass