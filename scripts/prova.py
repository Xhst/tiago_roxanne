#!/usr/bin/env python
import rospy
import sys

class TiagoTest:

    def __init__(self):
        self.node_name = 'test_node'


    def test(self, log):
        rospy.init_node(self.node_name)
        rospy.loginfo(log)
        

if __name__ == '__main__':
    try:
        text = ''
        if len(sys.argv) > 1:
            #last 2 args are __name and __log
            for arg in sys.argv[1:-2]:
                text += arg + ' '
        test = TiagoTest()
        test.test(text)
    except rospy.ROSInterruptException:
        pass