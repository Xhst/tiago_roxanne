#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

class TiagoBaseController:

    def __init__(self):
        self.node_name = 'base_controller'
        self.publisher = None
        self.dict = {'ciao': None}


    def callback(self, message):
        data = message.data

        split = data.split(" ")
        
        linear = Vector3()
        linear.x = float(split[0])
        angular = Vector3()
        angular.z = float(split[1])
        duration = float(split[2])

        self.move_to(linear, angular, duration)


    def start(self):
        rospy.init_node(self.node_name, anonymous=False)
        rospy.loginfo('Node %s started', self.node_name)
        self.publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_cmd', String, self.callback)
        rospy.spin()
    
    def move_to(self, linear, angular, duration):
        ##
        twist = Twist()
        twist.linear = linear
        twist.angular = angular

        now = rospy.Time.now()
        end_duration = now + rospy.Duration.from_sec(duration)

        rate = rospy.Rate(10)
        while rospy.Time.now() < end_duration:
            self.publisher.publish(twist)
            rate.sleep()
        

if __name__ == '__main__':
    try:
        base_controller = TiagoBaseController()
        base_controller.start()

    except rospy.ROSInterruptException:
        pass