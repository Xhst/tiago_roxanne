#!/usr/bin/env python
import rospy
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch, RLException

class TiagoRobot:
    
    def __init__(self):
        self.node_name = 'tiago_robot'
        self.log_level = rospy.INFO

        # 'pkg' and 'name' are required
        self.nodes = [
            {'pkg':'tiago_hrc', 'name':'head_controller', 'namespace':'controllers'},
            {'pkg':'tiago_hrc', 'name':'base_controller', 'namespace':'controllers'},
            {'pkg':'tiago_hrc', 'name':'torso_controller', 'namespace':'controllers', 'sleep':15},
            {'pkg':'tiago_hrc', 'name':'tts_controller', 'namespace':'controllers', 'sleep':15},
            {'pkg':'tiago_hrc', 'name':'play_motion_controller', 'namespace':'controllers', 'sleep':15},
            {'pkg':'tiago_hrc', 'name':'grasp_controller', 'namespace':'controllers'},
        ]


    def launch_node(self, package, node_name, namespace = '/', args = ''):
        '''
        Launch a roslaunch node instance.

        Parameters
        ----------
        package: str
            Node package.
        node: str
            Node name, it's also used to get the file (without the extension).
        namespace: str
            The directory where the file is located.
        args: str
            Args passed to the executable.
        ----------
        '''
        rospy.loginfo('Starting node %s from package %s.', node_name, package)        

        node_to_launch = Node(package, node_name+'.py', node_name, namespace, None, args)

        ros_launch = ROSLaunch()
        ros_launch.start()

        ros_launch.launch(node_to_launch)

        rospy.loginfo('Node %s started.', node_name)
        rospy.sleep(1)


    def launch_nodes(self):
        '''
        Launch all the roslaunch nodes instances
        '''

        for node in self.nodes:
            try:
                pkg = node['pkg']
                name = node['name']
                namespace = node.get('namespace', '/')

                self.launch_node(
                    pkg, 
                    name, 
                    namespace,
                    node.get('args', '')
                )

                # sleep after node launch, if duration is negative, sleep immediately returns.
                rospy.sleep(node.get('sleep', -1))

            except KeyError as e:
                rospy.logerr('"pkg" and "name" are required to identify the node to launch: %s', e)
            except RLException as e:
                rospy.logerr('Error while launching node: %s', e)


    def start(self):
        '''
        Start Tiago and all the nodes it needs.
        '''
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)
        rospy.loginfo('Starting Tiago Robot.')

        self.launch_nodes()

        rospy.loginfo('Tiago started.')

        rospy.spin()


if __name__ == '__main__':
    try:
        tiago_robot = TiagoRobot()
        tiago_robot.start()
    except rospy.ROSInterruptException:
        pass