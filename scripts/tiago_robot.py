#!/usr/bin/env python
import rospy
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
from tiago_hrc_buttons.srv import permission

class TiagoRobot:
    
    def __init__(self):
        self.node_name = 'tiago_robot'
        self.log_level = rospy.INFO

        # 'pkg' and 'name' are required
        self.nodes = [
            {'pkg':'tiago_hrc_buttons', 'name':'head_controller', 'namespace':'controllers'},
            {'pkg':'tiago_hrc_buttons', 'name':'base_controller', 'namespace':'controllers'},
            {'pkg':'tiago_hrc_buttons', 'name':'torso_controller', 'namespace':'controllers'},
            #{'pkg':'tiago_hrc_buttons', 'name':'tts_controller', 'namespace':'controllers'},
            #{'pkg':'tiago_hrc_buttons', 'name':'prova', 'args':'test args', 'sleep':2}
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

        ros_launch = ROSLaunch()
        ros_launch.start()
        ros_launch.launch(Node(package, node_name+'.py', node_name, namespace, None, args))

        rospy.loginfo('Node %s started.', node_name)


    def launch_nodes(self):
        '''
        Launch all the roslaunch nodes instances
        '''
        #rospy.loginfo('Waiting for "permission" service.')
        #rospy.wait_for_service('permission')
        #rospy.loginfo('Service "permission" reached.')

        #permission_service = rospy.ServiceProxy('permission', permission)

        for node in self.nodes:
            try:
                pkg = node['pkg']
                name = node['name']
                namespace = node.get('namespace', '/')

                # path with workspace as root
                #path = './src/%s/scripts/%s/%s.py' % (pkg, namespace, name)

                # sets node's file as executable
                #permission_response = permission_service(path)

                #if not permission_response.success:
                #    rospy.logwarn('Error while setting permission for node %s in package %s', name, pkg)

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
            except rospy.ServiceException as e:
                rospy.logerr('Unable to set permission for node: %s', e)


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