#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class TiagoInput:

    class CommandData:
        def __init__(self, name, args = '', desc = '', pub = None):
            self.name = name
            self.arguments = args
            self.description = desc
            self.publisher = pub


    def __init__(self):
        self.node_name = 'tiago_input'
        self.log_level = rospy.INFO
        
        self.publisher_suffix = '_cmd'

        # dictionary key must be equal to command name
        self.commands = {
            'base' : self.CommandData('base', '<linear> <angular> <duration>',
                                      ('Move or rotate the base of the robot.\n\n'
                                       '<linear>   (float): move ahead (positive values) or backwards (negative values) the robot.\n'
                                       '<angular>  (float): rotate the robot (clockwise positive)\n'
                                       '<duration> (float): duration in seconds.')),
            'head' : self.CommandData('head', '<horizontal> <vertical> <duration>',
                                      ('<horizontal> (float): move head horizontally (values between -1 and 1)'
                                       '<vertical>   (float): move head vertically (values between -1 and 1)'
                                       '<duration>   (int): duration in seconds')),
            'torso': self.CommandData('torso', '<position> <duration>')
        }


    def parse_command(self, input_str):
        parsed = input_str.split(' ', 1)

        command = parsed[0]

        try:
            args = parsed[1]
        except IndexError:
            args = ''

        if command == 'help':
            # print available commands
            if args == '': 
                output = 'Available commands are: help <command> | '
                for command in self.commands.values():
                    output += command.name + ' ' + command.arguments + ' | '
                print(output)
            # print the description of the given command (help <command>)
            else:
                try:
                    print('Description of command "'+ args +'"')
                    print(self.commands[args].description)
                except:
                    print('Command "'+ args +'" not found.')
            return

        self.commands[command].publisher.publish(args)

    
    def start(self):
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)
        rospy.loginfo('Starting Tiago input node.')

        for command in self.commands.values():
            command.publisher = rospy.Publisher(command.name + self.publisher_suffix, String, queue_size=3)

        rospy.loginfo('Tiago input node started.')
        print('Write "help" for commands list.')

        input_str = ''

        while not rospy.is_shutdown():
            input_str = raw_input()
            try:
                self.parse_command(input_str)
            except:
                print("Invalid command.")


if __name__ == '__main__':
    try:
        tiago_input = TiagoInput()
        tiago_input.start()
    except rospy.ROSInterruptException:
        pass