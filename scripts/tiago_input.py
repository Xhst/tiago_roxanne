#!/usr/bin/env python
import rospy
import json
from utils.terminal import AnsiColor as C, err, warn, info
from std_msgs.msg import String

class TiagoInput:

    class CommandData:
        def __init__(self, name, args = '', desc = '', pub = ''):
            self.name = name
            self.arguments = args
            self.description = desc
            self.publisher_name = pub


    def __init__(self):
        self.node_name = 'tiago_input'
        self.log_level = rospy.INFO
        
        self.publisher_prefix = '/tiago_roxanne/cmd/'

        # path with workspace as root
        self.commands_json_path = './src/tiago_roxanne/config/commands.json'

        self.publishers_name = ['head', 'base', 'torso', 'play_motion', 'tts', 'grasp']
        self.publishers = {}
        self.commands = {}


    def start_publishers(self):
        for name in self.publishers_name:
            self.publishers[name] = rospy.Publisher(self.publisher_prefix + name, String, queue_size=3)


    def load_commands(self):
        file = open(self.commands_json_path)
        data = json.load(file)

        for command in data['commands']:
            try:
                name = command['name']
            except:
                err('[JSON_ERROR]: command requires "name".')
                continue

            args = ''
            desc = command.get('desc', '') + '\n\n'

            try:
                for arg in command.get('args'):
                    args += '<' + arg['name'] + '> '
                    desc += C.BLUE + '<' + arg['name'] + '> ' + C.WHITE + '(' + arg['type'] +'): ' + C.LIGHT_GRAY + arg.get('desc', '') + '\n'
            except:
                err('[JSON_ERROR]: Command named "'+ name + '" has arguments with "name" or "type" missing.')
                
            desc += C.END

            # use the publisher data or the command name if the publisher is missing 
            publisher_name = command.get('publisher', command['name'])
            self.commands[name] = self.CommandData(name, args, desc, publisher_name)

        file.close()


    def parse_command(self, input_str):
        try:
            parsed = input_str.split(' ', 1)

            command = parsed[0]

            try:
                args = parsed[1]
            except IndexError:
                args = ''

            if command == 'help':
                # print available commands
                if args == '': 
                    output = 'Available commands are:\n# ' + C.MAGENTA + 'help ' + C.LIGHT_GRAY + '<command>' + C.END + '\n'
                    for command in self.commands.values():
                        output += '# ' + C.MAGENTA + command.name + ' ' + C.LIGHT_GRAY + command.arguments + C.END + '\n'
                    print(output)
                # print the description of the given command (help <command>)
                else:
                    try:
                        print('Description of command "'+ C.MAGENTA + args + C.END +'"\n')
                        print(self.commands[args].description)
                    except:
                        warn('Command "'+ args +'" not found.')
                return
            
            commandData = self.commands[command]

            self.publishers[commandData.publisher_name].publish(command + ' ' + args)
        except:
            warn('Invalid command.')

    
    def start(self):
        rospy.init_node(self.node_name, anonymous=False, log_level=self.log_level)
        rospy.loginfo('Starting Tiago input node.')

        self.start_publishers()
        self.load_commands()

        rospy.loginfo('Tiago input node started.')
        info('Write "help" for commands list.')

        input_str = ''

        while not rospy.is_shutdown():
            input_str = raw_input()
            self.parse_command(input_str)
            


if __name__ == '__main__':
    try:
        tiago_input = TiagoInput()
        tiago_input.start()
    except rospy.ROSInterruptException:
        pass