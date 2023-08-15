gnome-terminal -- bash -c 'sleep 2s; cd; cd tiago_public_ws; chmod +x ./src/tiago_hrc/bash/permission.bash; ./src/tiago_hrc/bash/permission.bash'

gnome-terminal --tab --title='tiago_gazebo' -e "bash -c 'cd; cd tiago_public_ws; source ./devel/setup.bash; roslaunch tiago_hrc tiago_hrc_gazebo_base.launch; exec bash -i'" --tab --title='tiago_input' -e "bash -c 'cd; cd tiago_public_ws; source ./devel/setup.bash; rosrun tiago_hrc tiago_input.py; exec bash -i'"
#--tab --title='tiago_robot' -e "bash -c 'cd; cd tiago_public_ws; source ./devel/setup.bash; rosrun tiago_hrc tiago_robot.py; exec bash -i'"