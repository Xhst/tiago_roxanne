#!/bin/bash 

WORLD="3table"
END_EFFECTOR="pal-hey5" # pal-gripper , pal-hey5
USE_ROXANNE=true

gnome-terminal -- bash -c 'sleep 2s; cd; cd tiago_public_ws; chmod +x ./src/tiago_roxanne/bash/permission.bash; ./src/tiago_roxanne/bash/permission.bash' &>/dev/null

gnome-terminal --tab --title='tiago_gazebo' -e "bash -c 'cd; cd tiago_public_ws; source ./devel/setup.bash; roslaunch tiago_roxanne tiago_roxanne_gazebo_base.launch end_effector:=$END_EFFECTOR world:=$WORLD use_roxanne:=$USE_ROXANNE; exec bash -i'" --tab --title='tiago_input' -e "bash -c 'sleep 45s; cd; cd tiago_public_ws; source ./devel/setup.bash; rosrun tiago_roxanne tiago_input.py; exec bash -i'" &>/dev/null

if $USE_ROXANNE
then
    gnome-terminal --tab --title='roxanne' -e "bash -c 'sleep 40s; cd; cd tiago_public_ws; source ./devel/setup.bash; ./src/tiago_roxanne/bash/roxanne.bash; exec bash -i'" --tab --title='tiago_roxanne' -e "bash -c 'sleep 60s; cd; cd tiago_public_ws; source ./devel/setup.bash; rosrun tiago_roxanne tiago_roxanne.py; exec bash -i'" &>/dev/null
fi