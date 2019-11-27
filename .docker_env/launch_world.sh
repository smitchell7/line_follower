#!/bin/bash

echo "launching xvfb"
screen -S display -dm bash -i -c 'Xvfb :1 -screen 0 1920x1080x24'
sleep 1

echo "launching gazebo world"
"roslaunch ard_forklift_launch world_sim.launch"


