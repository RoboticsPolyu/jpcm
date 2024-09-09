#!/bin/bash
echo -e " ${RED} [ Running px4ctrl nodes ] ${NC}"
source /home/amov/Fast250/devel/setup.bash

roslaunch px4ctrl vicon.launch & sleep 1;

echo -e " ${RED} [ Running px4ctrl nodes ] ${NC}" 
source /home/amov/Fast250/devel/setup.bash

roslaunch px4ctrl run_fusion.launch & sleep 1;

wait;
