#!/bin/bash
echo -e " ${RED} [ Running jpcm nodes ] ${NC}"
source /home/amov/Fast250/devel/setup.bash

roslaunch jpcm vicon.launch & sleep 1;

echo -e " ${RED} [ Running fusion nodes ] ${NC}" 
source /home/amov/Fast250/devel/setup.bash

roslaunch jpcm run_fusion.launch & sleep 1;

wait;
