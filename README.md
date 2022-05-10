# epistemic-planning
Repo for visualization and demonstration purposes of the temporal extension of epistemic planning.

In order to launch simulation:

catkin_make

source devel/setup.bash

roslaunch epistemic_sim party.launch

In order to build custom map:

cd src/epistemic_sim/utils

replace map.png with the desired map

python convert_to_binary.py

python make_ROS_map.py

Follow directions, the floorplan file will be map_binary.py, save at rviz_map


change image: [file_name] rviz_map.yaml to the saved rviz_map.pgm file
