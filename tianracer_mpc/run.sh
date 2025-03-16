chmod +x ./src/tianracer/tianracer_mpc/scripts/*

catkin_make

source ./devel/setup.bash

roslaunch tianracer_mpc demo_tianracer_mpc_nav.launch


