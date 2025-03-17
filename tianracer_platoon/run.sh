# chmod +x ./src/tianracer/tianracer_mpc/scripts/*
chmod +x./src/tianracer/tianracer_platoon/scripts/*

catkin_make

source ./devel/setup.bash

roslaunch tianracer_platoon demo_sim_leader_follower_path_tracking.launch


