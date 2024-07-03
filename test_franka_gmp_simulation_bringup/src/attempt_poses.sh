#!/bin/bash

# STEP 001
#
# RUN FRANKA SIMULATION
gnome-terminal --tab --title="gazebo" --command="bash -c 'roslaunch test_franka_gmp_simulation_bringup full.launch'"
sleep 60

# STEP 002
#
# MOVE TO CAPTURE POSITION
python3 "move_to_capture.py"
sleep 7

# STEP 003
#
# ATTEMPT EACH GRASP POSE UNTIL SUCCESSFULL
python3 "move_to_poses.py"

# STEP 004
#
# CLOSE TERMINALS
gnome-terminal -- bash -c "pkill gnome-terminal"
sleep 10;