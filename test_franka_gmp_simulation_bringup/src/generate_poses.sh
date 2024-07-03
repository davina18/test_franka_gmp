#!/bin/bash

# SCRIPT PATHS
TRANSFORMATION_PYTHON_SCRIPT="$HOME/test_franka_gmp/test_franka_gmp_simulation_bringup/dev/AdaptiveGoalRegion/src/transform_and_cluster.py" # CHANGE TO YOUR OWN PATH
CONDA_PATH=$HOME/anaconda3/etc/profile.d/conda.sh

# STEP 001
#
# RUN FRANKA SIMULATION
gnome-terminal --tab --title="gazebo" --command="bash -c 'roslaunch test_franka_gmp_simulation_bringup full.launch'"
sleep 60


# STEP 002
#
# MOVE TO AND CAPTURE SCENE
gnome-terminal --tab --title="capture" --command="bash -c 'python3 capture_scene.py'"
python3 move_to_capture.py
sleep 7


# STEP 003
#
# CONTACT GRASP-NET 
source $CONDA_PATH
conda activate contact_graspnet_env
cd $HOME/contact_graspnet/contact_graspnet
python3 inference.py --forward_passes=10 --z_range=[0.3,1.4]
conda deactivate
cd $HOME


# STEP 004
#
# TRANSFORMATION AND CLUSTERING
python3 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$HOME/contact_graspnet/contact_graspnet/results/predictions_scene.npz" -o "$HOME/test_franka_gmp/test_franka_gmp_simulation_bringup/src/grasp_poses/" # CHANGE TO YOUR OWN PATHS


# STEP 005
#
# CLOSE TERMINALS
gnome-terminal -- bash -c "pkill gnome-terminal"
sleep 30;
