#!/bin/bash

rosrun test_franka_common_bringup franka_grasping.py



# Step 3: Activate the Conda environment
source /home/furkan/anaconda3/etc/profile.d/conda.sh
conda activate contact_graspnet

# Step 4: Navigate to the directory of the first part
cd ~/contact_graspnet

python contact_graspnet/inference.py --np_path=test_data/franka_gazebo.npy --forward_passes=5 --z_range=[0.3,1.2] && wait
#sleep 10
# Step 5: Run the first part
# Example: python first_part.py
#sleep 10
conda deactivate
rosrun test_franka_common_bringup grasp_objects