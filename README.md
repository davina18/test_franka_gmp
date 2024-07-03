# CUCR Lab Demos (Franka GMP). How to Use:

These packages can be installed through [tue-env](https://github.com/CardiffUniversityComputationalRobotics/tue-env).

## Simulation tests

### 0. Installing (checking out) Franka simulation tests

Before installing any new package, update your installed packages:

```bash
cucr-get update
```

To install (check out) the Franka tests in simulation:

```bash
cucr-get install ros-test_franka_simulation_gmp_bringup
```

To build the new packages in either case:

```bash
cucr-make
```

Source updated setup files:

```bash
source ~/.bashrc
```

### 1. Grasp Pose Generation in Gazebo:

Navigate to the following directory:

```bash
cd test_franka_gmp_simulation_bringup/src/
```

Run the bash script:

```bash
bash generate_poses.sh
```
:warning: **Warning** :warning: Contact GraspNet must be installed as a prerequisite.

:warning: **Warning** :warning: The paths in the following files must be changed to your own paths: generate_poses.sh, capture_scene.py.

### 2. Attempting Grasp Poses in Gazebo:

Navigate to the following directory:

```bash
cd test_franka_gmp_simulation_bringup/src/
```
