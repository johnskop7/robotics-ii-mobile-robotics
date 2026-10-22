# Setup and execution

## Environment

The supplied appendix lists Ubuntu 18.04 / ROS Melodic and Ubuntu 20.04 / ROS Noetic. These instructions choose the latter because both Python scripts explicitly use Python 3. They describe the original coursework stack and are not a ROS 2 migration or a tested modern replacement.

Start with an existing ROS Noetic desktop-full installation and initialized rosdep. Then install the dependencies named in the appendix, plus the Python libraries imported by the scripts:

```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-navigation ros-noetic-gazebo-ros-pkgs \
  python3-numpy python3-matplotlib
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/oikonpar/mymobibot.git
```

Download this repository and put its top-level `robotics-ii-mobile-robotics` directory inside `~/catkin_ws/src/`. Catkin discovers the nested package at `robotics-ii-mobile-robotics/ros/robo2_mobile`. Keep exactly one copy of the `robo2_mobile` package in the workspace.

The archived Part A metadata is under a `CATKIN_IGNORE` marker so catkin will not discover a second package there.

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
chmod +x src/robotics-ii-mobile-robotics/ros/robo2_mobile/scripts/*.py
catkin_make
source devel/setup.bash
rospack find robo2_mobile
```

The `mymobibot` repository supplies `mymobibot_description`, `mymobibot_control`, `mymobibot_gazebo`, and the Part A world. It is an external dependency; its files are not bundled here. No upstream commit was pinned in the supplied archives. If its layout differs, compare against the original appendices.

## Part A: wall-following template

In terminal 1:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch mymobibot_gazebo mymobibot_world_wf.launch
```

In terminal 2:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch robo2_mobile mobile_partA.launch
```

The actual launch filename in the archive is `mobile_partA.launch`; the appendix's `follower.launch` is not supplied. The starter node runs at 100 Hz and publishes zero velocities until its marked algorithm section is implemented. The handout defines initial yaw and clockwise/counterclockwise direction from the group code; adjust the external wall-following launch configuration according to the handout when completing Part A.

## Part B: localization and random walking

Source the workspace in each terminal. Start the following in separate terminals:

1. `roslaunch robo2_mobile mymobibot_world_loc.launch yaw_init:=0.44`
2. `roslaunch robo2_mobile localization.launch`
3. `roslaunch robo2_mobile random_walk.launch`

Wait for the robot and sensor topics to be available before starting the estimator, and start the walker after the estimator. The original estimator uses a fixed one-second startup delay rather than checking sensor readiness.

The local world launch loads this package's `worlds/mymobibot_loc.world`; no copying into the external simulator package is needed. Robot description and control still come from `mymobibot`. The launch default yaw remains 0.0 as in the supplied file; the example explicitly requests the report's rounded 0.44 rad value.

Random-walker defaults from the supplied launch file:

| Parameter | Value |
| --- | --- |
| Loop rate | 10 Hz |
| Forward velocity | 0.25 m/s |
| Angular velocity magnitude | 0.5 rad/s |
| Walking duration | 3 minutes |
| Front obstacle threshold | 0.35 m |
| Front-left / front-right threshold | 0.25 m |

The estimator defaults to 100 Hz; change it with `roslaunch robo2_mobile localization.launch rate:=50`. Its shutdown callback attempts to save `state.png` and `Positions.png` under ROS_HOME (normally `~/.ros/`). These plots use sample indices rather than elapsed seconds. No automatic stop is added to the estimator when the random walker finishes.

Run one assignment part at a time: both use the same robot command topic and the Python scripts share the global `/rate` parameter.

## Record results

For example, after confirming that these topics are present:

```bash
rosbag record -O localization_run /imu /sensor/sonar_F /sensor/sonar_FL \
  /sensor/sonar_FR /sensor/sonar_L /sensor/sonar_R \
  /x_pos /y_pos /theta_ang /x_sens /y_sens /mymobibot/cmd_vel
```

Bags and generated plots are excluded by `.gitignore`. For the data-analysis context, read the [final report](part-b/report.pdf).
