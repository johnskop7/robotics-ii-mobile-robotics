# Robotics II: Mobile Robot Control and Localization

Semester project for **Robotics II: Intelligent Robotic Systems**, School of Electrical and Computer Engineering, National Technical University of Athens (2022–2023).

The two assignment parts share a differential-drive robot simulated with **ROS 1 and Gazebo**, equipped with five ultrasonic sonar sensors and an IMU.

| Part | Objective | Included material |
| --- | --- | --- |
| A | Follow an obstacle boundary while maintaining a chosen distance | Handout, setup appendix, ROS wall-following **starter template** |
| B | Estimate position and orientation in a known 4 m × 4 m environment using an Extended Kalman Filter approach | Handout, appendix, Python localization implementation, C++ random walker, world, final report |

**Archive status:** Part A's supplied `follower.py` still contains `INSERT YOUR MAIN CODE HERE` and commands zero velocity. A completed Part A controller and report were not present in the supplied files. Part B preserves the submitted estimator, including its numerical limitations; see [implementation notes](docs/implementation-notes.md). This repository organizes the coursework rather than claiming a newly validated estimator.

## Contents

- [`ros/robo2_mobile/`](ros/robo2_mobile/): one catkin package containing both parts.
- [`docs/part-a/handout.pdf`](docs/part-a/handout.pdf) and [`appendix.pdf`](docs/part-a/appendix.pdf): Part A assignment and setup instructions.
- [`docs/part-b/handout.pdf`](docs/part-b/handout.pdf), [`appendix.pdf`](docs/part-b/appendix.pdf), and [`report.pdf`](docs/part-b/report.pdf): Part B assignment, appendix, and final report (Greek).
- [`docs/setup.md`](docs/setup.md): environment, build, and execution instructions.
- [`docs/implementation-notes.md`](docs/implementation-notes.md): scope, topics, and known limitations.
- [`docs/provenance.json`](docs/provenance.json): source-to-destination mapping and SHA-256 hashes for unchanged files.
- [`reference/original-integration/`](reference/original-integration/): original build metadata and simulator launch file.
- [`GITHUB_SETUP.md`](GITHUB_SETUP.md): repository name, description, topics, and upload instructions.

## Run the project

Use the assignment's ROS 1 environment. The setup guide targets **Ubuntu 20.04 / ROS Noetic / Python 3**, matching the Python shebangs. Installation and simulator execution have not been verified in this packaging environment.

After following [setup and build instructions](docs/setup.md), source `~/catkin_ws/devel/setup.bash` in every terminal.

**Part A** — start Gazebo and the supplied starter node in separate terminals:

```bash
roslaunch mymobibot_gazebo mymobibot_world_wf.launch
```

```bash
roslaunch robo2_mobile mobile_partA.launch
```

The template subscribes to sensors but requires the wall-following algorithm before it will move the robot.

**Part B** — start the world, estimator, and random walker in three separate terminals, in that order:

```bash
roslaunch robo2_mobile mymobibot_world_loc.launch yaw_init:=0.44
```

```bash
roslaunch robo2_mobile localization.launch
```

```bash
roslaunch robo2_mobile random_walk.launch
```

The value `0.44` follows the rounded initial heading stated in the report. The estimator contains an additional hard-coded orientation offset; see the implementation notes before interpreting its output. The random walker controls motion for three minutes. Stop the localization node separately with Ctrl+C.

## Methods and tooling

- Differential-drive simulation and ROS topic communication.
- Sonar-based obstacle sensing and randomized collision avoidance (C++).
- IMU-based motion prediction and geometric sonar measurements (Python/NumPy).
- Kalman-filter prediction and correction as attempted in the original coursework.
- State and sensor plots using Matplotlib; final analysis in the report.

## Authors and attribution

The Part B report credits **Ioannis Litsos** and **Nikolaos Benetos**. The handouts, appendices, starter package, random walker, and simulator supplements include course-provided material. The external robot model is maintained separately in [oikonpar/mymobibot](https://github.com/oikonpar/mymobibot).

See [NOTICE.md](NOTICE.md) for attribution and licensing status.
