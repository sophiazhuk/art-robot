# art-robot

## Set Up Guide
 
On a lab machine (or ssh), from a fresh terminal:
 
```bash
source set_pixi_env
cd /local/$USER
git clone git@github.com:sophiazhuk/art-robot.git
cd art-robot
pixi install
pixi run mock
```

Run `source set_pixi_env` in every new terminal before any pixi command. If you forget, pixi downloads everything again into your home directory.


| Command | What it does |
|---|---|
| `pixi run mock` | Simulated arm in RViz (no physics). |
| `pixi run moveit` | MoveIt motion planner with GUI. Run alongside `mock`, `sim`, or `driver`. |
| `pixi run sim` | Full Gazebo physics simulation with MoveIt. |
| `pixi run test` | Sends a test trajectory. Moves the arm, so use with `mock` first. |
| `pixi run driver` | Connects to the physical robot. See below. |
 

 ## Project Summary

Music Frequency Art Bot for CSCI 497F (Robotics). A UR3e robot arm that turns sound into drawings. We analyze the audio's frequencies and convert them into patterns the arm draws with a marker.

Group 6: Daisy N, Meadow C, Sophia Z, Haakon S

 ## References
 
- [Lab equipment info: UR3e arm](https://github.com/MRRP-lab/equipment-info/tree/main/robotarm)
- [Universal Robots ROS 2 driver docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)
- [MoveIt 2 docs](https://moveit.picknik.ai/main/index.html)
- [ROS 2 Jazzy launch file tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 executors and threading](https://docs.ros.org/en/jazzy/Concepts/Intermediate/AboutExecutors.html)