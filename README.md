# art-robot

## Set Up Guide
 
### Run the sim over SSH Windows
 
First time only, on the lab machine:
```bash
source set_pixi_env
cd /local/$USER
git clone git@github.com:sophiazhuk/art-robot.git
cd art-robot
pixi install
```
 
Every session:
 
1. Start VcXsrv (PowerShell):
```powershell
& "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -ac
```
2. Open terminal 1 and terminal 2 (PowerShell). In each:
```powershell
$env:DISPLAY = "127.0.0.1:0.0"
ssh -Y -J <user>@jump.cs.wwu.edu <user>@kb311-10.cs.wwu.edu
```
3. In both terminals:
```bash
source set_pixi_env
cd /local/$USER/art-robot
```
4. Terminal 1:
```bash
pixi run sim
```
5. Wait for RViz and Gazebo to open with the arm standing up.
6. Terminal 2:
```bash
pixi run square
```
7. Stop with `Ctrl+C` in each terminal.
On macOS, install XQuartz, skip steps 1 and the `$env:DISPLAY` line, and use the same `ssh -Y` command.

### Run on the arm in the robot lab
  
1. Log into the lab computer connected to the arm. Open three terminals. In each:
```bash
source set_pixi_env
cd /local/$USER/art-robot
git pull
```
2. Test in sim first. Terminal 1: `pixi run sim`. Terminal 2: `pixi run square`. Then `Ctrl+C` both.
3. Turn on the arm and set the teach pendant to remote control mode.
4. Clear the table and the area around the arm.
5. Terminal 1:
```bash
pixi run driver
```
6. Terminal 2:
```bash
pixi run moveit
```
7. Check that RViz shows the arm in its real pose.
8. Terminal 3, with a hand near the e-stop:
```bash
pixi run square
```
9. Stop with `Ctrl+C` in terminals 3, 2, then 1.

 ## Project Summary

Music Frequency Art Bot for CSCI 497F (Robotics). A UR3e robot arm that turns sound into drawings. We analyze the audio's frequencies and convert them into patterns the arm draws with a marker.

Group 6: Daisy N, Meadow C, Sophia Z, Haakon S

 ## References
 
- [Lab equipment info: UR3e arm](https://github.com/MRRP-lab/equipment-info/tree/main/robotarm)
- [Universal Robots ROS 2 driver docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)
- [MoveIt 2 docs](https://moveit.picknik.ai/main/index.html)
- [ROS 2 Jazzy launch file tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 executors and threading](https://docs.ros.org/en/jazzy/Concepts/Intermediate/AboutExecutors.html)
