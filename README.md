# Setup

```bash
git clone https://github.com/Vinothhk/Task2.git 
```
Install dependencies
```bash
rosdep install --from-paths src -y --ignore-src
```
Build & Source the Workspace
```bash
colcon build

source install/setup.bash
```
To Launch the robot
```bash
ros2 launch diffbot gazebo.launch.py
```
# Part 1 - Differential Drive Controller

This packages contains C++ Node to Listen Twist Messages from /cmd_vel, convert them into RPM and then publish them in topics /left_wheel_rpm and /right_wheel_rpm .

To run the Node,
```bash
ros2 run differential_drive_controller rpm_publisher
```

### Plugin for Wheels

A Plugin is created in "rpmdiffdrive" pakcage which is used to convert these RPM values into velocities and apply them in wheel joints.

Plugin Parameters:
- Wheel Joint Names
- RPM topics for wheels
- Wheel Base Lenght (in meter)
- Wheel Radius (in meter)

```xml
<plugin filename="libCustomDiffDrive.so" name="custom::CustomDiffDrive">
    <left_wheel_joint>left_wheel_joint</left_wheel_joint>
    <right_wheel_joint>right_wheel_joint</right_wheel_joint>
    <left_rpm_topic>/left_wheel_rpm</left_rpm_topic>
    <right_rpm_topic>/right_wheel_rpm</right_rpm_topic>
    <wheel_radius>0.2</wheel_radius>
    <wheel_base>1.1</wheel_base>
</plugin>
```

# Part 2 - Waypoint Navigation 

To run the python script to navigate the robot through waypoints:

```bash
ros2 run differential_drive_controller waypoint_navigation.py --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
```

If needed, PID parameters such as Kp, Ki and Kd can be tuned dynamically via rqt.

**After the robot reaches both the waypoints, the robot will be reset to origin via gz service.**