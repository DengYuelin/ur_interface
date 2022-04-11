# A ROS interface for multiple Universal Robots

## Installing interface docker image

A [simple docker instruction](docker/Docker_tutorial.md) to get started with docker.

Download docker image:

```bash
docker pull yuelindeng/ur_interface
```

## Using joystick for velocity control

Run docker driver:

```bash
docker run -it --rm --net=host yuelindeng/ur_interface /bin/bash
```

Set UR arm to **remote** mode, then run the following command in side docker container:

```bash
source catkin_ws/devel/setup.bash
roslaunch arm_controller single_ur3e_control.launch robot_ip:=192.168.0.102 robot_name:=left
```

Replace with correct ip address and robot type.

In **host** terminal, run joy_node:

```bash
rosrun joy joy_node
```

NOTE: the code is written for xbox family joysticks, i.e. D-Pad is interpreted as axis, not buttons.

## Pose control

Default TCP is located at /catkin_ws/src/ur_interface/arm_controller/config/default_config.yaml

Run docker driver:

```bash
docker run -it --rm --net=host yuelindeng/ur_interface /bin/bash
```

Set UR arm to **remote** mode, then run the following command in side docker container:

```bash
source catkin_ws/devel/setup.bash
roslaunch arm_controller single_ur3e_control.launch robot_ip:=192.168.0.102 robot_name:=left controller_mode:=1
```

Pose control signals can be published to topic /left/pose_cmd
