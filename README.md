# tonypi_obj_grasp_control
# Function Introduction

This package controls the robot to grasp objects by receiving messages from the object recognition node.

# Usage

## Preparations

1. Have a TonyPi robot, including the robot body, camera, and RDK suite, and ensure it runs normally.
2. Prepare relevant props such as small balls.

## Compile and Runs

**1. Compile**

After starting the robot, connect to it via SSH or VNC on the terminal, open the terminal, pull the corresponding code, and compile and install it.

```bash
# Pull and install the robot SDK
mkdir -p /home/pi && cd /home/pi

# RDK X5
git clone https://github.com/wunuo1/TonyPi.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/TonyPi.git -b feature-foxy-x3

cd /home/pi/TonyPi/HiwonderSDK
pip install .

# Pull grab control code, control message code, and task disassembly code
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src

# RDK X5
git clone https://github.com/wunuo1/tonypi_obj_grasp_control.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/tonypi_obj_grasp_control.git -b feature-foxy-x3

git clone https://github.com/wunuo1/robot_pick_obj_msg.git
git clone https://github.com/wunuo1/hobot_awareness.git

# Compile
cd ..
source /opt/tros/setup.bash
colcon build
```
**2. Run the Task Decomposition Function**

```shell
source ~/tonypi_ws/install/setup.bash

# Grasping with fixed relative position
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=False fixed_rel_pos:=True target_type:=red_ball task_type:=catch

# Grasping with fixed height
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=False fixed_rel_pos:=False target_type:=red_ball task_type:=catch

# Receive tasks from the large model for grasping/placing
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=True fixed_rel_pos:=False

```

# Interface Description

## Topic

### Subscribed Topics

|Name  | Type                                  |  Description           |
|------| --------------------------------------| --------------------------------|
|/robot_target_detection |ai_msgs::msg::PerceptionTargets | Publishes information about obstacles|


## Services

### Send Request

|Name  | Type                                  |  Description           |
|------| --------------------------------------| --------------------------------|
|/task	|robot_pick_obj_msg::srv::TaskExecution	|Receives task execution, performs actions + targets. Request: string target_type string task_type; Response: bool successful|


## Parameters
| Parameter Name             | Type       | Description  |
| --------------------- | ----------- | ----------------------------------------------------- |
| task_input	|bool	|Whether there is task input, used in conjunction with the large model task decomposition node. Default value is False |
| fixed_rel_pos	|bool	|Whether to grasp with fixed relative position. Default value is True. |
| task_type	|string	|Task type (grasping or placing). Default value is catch |
| target_type	|string	|Type of the recognized target. Default value is red_ballfixed_rel_pos	bool	Whether to grasp with fixed relative position. Default value is True. |