# tonypi_obj_grasp_control
# Function Introduction

This package controls the robot to grasp objects by receiving messages from the object recognition node.

# Usage

## Preparations

1. Have a TonyPi robot, including the robot body, camera, and RDK suite, and ensure it runs normally.
2. Prepare relevant props such as small balls.

## Install the Package

**1. Install the package**

After starting the robot, connect to the robot through terminal SSH or VNC, click the "One-click Deployment" button at the top right of this page, copy the following command to run on the RDK system to complete the installation of the relevant Node.

```bash
sudo apt update
sudo apt install libeigen3-dev
sudo apt install -y tros-tonypi-obj-grasp-control
```
**2. Run the Task Decomposition Function**

```shell
source /opt/tros/local_setup.bash

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