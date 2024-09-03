# 功能介绍

该功能包通过接收物体识别节点的消息，控制机器人抓取物体

# 使用方法

## 准备工作

1. 具备TonyPi机器人，包含机器人本体、相机及RDK套件，并且能够正常运行。
2. 具备小球等相关道具

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，打开终端拉取相应代码并编译安装

```bash
# 拉取人形机器人SDK并安装
mkdir -p /home/pi && cd /home/pi

# RDK X5
git clone https://github.com/wunuo1/TonyPi.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/TonyPi.git -b feature-foxy-x3

cd /home/pi/TonyPi/HiwonderSDK
pip install .

# 拉取抓取控制代码、控制消息代码、任务拆解代码
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src

# RDK X5
git clone https://github.com/wunuo1/tonypi_obj_grasp_control.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/tonypi_obj_grasp_control.git -b feature-foxy-x3

git clone https://github.com/wunuo1/robot_pick_obj_msg.git
git clone https://github.com/wunuo1/hobot_awareness.git

# 编译
cd ..
source /opt/tros/setup.bash
colcon build
```

**2.运行抓取/放置功能**

```shell
source ~/tonypi_ws/install/setup.bash

#固定相对位置的抓取
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=False fixed_rel_pos:=True target_type:=red_ball task_type:=catch

#固定高度目标抓取
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=False fixed_rel_pos:=False target_type:=red_ball task_type:=catch

#接收大模型任务进行抓取/放置
ros2 launch tonypi_obj_grasp_control target_grasp_control.launch.py task_input:=True fixed_rel_pos:=False
```


# 接口说明

## 话题

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| robot_target_detection      | ai_msgs::msg::PerceptionTargets        | 目标物体的位置消息                  |

## 服务

### 接收请求

|名称  | 类型                                    | 说明            |
|------| -------------------------------------------| --------------------------------|
|/task |robot_pick_obj_msg::srv::TaskExecution      | 接收执行任务，执行动作+目标。请求：string target_type string task_type ；回复：bool successful|


## 参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | ----------------------------------------------------- |
| task_input    | bool |    是否有任务输入，结合大模型任务拆解节点使用，默认值为False |
| fixed_rel_pos    | bool |   是否是固定相对位置进行抓取，默认值为True |
| task_input    | string |    任务的类别（抓取或放置），默认值为catch |
| target_type    | string |    识别目标的类型，默认值为red_ball |