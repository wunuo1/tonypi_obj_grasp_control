# 功能介绍

该功能包通过接收物体识别节点的消息，控制机器人抓取物体

# 使用方法

## 准备工作

1. 具备TonyPi机器人，包含机器人本体、相机及RDK套件，并且能够正常运行。
2. 具备小球等相关道具

## 安装功能包

**1.安装功能包**

启动机器人后，通过终端SSH或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install libeigen3-dev
sudo apt install -y tros-tonypi-obj-grasp-control
```

**2.运行抓取/放置功能**

```shell
source /opt/tros/local_setup.bash

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