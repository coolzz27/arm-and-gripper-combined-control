# arm-and-gripper-combined-control

同时控制 Flexiv Rizon 4s 机械臂和 Robotiq EPick 真空吸盘。

## 环境依赖

1. 机械臂控制依赖 [Flexiv RDK v1.5](https://github.com/flexivrobotics/flexiv_rdk/tree/87800658ab4814bb4ec49026f0b218af1c85a823)，对应机械臂软件 Flexiv Elements v3.7。在 python3.10 虚拟环境下运行机械臂控制程序。

2. ros noetic 吸盘控制使用ros包 [robotiq_epick_control](https://github.com/JureHudoklin/robotiq_epick_control)，这个包原本使用的是python2语法，修改成python3语法放在 ```./robotiq_epick_control-master```。在 python3.8 虚拟环境下运行吸盘控制程序。

## 控制逻辑

将吸盘控制封装为 ros service (activate_epick 和 release_epick)，让机械臂控制程序调用。

## 使用方法

1. terminal 1: ```roscore```
2. terminal 2: ```roslaunch robotiq_epick_control robotiq_EPickSimpleCOntroller_launch.launch```
3. terminal 3: ```python3.8 combined_control/epick_ros_server.py```
4. terminal 4: ```python3.10 combined_control/arm_control.py```
