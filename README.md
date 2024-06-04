# DUT-COMI-ROS
The repo for Dalian University of Technology COMI-Lab.

## System requirements

* Ubuntu18.04 ARM64
* ROS-Melodic
* python 2.7

## Arm_requirement

* python 2.7

## Other requirements

Please refer to `requirements.txt` for python packages details.

## How to use (quick start)

1. Make sure you have installed ROS-Melodic and python 2.7. [Installing ROS Melodic, CSDN](https://blog.csdn.net/hxj0323/article/details/121215992)
2. You should initialize a ROS workspace first. If you don't know how to do it, please refer to [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
3. Clone this repo to another folder, and copy the `src` folder to the same folder as your ROS workspace.
4. Run `catkin_make` in command line. This is done by running `catkin_make` in the `catkin_ws` folder.
5. Run `source devel/setup.bash` in command line. You can put this command in your `~/.bashrc` file to make it run automatically.
6. Make sure you have all robots and robotic arms connected to you computer.
7. Run `rosrun  multi_nav_server multi_nav_server_static.py` in command line. This will start the server node and dispatch automatically.

## Package details

### arm_status_msgs / arm_work_msgs / car_status_msgs

These three packages are used to define the message types used in the project. You can find the message types in the `msg` folder of each package.

### comicar_multi

This package should be put in the robot car's workspace. Start the car and run `roslaunch comicar_multi comicar_nav_multi.launch` to lauch the car and related nodes.

### multi_nav_server

Refer to `scripts` folder to see details of each part's implementations.

Under `params` and `config` folder, you can find the configuration files for the jobs and arms.

When running `rosrun  multi_nav_server multi_nav_server_static.py` in command line, the server will start and dispatch automatically. You can add the number of cars in the system by adding the number in the command line, like `rosrun  multi_nav_server multi_nav_server_static.py 3`.

# DUT-COMI-ROS
大连理工大学COMI实验室的代码仓库。

## 系统要求

* Ubuntu18.04 ARM64
* ROS-Melodic
* python 2.7

## 机械臂要求

* python 2.7

## 其他要求

请参考 `requirements.txt` 文件了解python包的详细信息。

## 如何使用（快速开始）

1. 确保您已安装ROS-Melodic和python 2.7。[安装ROS Melodic, CSDN](https://blog.csdn.net/hxj0323/article/details/121215992)
2. 您应该首先初始化一个ROS工作空间。如果您不知道如何操作，请参考[安装和配置ROS环境](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)。
3. 将此仓库克隆到另一个文件夹，并将 `src` 文件夹复制到与您的ROS工作空间相同的文件夹中。
4. 在命令行中运行 `catkin_make`。这是通过在 `catkin_ws` 文件夹中运行 `catkin_make` 来完成的。
5. 在命令行中运行 `source devel/setup.bash`。您可以将此命令放在您的 `~/.bashrc` 文件中，以使其自动运行。
6. 确保所有机器人和机械臂已连接到您的计算机。
7. 在命令行中运行 `rosrun multi_nav_server multi_nav_server_static.py`。这将启动服务器节点并自动调度。

## 包详情

### arm_status_msgs / arm_work_msgs / car_status_msgs

这三个包用于定义项目中使用的消息类型。您可以在每个包的 `msg` 文件夗中找到消息类型。

### comicar_multi

此包应放在机器人车的工作空间中。启动车辆并运行 `roslaunch comicar_multi comicar_nav_multi.launch` 来启动车辆及相关节点。

### multi_nav_server

请参考 `scripts` 文件夹以查看每个部分的实现细节。

在 `params` 和 `config` 文件夹下，您可以找到作业和机械臂的配置文件。

当在命令行中运行 `rosrun multi_nav_server multi_nav_server_static.py` 时，服务器将自动启动并调度。您可以通过在命令行中添加车辆数量来增加系统中的车辆数量，例如 `rosrun multi_nav_server multi_nav_server_static.py 3`。