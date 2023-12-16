# turtle_controller
[日本語(Qiita)](https://qiita.com/Azi/items/6cd6944fc3a94f7f694a)

## Introduction
Using Nintendo Switch Joy Controller on ROS2.
In this package, you can manipulate turtlesim.

<img src="media_for_readme/VID_20220105_vertical_Trim.gif" width="300">

## Dependencies

- ROS2 Foxy
- [pygame](https://www.pygame.org/docs/)
- [turtlesim](https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html)

## My Environment

- CPU: intel core i5 8250U
- OS : Ubuntu 20.04.3LTS
- ROS: ROS2 foxy

## Installation of turtlesim

```shell
# Install the turtlesim package.
$ sudo apt update
$ sudo apt install ros-foxy-turtlesim

# Start turtlesim
$ ros2 run turtlesim turtlesim_node
```

## Installation of this package

```shell
$ . /opt/ros/foxy/setup.bash
$ mkdir -p ros2_ws/src && cd ros2_ws/src

$ git clone https://github.com/IotaKataoka/turtle_controller.git
$ pip3 install pygame

$ cd ~/ros2_ws
$ rosdep install -i --from-path src --rosdistro foxy -y
$ colcon build --symlink-install
```

## Demo turtle_controller
Open a new terminal, and source setup.
```shell
$ cd ros2_ws
$ . /opt/ros/foxy/setup.bash
$ . install/setup.bash
```
Next command will fail, if JoyCon is not connected to PC.
Please connnect by Bluetooth.

<img src="media_for_readme/IMG_20220105_JoyConC_small.jpg" width="250">

Run the turtle_controller Node!

```shell
$ ros2 run turtle_controller turtle_controller
```

## Change Orientation
When you want to use Joy-Con L or change the orentation,
look at line 16 in [turtle_controller.py](turtle_controller/turtle_controller.py) and edit the value of ROTATON.
```turtle_controller.py
11  #-------- Rotation --------#
12  # JoyCon R, Vertical   => 0
13  # JoyCon L, Vertical   => 1
14  # JoyCon R, Horizontal => 2
15  # JoyCon L, Horizontal => 3
16  ROTATION = 0
```

<img src="media_for_readme/VID_20220105_horizontal_Trim.gif" width="300">

## About writer
- Azi : Japanese student majoring in marine system engineering.
- Twitter : https://twitter.com/Azi_mark01
- Qiita (Japanese) : https://qiita.com/Azi
