# FEUP-RI

## Requirements

These were the requirements used to develop this project:

- OS : Ubuntu 20.04
- Ros version: Noetic
- Simulator: Gazebo

## Installation

Clone this repository into src of your catkin workspace.

Then run the following:

```bash=
username@os:~/your/catkin_ws$ source devel/setup.bash
username@os:~/your/catkin_ws$ catkin_make
```
In three separate terminals run in order:

1:
```bash=
username@os:~/your/catkin_ws$ roscore
```

2:

```bash=
username@os:~/your/catkin_ws$ roslaunch follow_wall A1.launch
```

3:

```bash=
username@os:~/your/catkin_ws$ rosrun follow_wall follow_wall
```

You may need to run the śource command on all terminals.
