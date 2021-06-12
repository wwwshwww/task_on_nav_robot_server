# `task_on_nav` Robot-Server-Side

<img src='http://drive.google.com/uc?export=view&id=1X7nUh9TZ7J2qQBCnxsvelqz5fS3D2IPR' width=80% center>

<img src='http://drive.google.com/uc?export=view&id=1S2LvSdFfj9dmrkPv8e0YZRKaWa-AIJK7' width=80%>

<img src='http://drive.google.com/uc?export=view&id=1dGHVnu6JXPqjYnh92ZxUueYqzY1SFWKN' width=80%>

## Installation

Reference:

- https://github.com/jr-robotics/robo-gym
- https://github.com/jr-robotics/robo-gym-robot-servers

```bash
pip install --upgrade pip && pip install numpy-quaternion==2019.12.11.22.25.52 pcg_gazebo==0.7.12 roomor
cd ~/robogym_ws/src/robo-gym-robot-servers
git clone https://github.com/wwwshwww/task_on_nav_robot_server.git
cd ../../
rosdep install --from-paths src -i -y --rosdistro melodic
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
source devel/setup.bash
```

And then, remove `rospy.init_node()` at `/usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py` by using command such as below.

```bash
sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py
```

### Docker

This env of Robot-Server-Side can use from docker image below. That version using port for GRPC is fixed to 54321.

docker image: `moyash/robo-gym-rs:cuberoom`

Usage:

```bash
docker run --rm --name robot-server-side --expose 54321 moyash/robo-gym-rs:cuberoom
```

## Details

### robot server state:

- map size: [1, ]
- actual data of Occupancy Grid: [*map_size***2, ]
- trueth data of Occupancy Grid: [*map_size***2, ]
- 2D pose of agent on the `world` frame (pos x, pos y, ori z): [3, ]
- agent twist (linear x, angular z): [2, ]
- flag of agent collision: [1, ]
- flag of to generate new room when resetting env: [1, ]
- flag of to generate new agent pose when resetting env: [1, ]
- parameters of room generator: [8, ]
- 2D poses of target on the `world` frame (pos x, pos y, ori z): [*n*\*3, ]

Parameters of room generator is used for generating new room by `CubeRoomGenerator`.
Details: https://github.com/wwwshwww/roomor

### robot server action

- Pose that let Agent to try to reach on the `map` frame (pos x, pos y, ori z): [3, ]

Robot is controlled by move_base from Navigation Stack.

### Environment Side

https://github.com/wwwshwww/task_on_nav_env