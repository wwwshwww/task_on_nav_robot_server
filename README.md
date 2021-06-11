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

### Environment Side

https://github.com/wwwshwww/task_on_nav_env