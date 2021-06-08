# `task_on_nav` Robot-Server-Side

## Installation

Reference:

- https://github.com/jr-robotics/robo-gym
- https://github.com/jr-robotics/robo-gym-robot-servers

```
pip install --upgrade pip && pip install numpy-quaternion==2019.12.11.22.25.52 pcg_gazebo==0.7.12 roomor
cd ~/robogym_ws/src/robo-gym-robot-servers
git clone https://github.com/wwwshwww/task_on_nav_robot_server.git
cd ../../
rosdep install --from-paths src -i -y --rosdistro melodic
catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
source devel/setup.bash
```

And then, remove `rospy.init_node()` at `/usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py` by using command such as below.

`sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py`

### Docker

`moyash/robo-gym-rs:cuberoom`

### Environment Side

https://github.com/wwwshwww/task_on_nav_env