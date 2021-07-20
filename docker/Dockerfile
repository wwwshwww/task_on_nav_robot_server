FROM moyash/robo-gym-rs:base

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=melodic ROBOGYM_WS=/robogym_ws MY_PKG=task_on_nav_robot_server

ADD ../* $ROBOGYM_WS/src/robo-gym-robot-servers/$MY_PKG/

RUN cd $ROBOGYM_WS && apt update && \
    rosdep update && rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

WORKDIR $HOME/tmp-module
EXPOSE 54321
RUN git clone https://github.com/wwwshwww/robo-gym-server-modules.git . && \
    pip uninstall robo-server-modules && pip install -e . && \
    pip install --upgrade pip && \
    pip install numpy-quaternion==2019.12.11.22.25.52 pcg_gazebo==0.7.12 roomor && \
    sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py 