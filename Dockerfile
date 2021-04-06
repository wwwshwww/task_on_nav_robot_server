FROM joanneumrobotics/robo-gym-robot-server-side

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO=melodic
ENV ROBOGYM_WS=/robogym_ws

RUN cd $ROBOGYM_WS/src/robo-gym-robot-servers && \
    git clone https://github.com/wwwshwww/task_on_nav_robot_server.git && \
    cd $ROBOGYM_WS && \
    apt-get update && \
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    catkin init && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

RUN pip install --upgrade pip ; \
    pip install roomor

RUN sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py 