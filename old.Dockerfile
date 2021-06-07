FROM joanneumrobotics/robo-gym-robot-server-side

ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO=melodic
ENV ROBOGYM_WS=/robogym_ws

RUN cd $ROBOGYM_WS/src/robo-gym-robot-servers && \
    git clone https://github.com/wwwshwww/task_on_nav_robot_server.git && \
    cd $ROBOGYM_WS && \
    apt-get update && \
    apt install -y ros-melodic-hector-slam ros-melodic-move-base ros-melodic-move-base-msgs \
    ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-urdf && \
    rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO --as-root=apt:false && \
    catkin init && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo

RUN pip install --upgrade pip &&\
    pip install numpy-quaternion==2019.12.11.22.25.52 pcg_gazebo==0.7.12 roomor 

RUN sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py 