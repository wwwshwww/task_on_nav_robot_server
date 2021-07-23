FROM moyash/robo-gym-rs:base

ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=melodic ROBOGYM_WS=/robogym_ws MY_PKG=task_on_nav_robot_server

ADD . ${ROBOGYM_WS}/src/robo-gym-robot-servers/${MY_PKG}/

RUN apt update && apt upgrade -y && apt install --no-install-recommends xterm -y &&\
    cd ${ROBOGYM_WS}/src && git clone https://github.com/wwwshwww/hector_slam && \
    cd ${ROBOGYM_WS} && rosdep update && rosdep install --from-paths src -i -y --rosdistro ${ROS_DISTRO} && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo && \
    apt clean && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip && \
    pip install numpy-quaternion==2019.12.11.22.25.52 pcg_gazebo==0.7.12 roomor && \
    sed -i -e '/^.*init_node.*/d' /usr/local/lib/python2.7/dist-packages/pcg_gazebo/task_manager/gazebo_proxy.py