
FROM osrf/ros:humble-desktop

WORKDIR /workspace


RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    ros-humble-behaviortree-cpp* \
    ros-humble-test-msgs \
    libreadline-dev \
    openjdk-17-jdk \
    openjdk-17-jre 


RUN pip install setuptools

# Set up ROS 2 environment and ensure it's sourced
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 

ENV ROS_DISTRO=humble

# Install Python dependencies
RUN pip install --pre unified-planning[pyperplan,tamer] \
    && pip install ConfigSpace \
    && pip install typing_extensions==4.7.1 --upgrade

# Set up the workspace
RUN mkdir -p /workspace/src \
    && cd /workspace/src \
    && git clone https://github.com/PlanSys2/UPF4ROS2.git \
    && vcs import . < UPF4ROS2/upf.repos

# Build UPF4ROS2
RUN cd /workspace \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install
    
RUN cd /workspace \
    && echo "source install/setup.bash" >> ~/.bashrc 

CMD ["bash"]

