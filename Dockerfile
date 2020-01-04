FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential python-catkin-tools

# Create ROS workspace
COPY . /ws/src/mesh_tools
WORKDIR /ws

# Use rosdep to install all dependencies (including ROS itself)
RUN rosdep install --from-paths src -i -y --rosdistro melodic

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    catkin init && catkin config --install && catkin build"
