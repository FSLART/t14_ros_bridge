FROM ros:humble

# update the system
RUN apt update
RUN apt upgrade -y

# install dependencies
RUN apt install python3-can ros-humble-ackermann-msgs ros-humble-foxglove-bridge -y

# create the workspace
RUN mkdir -p /app/src

# copy the source
COPY . /app/src/t14_ros_bridge
WORKDIR /app

# build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /app && colcon build"

# run the bridge
CMD /bin/bash -c "source /app/install/setup.bash && ros2 run t14_ros_bridge bridge"