FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update || true

COPY ./src /ros/src

WORKDIR /ros

RUN rosdep install --from-paths src --ignore-src -y || true

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec bash"]
