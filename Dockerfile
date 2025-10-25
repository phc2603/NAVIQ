FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Instala dependências básicas
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Inicializa rosdep
RUN rosdep init || true
RUN rosdep update || true

# Copia workspace
COPY ./src /ros/src

WORKDIR /ros

# Instala dependências do workspace
RUN rosdep install --from-paths src --ignore-src -y || true

# Build do workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec bash"]
