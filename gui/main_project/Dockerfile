
FROM osrf/ros:jazzy-desktop


RUN apt-get update && apt-get install -y \
    python3-pip python3-tk python3-pyqt5 python3-pyqt5.qtsvg \
    x11-apps git nano \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /app


COPY . /app

#python dependencies
RUN pip3 install --no-cache-dir \
    pygame \
    pillow \
    rclpy \
    setuptools \
    colcon-common-extensions \
    && if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt; fi


SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc


ENTRYPOINT ["bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 src/main.py"]
