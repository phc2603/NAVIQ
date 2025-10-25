# Underlay: ROS 2 Jazzy do sistema
echo 'if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi' >> ~/.bashrc

# (Opcional) Overlay: seu workspace
echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' >> ~/.bashrc

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi
if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi

# Recarregar agora
source ~/.bashrc
