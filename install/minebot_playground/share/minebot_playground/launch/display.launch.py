import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Caminho para o seu pacote
    pkg_naviq_description = get_package_share_directory('minebot_playground')

    # Caminho para o arquivo URDF/XACRO
    urdf_file_path = os.path.join(pkg_naviq_description, 'urdf', 'naviq.urdf')

    # path to custom world
    world_file = os.path.join(pkg_naviq_description, 'worlds', 'world_mine.sdf')

    # Carrega o conteúdo do URDF ou XACRO (suporta ambos)
    if urdf_file_path.endswith('.xacro'):
        robot_description_config = xacro.process_file(urdf_file_path)
        robot_description = robot_description_config.toxml()
    else:
        # se for .urdf (XML já processado), apenas lê o arquivo
        with open(urdf_file_path, 'r') as f:
            robot_description = f.read()

    # 1. Iniciar o Gazebo (ros_gz_sim)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 3. Spawnar o robô no Gazebo (com pequeno offset no z)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'naviq',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02'
        ],
        output='screen'
    )

    # 4. ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen'
    )

    # 5. image_trans9port republish: raw -> compressed
    # remapeamos para publicar em /usb_cam/compressed (o seu subscriber espera usb_cam/compressed)
    image_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out', '/usbcam_node')   # -> vai publicar '/usb_cam/compressed'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        image_republisher_node
    ])
