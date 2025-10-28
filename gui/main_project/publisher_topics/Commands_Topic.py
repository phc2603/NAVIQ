import roslibpy

COMMANDS_TOPIC = '/cmd_vel'  # topic to control the robot


class Commands_Topic:
    def __init__(self, client):
        # --- Setup client by parameter
        self.__ros_client = client

        # --- Setup publisher attributes
        self.__robot_commands_publisher = None

        # --- Setup publisher topic
        self.__publisher_robot_commands_topic()

    #method to create the publisher topic
    def __publisher_robot_commands_topic(self):
        self.__robot_commands_publisher = roslibpy.Topic(
            self.__ros_client,
            COMMANDS_TOPIC,
            'std_msgs/Twist'
        )

    def send_commands(self, twistCommand):
        if (self.__ros_client.is_connected) and (self.__robot_commands_publisher is not None):
            self.__robot_commands_publisher.publish(twistCommand)
            print(f"Sent cmd -> {twistCommand}")
        else:
            print("ROS client not connected to sendo robot commands")


