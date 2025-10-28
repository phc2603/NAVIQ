from rclpy.node import Node
from geometry_msgs.msg import Twist

COMMANDS_TOPIC = '/cmd_vel'  # topic to control the robot


class Commands_Topic(Node):
    def __init__(self):
        #--setup publisher command node
        super().__init__('commands_topic_node_publisher')
        # --- Setup publisher attributes
        self.__robot_commands_publisher = None

        # --- Setup publisher topic
        self.__publisher_robot_commands_topic()

    #method to create the publisher topic
    def __publisher_robot_commands_topic(self):
        self.__robot_commands_publisher = self.create_publisher(
            Twist,
            COMMANDS_TOPIC,
            10  
        )

    def send_commands(self, twistCommand):
        twist = Twist()
        twist.linear.x = twistCommand.linear.x
        twist.linear.y = twistCommand.linear.y
        twist.linear.z = twistCommand.linear.z
        twist.angular.x = twistCommand.angular.x
        twist.angular.y = twistCommand.angular.y
        twist.angular.z = twistCommand.angular.z

        if self.__robot_commands_publisher is not None:
            self.__robot_commands_publisher.publish(twist)
            print(f"Sent cmd -> {twist.linear}; {twist.angular}")
        else:
            print("ROS client not connected to sendo robot commands")


