import roslibpy

from subscriber_topics.Image_Topic import Image_Topic
from subscriber_topics.CO2_Topic import CO2_Topic
from subscriber_topics.CH4_Topic import CH4_Topic
from publisher_topics.Commands_Topic import Commands_Topic

# --- Global setting ---
ROS_MASTER_IP = '172.30.220.39'
ROS_MASTER_PORT = 9090


class ROS_Bridge:
    def __init__(self):

        # --- Setup ROS client, subscribe and publisher attributes
        self.ros_client = roslibpy.Ros(host=ROS_MASTER_IP, port=ROS_MASTER_PORT)
        self.image_subscriber = None
        self.gas_co2_subscriber = None
        self.gas_ch4_subscriber = None
        self.robot_command_publisher = None

        # --- Setup ROS callbacks, for when the connection is ready or any error happens
        self.ros_client.on_ready(self.on_ros_connect)
        self.ros_client.on('error', self.on_ros_error)
        self.ros_client.on('close', self.on_ros_close)

        print(f"Trying to connect in: {ROS_MASTER_IP}:{ROS_MASTER_PORT}...")

        try:
            self.ros_client.run()
        except Exception as e:
            print(f"Failed to connect: {e}")

    def on_ros_connect(self):
        print("Connected to ROS MASTER")
        self.setup_subscribers()

    def setup_subscribers(self):
        if not self.ros_client.is_connected:
            print("Not possible to subscribe in the topic.")
            return

        self.image_subscriber = Image_Topic(client=self.ros_client)
        self.gas_co2_subscriber = CO2_Topic(client=self.ros_client)
        self.gas_ch4_subscriber = CH4_Topic(client=self.ros_client)
        self.robot_command_publisher = Commands_Topic(client=self.ros_client)

    # close connection
    def on_ros_close(self, *args):
        print("Closing connection with ROS Bridge")

        if self.image_subscriber:
            self.image_subscriber.unsubscribe()

        if self.gas_co2_subscriber:
            self.gas_co2_subscriber.unsubscribe()

        if self.gas_ch4_subscriber:
            self.gas_ch4_subscriber.unsubscribe()

        if self.robot_command_publisher:
            self.robot_command_publisher.unadvertise()

    def on_ros_error(self, error_message):
        print(f"Failed to connect: {error_message}")
