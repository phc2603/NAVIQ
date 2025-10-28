import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading


from subscriber_topics.Image_Topic import Image_Topic
from subscriber_topics.CO2_Topic import CO2_Topic
from subscriber_topics.CH4_Topic import CH4_Topic
from publisher_topics.Commands_Topic import Commands_Topic


class Ros_Topics:
    def __init__(self):
        rclpy.init(args=None)

        #initiate each node
        self.gas_ch4_subscriber = CH4_Topic()
        self.gas_co2_subscriber = CO2_Topic()
        self.image_subscriber = Image_Topic()
        self.robot_command_publisher = Commands_Topic()

        self.executor = None

        self.initiate_multi_thread_nodes()

    #initiate nodes
    def initiate_multi_thread_nodes(self):
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.gas_ch4_subscriber)
        self.executor.add_node(self.gas_co2_subscriber)
        self.executor.add_node(self.image_subscriber)
        self.executor.add_node(self.robot_command_publisher)

    #spin in a separate thread
    def spin_in_thread(self):
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    #shutdown topics
    def shutdown(self):
        self.image_subscriber.destroy_node()
        self.gas_co2_subscriber.destroy_node()
        self.gas_ch4_subscriber.destroy_node()
        self.robot_command_publisher.destroy_node()
        rclpy.shutdown()

