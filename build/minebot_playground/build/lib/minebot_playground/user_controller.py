# mqtt_listener_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.hivemq.com"
MQTT_TOPIC_CMD = "my_robot/cmd_vel"

class MQTTListenerNode(Node):
    def __init__(self):
        super().__init__('mqtt_listener_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(MQTT_BROKER, 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(MQTT_TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload)
        twist = Twist()
        twist.linear.x = data.get("linear_x", 0.0)
        twist.angular.z = data.get("angular_z", 0.0)
        self.pub.publish(twist)
        self.get_logger().info(f"Received command: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
