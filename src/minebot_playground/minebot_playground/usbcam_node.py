import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import CompressedImage  # Import the Image message type from sensor_msgs

FPS = 20

class CameraNode(Node):
    def __init__(self):
        super().__init__('usbcam_node')  # Initialize the Node with the name 'webcam_node'
        self.publisher_ = self.create_publisher(CompressedImage, 'usbcam_node/compressed', 10)  # Create a publisher for the Image topic with queue size of 10
        #self.timer = self.create_timer(0.001, self.timer_callback)  # Create a timer to call timer_callback (30 FPS)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Open the specified webcam
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set the buffer size to 1 to minimize latency
        if not self.cap.isOpened():
            self.get_logger().error("Failed to capture image in index 0")
            rclpy.shutdown()
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set the frame width to 640 pixels
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set the frame height to 480 pixels
        
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Set the frames per second to 20
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) #compress/encode data to avoid timeout
        self.bridge = CvBridge()  # Initialize the CvBridge to convert between ROS and OpenCV images

        #mocking to publish every time, while doest not have a trigger
        timer_period = 1.0 / FPS  # period to 20 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)#define a timer to call the funcion every 0.05sec 

    #function that is called to send a frame
    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture a frame from the webcam
        self.get_logger().info(f"Debug; cam: {self.cap}, frame: {frame}")
        if ret:  # Check if the frame was captured successfully
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')#compact and encode the message
            self.publisher_.publish(msg)  # Publish the Image message
            #cv2.imshow('Webcam', frame)  # Display the frame in an OpenCV window - should uncoment the constructor
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Check if the 'q' key is pressed
                rclpy.shutdown()  # Shut down the ROS 2 node
        else:
            self.get_logger().error('Failed to capture image')  # Log an error message if the frame was not captured

def main():
    rclpy.init()  
    node = CameraNode()  # Create an instance of the CameraNode with the specified camera index
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.cap.release()  # Release the webcam
        cv2.destroyAllWindows()  # Close any OpenCV windows
        node.destroy_node()  # Destroy the ROS 2 node
        rclpy.shutdown()  # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    print(cv2.__version__)
    main()  # Run the main function if this script is executed
