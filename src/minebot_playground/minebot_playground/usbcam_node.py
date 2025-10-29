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
        
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set the frame width to 640 pixels
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set the frame height to 480 pixels
        
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
    main()  # Run the main function if this script is executed



import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from picamera2 import Picamera2
from sensor_msgs.msg import CompressedImage
import time

class CameraPublisher(Node):
    def _init_(self):
        super()._init_('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw', 10)

        self.get_logger().info('Iniciando Picamera2...')
        try:
            self.picam2 = Picamera2()
            # Configura para visualização (mais rápido)
            self.config = self.picam2.create_video_configuration(main={"size": (640, 480)})
            self.picam2.configure(self.config)
            self.picam2.start()
            self.get_logger().info('Câmera iniciada com sucesso.')
        except Exception as e:
            self.get_logger().error(f'Falha ao iniciar câmera: {e}')
            # Tenta fechar
            try: self.picam2.close()
            except: pass
            rclpy.shutdown()
            return

        self.bridge = CvBridge()

        # Cria um timer para capturar e publicar imagens
        self.timer = self.create_timer(0.05, self.timer_callback) # ~20 FPS

    def timer_callback(self):
        try:
            # Captura o frame como um array numpy
            im = self.picam2.capture_array()

            # Converte o array (BGR) para uma mensagem ROS
            # Nota: picamera2 captura em BGR por padrão, o que é ótimo para o ROS
            ros_image = self.bridge.cv2_to_compressed_imgmsg(im, dst_format='jpeg')#compact and encode the message

            # Adiciona o carimbo de tempo
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera"

            # Publica a imagem
            self.publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().warn(f'Falha ao capturar/publicar frame: {e}')

    def destroy_node(self):
        # Garante que a câmera será desligada
        self.get_logger().info('Desligando câmera...')
        self.picam2.stop()
        self.picam2.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    if rclpy.ok():
        try:
            rclpy.spin(camera_publisher)
        except KeyboardInterrupt:
            pass
        finally:
            # Destroi o nó e desliga a câmera
            camera_publisher.destroy_node()
            rclpy.shutdown()

if _name_ == '_main_':
    main()