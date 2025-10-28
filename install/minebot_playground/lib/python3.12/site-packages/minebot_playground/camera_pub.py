import rclpy, numpy as np, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import cv2
except Exception:
    cv2 = None

class CameraPub(Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # parâmetros
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 20)
        self.declare_parameter('synthetic_on_fail', True)

        self.cap = None
        if cv2 is not None:
            dev = int(self.get_parameter('device_id').value)
            self.cap = cv2.VideoCapture(dev)
            if self.cap.isOpened():
                w = int(self.get_parameter('width').value)
                h = int(self.get_parameter('height').value)
                fps = int(self.get_parameter('fps').value)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                self.cap.set(cv2.CAP_PROP_FPS, fps)
            else:
                self.cap = None

        period = 1.0 / max(1, int(self.get_parameter('fps').value)) #definindo o tempo
        self.timer = self.create_timer(period, self.tick)
        self.synthetic = (self.cap is None) and bool(self.get_parameter('synthetic_on_fail').value)
        if self.synthetic:
            self.get_logger().warn('Webcam indisponível — publicando frames sintéticos.')

    def tick(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            if not ok:
                return
        else:
            # frame sintético
            w = int(self.get_parameter('width').value)
            h = int(self.get_parameter('height').value)
            frame = np.zeros((h, w, 3), dtype=np.uint8)
            t = time.strftime('%H:%M:%S')
            txt = f'Minebot {t}'
            if cv2 is not None:
                cv2.putText(frame, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            else:
                # sem cv2, desenha uma faixa branca
                frame[:, 0:10, :] = 255

        # BGR8 -> Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            try: self.cap.release()
            except: pass
        super().destroy_node()

def main():
    rclpy.init()
    n = CameraPub()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
