#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2


class ClickControlNode(Node):
    """
    Node ROS2, który:
    - subskrybuje obraz z kamery (/image_raw),
    - wyświetla go w oknie OpenCV,
    - reaguje na kliknięcia myszą:
        * klik powyżej środka obrazu -> robot jedzie do przodu,
        * klik poniżej środka -> robot jedzie do tyłu,
    - steruje robotem publikując Twist na /cmd_vel.
    """

    def __init__(self):
        super().__init__('click_control_node')

        # Konwerter ROS Image <-> OpenCV
        self.bridge = CvBridge()

        # Subskrypcja obrazu z kamery
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',          # zmień, jeśli Twoja kamera publikuje na innym topicu
            self.image_callback,
            10
        )

        # Publisher komend prędkości robota
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Bieżąca klatka obrazu (żeby callback myszki miał dostęp)
        self.current_frame = None

        # Nazwa okna OpenCV
        self.window_name = 'Camera view - click to move robot'

        # Tworzymy okno + podpinamy callback myszki
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info(
            'ClickControlNode uruchomiony. '
            'Kliknij w oknie obrazu: powyżej środka = jedź do przodu, poniżej = do tyłu.'
        )

    def image_callback(self, msg: Image):
        """
        Wywoływany przy każdym obrazie z kamery.
        Zamienia wiadomość ROS -> obraz OpenCV i pokazuje w oknie.
        """
        try:
            # Konwersja ROS Image -> OpenCV (format BGR)
            frame = self.bridge.to_cv2(msg, desired_encoding='bgr8')
            self.current_frame = frame

            # Wyświetlenie
            cv2.imshow(self.window_name, frame)
            # waitKey(1) pozwala GUI odświeżyć okno i obsłużyć eventy
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Błąd w image_callback: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        """
        Callback myszki z OpenCV.
        Interesuje nas kliknięcie lewym przyciskiem (EVENT_LBUTTONDOWN).
        """
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.current_frame is None:
            self.get_logger().warn('Brak klatki obrazu – jeszcze nie przyszły dane z kamery.')
            return

        height, width, _ = self.current_frame.shape
        middle_y = height // 2

        self.get_logger().info(f'Kliknięcie w (x={x}, y={y}), środek_y={middle_y}')

        twist = Twist()

        if y < middle_y:
            # klik powyżej środka -> do przodu
            twist.linear.x = 0.2  # m/s
            self.get_logger().info('Polecenie: jedź do przodu')
        else:
            # klik poniżej środka -> do tyłu
            twist.linear.x = -0.2
            self.get_logger().info('Polecenie: jedź do tyłu')

        self.cmd_vel_pub.publish(twist)

        # W wersji podstawowej pozwalamy robotowi jechać,
        # zatrzymanie możesz zrobić osobnym przyciskiem / logiką czasową.


def main(args=None):
    rclpy.init(args=args)
    node = ClickControlNode()

    try:
        # Ręczne "spinowanie" – żeby nie blokować OpenCV
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # Resztą GUI zajmuje się cv2.waitKey(1) w image_callback
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

