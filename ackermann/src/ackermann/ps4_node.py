import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import subprocess
import time
import cv2

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # Iniciar el nodo joy_node usando subprocess
        self.start_joy_node()

        # Suscribirse al topic '/joy' que publica los datos del mando
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # Publicar en el topic '/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscribirse al topic '/cam/image_raw' para recibir imágenes
        self.image_subscription = self.create_subscription(
            Image,
            '/cam/image_raw',
            self.image_callback,
            10)

        self.cv_bridge = CvBridge()

        self.get_logger().info("Joy to CmdVel Node started.")

    def start_joy_node(self):
        # Ejecutar joy_node utilizando subprocess
        try:
            # Ejecutamos joy_node en segundo plano (sin bloquear el hilo principal)
            self.get_logger().info("Starting joy_node...")
            subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])

            # Esperar un poco para asegurarse de que el joy_node haya comenzado
            time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"Failed to start joy_node: {e}")

    def joy_callback(self, msg):
        twist_msg = Twist()

        # Mapeo de los valores de los joysticks a velocidades lineales y angulares
        # Joystick izquierdo: control de movimiento lineal
        twist_msg.linear.x = msg.axes[1]*3  # Eje Y del joystick izquierdo (movimiento adelante/atrás)
        twist_msg.linear.y = msg.axes[0]*3  # Eje X del joystick izquierdo (movimiento lateral, opcional)

        # Joystick derecho: control de rotación
        twist_msg.angular.z = msg.axes[3]*2  # Eje X del joystick derecho (rotación)

        # Publicamos el mensaje en /cmd_vel
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Publishing: linear.x = {twist_msg.linear.x}, angular.z = {twist_msg.angular.z}")

    def image_callback(self, msg):
        """Callback para recibir la imagen y mostrarla en pantalla."""
        # Convertir el mensaje de imagen de ROS a una imagen de OpenCV
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Mostrar la imagen en una ventana llamada 'Frame'
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)  # Necesario para que OpenCV actualice la ventana

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
