# -*- coding: utf-8 -*-
"""Nodo de ejemplo de como hacer un publicador"""
# Como el publicado en https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

# Líneas base para crear un nodo de ROS2

# rclpy contiene todas las funciones para interactuar con ROS2
import rclpy
from rclpy.node import Node

# Hay que importar cualquier tipo de mensaje que vayamos a 
# recibir/enviar por medio de los tópicos (suscribiendo/publicando)
from std_msgs.msg import String

# Para acceder a las bondades de ROS, usamos la clase Node como base para una clase propia
class SimplePublisher(Node):

    # Constructor de un Nodo, lo registra con un nombre, 
    # se registra para publicar, y crea un timer para sus acciones.
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # Callback que se llama cada vez que termine el timer.
    # Como el nodo solo publica, necesita el timer para hacer 
    # algo útil dentro del loop infinito de ejecución del programa.
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    simple_publisher = SimplePublisher() 

    # Usa spin para ejecutar infinitamente nuestro nodo
    rclpy.spin(simple_publisher) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_publisher.destroy_node() # Buena práctica
    rclpy.shutdown() # Desconecta el prorama de ROS antes de cerrar


# Llama a main() si se ejecuta este archivo como un programa (nodo)
if __name__ == '__main__':
    main()
