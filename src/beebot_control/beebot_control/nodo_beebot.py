#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Nodo ejemplo que publica y suscribe"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32,Int32

class NodoBeebot(Node):

    # Constructor de un Nodo, lo registra con un nombre, 
    # se registra para publicar, y crea un timer para sus acciones.
    def __init__(self, nombre=None):
        if nombre is None:
            nombre = "nodo_beebot"
        super().__init__(nombre)
        self.nombre = nombre

        self.pub_avanzar = self.create_publisher(Float32, 'avanzar', 10)

        self.sub_pasos = self.create_subscription(
            Int32,
            'pasos',
            self.callback_trabajo,
            1)

    # Se corre cada vez que llega un mensaje
    # ("reacciona" a los mensajes)
    def callback_trabajo(self):
        msg = Float32()
        msg.data = 100.0
        self.pub_avanzar.publish(msg)
        self.logger('Avanzando a: "%s"' % msg.data)

    def logger(self, texto):
        self.get_logger().info(texto)



# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    nodo = NodoBeebot("avanza") 

    # En este caso solo espera a que llegue un mensaje y reaccionar
    rclpy.spin(nodo) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nodo.destroy_node() # Buena práctica
    rclpy.shutdown() # Desconecta el prorama de ROS antes de cerrar


# Llama a main() si se ejecuta este archivo como un programa (nodo)
if __name__ == '__main__':
    main()
