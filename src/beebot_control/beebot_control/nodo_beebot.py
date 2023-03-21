# -*- coding: utf-8 -*-

"""Nodo ejemplo que publica y suscribe"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32,Int32

class NodoBeebot(Node):

    # Constructor de un Nodo, lo registra con un nombre, 
    # se registra para publicar
    def __init__(self, nombre=None):
        if nombre is None:
            nombre = "nodo_beebot"
        super().__init__(nombre)
        self.nombre = nombre

        ### PUBLICADORES PARA ENVIAR MENSAJES ### 
        self.pub_avanzar = self.create_publisher(
            Float32, # Tipo de mensaje
            'avanzar', # Nombre del /topico (canal) donde publicaremos
            10)

        ### SUBSCRIPTORES PARA RECIBIR MENSAJES ###
        # Monitorean si llega un mensaje y llama un callback para procesarlo
        self.sub_pasos = self.create_subscription(
            Int32, # Tipo de mensaje
            'pasos', # Nombre del /topico (canal) donde buscaremos los mensajes
            self.callback_trabajo, # función callback para manejar el mensaje
            1)

    # Se corre cada vez que llega un mensaje
    # ("reacciona" a los mensajes)
    def callback_trabajo(self, msg):
        pasos = msg.data
        
        msg_out = Float32()
        msg_out.data = 100.0*pasos
        self.pub_avanzar.publish(msg_out)
        self.logger('Avanzando a: "%s"' % msg_out.data)

    #### Ejemplo de un callback para ejecutar en un timer ###
    # def timer_callback(self):
        # '''Este ejemplo publica un mensaje cada cierto tiempo, según el timer.'''
        # Crea el mensaje
        #msg = String()   
        # Guarda un string en el mensaje
        #msg.data = 'Hello World' 
        # Publica el mensaje
        #self.publisher_.publish(msg)  
        # Anota la acción
        #self.get_logger().info('Publishing: "%s"' % msg.data) 

    def logger(self, texto):
        self.get_logger().info(texto)



# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    nodo = NodoBeebot("avanza") 

    # En este caso solo espera a que llegue un mensaje y reaccionar
    try:
        rclpy.spin(nodo) 
    except KeyboardInterrupt:
        pass 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    try:
        nodo.destroy_node() # Buena práctica
    except rclpy.handle.InvalidHandle:
        pass
    rclpy.shutdown() # Desconecta el prorama de ROS antes de cerrar


# Llama a main() si se ejecuta este archivo como un programa (nodo)
if __name__ == '__main__':
    main()
