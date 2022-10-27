#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Nodo ejemplo que publica y suscribe"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32,Int32

# Se necesitan ambos
from beebot_msgs.msg import CanalPCA, CanalesPCA

from pca9685_driver import Device

class InterfazPCA(Node):

    # Constructor de un Nodo, lo registra con un nombre, 
    # se registra para publicar, y crea un timer para sus acciones.
    def __init__(self,):

        super().__init__("interfaz_pca")

        # self.pub_avanzar = self.create_publisher(Float32, 'avanzar', 10)

        self.sub_pasos = self.create_subscription(
            CanalesPCA,
            'comando_pca',
            self.leer_comando,
            1)
        self.pca = None
        

    def inicializar_pca(self, dev=0x40, freq=None):
        # Inicializa la conexión el PCA
        self.pca = Device(dev) # TODO: Revisar si fue satisfactorio 

        if freq is None:
            self.pca.set_pwm_frequency(50)
            return

        self.pca.set_pwm_frequency(freq)
        

    # Se corre cada vez que llega un mensaje
    # ("reacciona" a los mensajes)
    def leer_comando(self,msg):
        if self.pca is None:
            # Entramos aquí si se llamó a spin() y nunca se inicializó el PCA.
            self.logger("PCA: No se inicializó el dispositivo. Saliendo.")
            self.destroy_node() # debería funcionar
            return
        #msg es de tipo CanalesPCA()
        for canal in msg.lista:
            self.enviar_pwm(canal.ch, canal.valor)

    def logger(self, texto):
        self.get_logger().info(texto)

    def enviar_pwm(self, canal, valor):
        valor = min(max(0,valor),4096) # En realidad, la librería de PCA ya hace esto

        self.pca.set_pwm(canal, valor)
        # Pudiera publicarse para poder grabarse en un bag
        self.logger("PCA: (%s, %s)" %(canal, valor))



# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    nodo = InterfazPCA() 
    
    # Inicializa el PCA después de registrar el nodo.
    # Como es una acción en un archivo, mejor después 
    # que todo está funcoinando con ROS.
    nodo.inicializar_pca() 

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
