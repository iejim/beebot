# -*- coding: utf-8 -*-

"""Nodo ejemplo que publica y suscribe"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32,Int32

# Se necesitan ambos
from beebot_msgs.msg import CanalPCA, CanalesPCA

from pca9685_driver import Device, DeviceException

class InterfazPCA(Node):

    # Constructor de un Nodo, lo registra con un nombre, 
    # se registra para publicar, y crea un timer para sus acciones.
    def __init__(self,):

        super().__init__("interfaz_pca")

        # self.pub_avanzar = self.create_publisher(Float32, 'avanzar', 10)

        self.sub_pasos = self.create_subscription(
            CanalesPCA,  # Tipo de mensaje
            'comando_pca', # Nombre del /topico (canal) donde publicaremos
            self.leer_comando, # Función callback para manejar el mensaje
            1)
            
        self.pca = None

        self.declare_parameter('direccion_i2c', 0x40) # Valor por defecto de la dirección i2c

        

    def inicializar_pca(self, freq=None): # dev=None
        # Inicializa la conexión el PCA
        
        d = self.get_parameter('direccion_i2c').get_parameter_value().integer_value
        try:
            self.logger("Registrando el controlador del PCA en la dirección: {}".format(d))
            self.pca = Device(d) # TODO: Revisar si fue satisfactorio 
        except DeviceException as e:
            self.logger("Error abriendo el dispositivo I2C: {}".format(d))
            self.destroy_node() # debería evitar seguir trabajando
            return

        if freq is None:
            self.pca.set_pwm_frequency(50)
            return

        self.pca.set_pwm_frequency(freq)
        
    def get_conectado(self):
        return not self.pca is None

    # Se corre cada vez que llega un mensaje
    # ("reacciona" a los mensajes)
    def leer_comando(self,msg):
        if self.pca is None:
            # Entramos aquí si se llamó a spin() y nunca se inicializó el PCA.
            self.logger("PCA: No se inicializó el dispositivo I2C. Saliendo.")
            self.destroy_node() # debería evitar seguir trabajando
            return
        #msg es de tipo CanalesPCA()
        for canal in msg.lista:
            self.enviar_pwm(canal.ch, canal.valor)

    def logger(self, texto):
        self.get_logger().info(texto)

    def enviar_pwm(self, canal, valor):
        valor = min(max(0,valor),4096) # En realidad, la librería de PCA ya hace esto (hasta 4095)

        if valor < 4096: # Recibe un valor normal
            self.pca.set_pwm(canal, valor)
        elif valor == 0: # Es raro un cero, así que lo forzamos
            self.full_off(canal)
        else: # Si no es porque pidieron 4096, así que forzamos el ON
            self.full_on(canal) 
        # Pudiera publicarse para poder grabarse en un bag
        self.logger("PCA: (%s, %s)" %(canal, valor))
    
    def full_off(self, canal):
        self.__check_range('led_number', canal)
        registro_OFF_High = self.pca.calc_led_register(canal) #    apunta a LEDx_OFF_L
        self.pca.write(registro_OFF, 0x00) # Reinicia cualquier timer
        self.pca.write(registro_OFF + 1, 0x10) # Fuerza el pin a OFF (escriba a LEDx_OFF_H)
        self.pca.write(registro_OFF - 1, 0x00) # Deshabilita forzar el pin a ON (escriba a LEDx_ON_H)
        
    def full_on(self, canal):
        self.__check_range('led_number', canal)
        register_ON_High = self.pca.calc_led_register(canal)-1 # apunta a LEDx_ON_H
        self.pca.write(register_ON_High, 0x10) # Fuerza el pin a ON
        
    def __check_led_range(self, type, value):
        range = (0, 15)
        # range = self.ranges[type]
        if value < range[0]:
            raise DeviceException("%s debe ser mayor que %s, recibí %s" % (type, range[0], value))
        if value > range[1]:
            raise DeviceException("%s debe ser menor que %s, recibí %s" % (type, range[1], value))

        



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
    if nodo.get_conectado(): 
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
