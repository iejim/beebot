# -*- coding: utf-8 -*-

# Nodo de ROS2 para procesar y leer los botones del gamepad.
import rclpy
from rclpy.node import Node

from time import sleep
from .recursos import * # Para ver lo que se importa, ver recursos/__init__.py
from beebot_msgs.msg import Gamepad
from std_msgs.msg import String

# Se debería conectar a los gamepads tipo
#vID = 0x046d  # Vendor ID
#pID = 0xc216  # Product ID
#pID = 0xc21d # Nuevos

MID_VAL_ABS = 128
L = "L"
R = "R"

DICT_VALORES_VACIO = {
    X: 0,
    Y: 0,
    A: 0,
    B: 0,

    L: 0,
    R: 0,

    BACK: 0,
    START: 0,

    LX: 0,
    LY: 0,
    RX: 0,
    RY: 0,

    PADX: 0,
    PADY: 0
  }

class GamepadControlNode(Node):

  _nombre_nodo = "nodo_gamepad_control"
  _topico_gamepad = "gamepad_control"
  _topico_status_gamepad = "status_gamepad"

  _topico_control_remoto = "control_remoto"
  
  _status_read_timeout = 10


  valores = DICT_VALORES_VACIO

  actualizado = False
  leido =  False

  _synced = False
  _px = 0
  _py = 0

  def __init__(self, nombre = None):
    if not (nombre is None):
      self._nombre_nodo = nombre

    super().__init__(nombre)
    
    self._sub = self.create_subscription(Gamepad, self._topico_gamepad, self.callback_gamepad,1)
    self._sub_control_status = self.create_subscription(String,self._topico_status_gamepad, self.callback_gamepad_status, 1)
    

    self._timer_status = self.create_timer(self._status_read_timeout, self.activar_control_timeout)

    # self._status_msg = String()
    # self._status_msg.data = "UP"

  def logger(self, texto):
        self.get_logger().info(texto)

  
  def callback_gamepad(self,datos_gamepad):
    """Procesa los mensajes llegados del control remoto."""

    # En esta función se puede monitorear cuál botón se está presionando
    # y enviar la información al sitio/nodo adecuado (ver casos de uso).

    self.logger(datos_gamepad)
    # recoger la informacion, convertirla y enviarla

    self.leer_LR("L", datos_gamepad)
    self.leer_LR("R", datos_gamepad)
    
    self.leer_letras(datos_gamepad)

    self.leer_ejes(datos_gamepad)

    # No está definida (ver abajo)
    # self.leer_flechas(datos_gamepad) 

    self.actualizado = True

    # Ahora toca analizar los valores de los botones y enviarlos
    # a donde tengan que ir, tal vez a un nodo de telecomando
    # 
    # Por ejemplo, para saber que estamos controlando el drivetrain
    # (se presiona R2)
    # if (self.valores[R]>0): 
    #     self.calcular_velocidades()
    #     ....
    

    # Últimamente, decidir a donde se quiere publicar

    

  def callback_gamepad_status(self,msg):
    """Si llega un mensaje, reinicia el timer"""
    if (msg.data == "UP"):
      self._synced = True # Se usa para monitorear conexión
      self._timer_status.reset() # Debería reiniciar el timer

  def activar_control_timeout(self):
    """Si no llega un mensaje en 10s, solicita apagar los motores"""
    self._synced = False
    self.logger("Se dejó de recibir info dede el gamepad.")
    # Enviar mensaje de cero al control de velocidad
    # blah blah


  def leer_letras(self, msg):
    letra = chr(msg.letter)

    if (letra == "N"):
      return

    self.valores[letra] = 1

    self.actualizado = True

  def leer_LR(self,lado, msg):

    if(lado == "R"):
      val = msg.r 
    if(lado == "L"):
      val = msg.l 

    if (val > 0 and val < 4):
      self.valores[lado] = -val # Negativos para indicar R1 (-1), R3(-3)
    elif(val>3):
      self.valores[lado] = val  # Se presionó R2, que da un valor [0-255]

    self.actualizado = True

  def leer_ejes(self, msg):

    self.valores[LX] = msg.lx
    self.valores[LY] = msg.ly
    self.valores[RX] = msg.rx
    self.valores[RY] = msg.ry

    self.actualizado = True


  def leer_flechas(self, msg):
    pass

  def valor(self, boton):
    # self.leido =  True
    return self.valores[boton]

  def byte_a_porciento(self, num):
    out_max = 100.0
    out_min = -100.0
    in_max = 255.0 -1.0 # Para tener un cero
    in_min = 0.0
    val = (num - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return max(min(val, out_max), out_min) # Limitar al rango



# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    nodo = GamepadControlNode("nodo_gamepad_control") 

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