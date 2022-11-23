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

class GcsGamepadNode(Node):

  _nombre_nodo = "nodo_gamepad"
  _topico_gamepad = "gamepad_control"
  _topico_control = "status_gamepad"

  _usb_read_time = 0.005
  _status_send_time = 5

  _pub = None
  _pub_control = None

  _dev_usb = None
  _usb_msg = None
  _old_msg = None

  _synced = False
  _px = 0
  _py = 0

  def __init__(self, nombre = None):
    if not (nombre is None):
      self._nombre_nodo = nombre

    super().__init__(nombre)

    self._pub = self.create_publisher(Gamepad, self._topico_gamepad, 2)
    self._pub_status = self.create_publisher(String,self._topico_control, 1)
    
    self._timer_usb = self.create_timer(self._usb_read_time, self.procesar_eventos_usb)

    self._timer_status = self.create_timer(self._status_send_time, self.enviar_estado_nodo)


    self._usb_msg = Gamepad()
    #valores por defecto
    self._usb_msg.lx = MID_VAL_ABS
    self._usb_msg.ly = MID_VAL_ABS
    self._usb_msg.rx = MID_VAL_ABS
    self._usb_msg.ry = MID_VAL_ABS

    self._old_msg = self._usb_msg

    self._status_msg = String()
    self._status_msg.data = "UP"

    # Preparar gamepad 
    if not self.conectarUSB():
      return

  def logger(self, texto):
        self.get_logger().info(texto)

  def enviar_estado_nodo(self):
    self.logger("Enviando estado: "+self._status_msg.data)
    self._pub_status.publish(self._status_msg)

  def conectarUSB(self):
    """Extrae los dispositivos tipo gamepad y los regristra."""
    
    
    try:
      dev = inputs.devices.gamepads[0]
    except IndexError:
      dev = False
      self.logger("No se pudo encontrar un dispositivo.")
      self.logger("Revise la conexion del Gamepad. Saliendo.")
      exit()
    
    if dev:
      self.logger("Trabajando con el dispositivo: %s." % (dev))
    else:
      return False

    self.logger("Dispositivo abierto. Listo para usar.")
    self._dev_usb = dev # TODO: confirmar que sigue existiendo
    return True
    

  def procesar_eventos_usb(self, e):
    """Process available events."""
    try:
      events = self._dev_usb.read()
    except EOFError:
      events = []
    for event in events:
      self.procesar_evento(event)
    # print('n')

  def procesar_evento(self, evt):
    # Procesar evento
    msg = self._usb_msg

    t = evt.ev_type
    cod = evt.code
    try:
      c = TIPOS_EVT[cod]
    except KeyError: # en caso de SYNC o algo asi
      c = ""
    val = evt.state
    if t == 'Key':
      self._synced = False
      if c == LETRAS: # actualizar letras
        msg.letter = self.leer_letras(cod, val)
      elif c == BOT_L: # actualizar L
        msg.l = self.leer_LR(cod, val)
      elif c == BOT_R: # actualizar R
        msg.r = self.leer_LR(cod, val)
      elif c == BOT_CONTROL:
        msg.control = self.leer_control(cod, val)
      else: # wtf
        pass
      
    elif t == 'Absolute':
      self._synced = False
      if c == BOT_L:
        # Procesar L2
        msg.l = val>>1
      elif c == BOT_R:
        # R2
        msg.r = val>>1
      elif c == A_LX:
        msg.lx = self.normalizar_eje(val)
      elif c == A_LY:
        msg.ly = self.normalizar_eje(val)
      elif c == A_RX:
        msg.rx = self.normalizar_eje(val)
      elif c == A_RY:
        msg.ry = self.normalizar_eje(val)
      elif c == FLECHAS:
        msg.arrow = self.leer_flechas(cod,val)
      else:
        pass

    elif t == 'Sync':    
      ## Ejecutar si ya llego un SYN_REPORT
      self._synced = True
      # print(msg)
      try:
        self._pub.publish(msg)
        # self.logger("Enviando %s" % repr(msg))
      except Exception as e:
        self.logger("Error enviando mensaje: %s" % e)

    else:
      return

  def leer_LR(self,cod, val):
    #Devuelve 0,1,3
    if val:
      bot = NOMBRES_EVT[cod]
      return int(bot[1]) # numero en L1 o R2
    return 0

  def leer_letras(self,cod, val):
    if val:
      return ord(NOMBRES_EVT[cod])
    return ord("N")
    
  def leer_flechas(self,cod, val):
    # Leer y completar una lectura
    if self._synced:
      n = self._px*2 + self._py*3
      if n>0:
        v = flechas[n]
        return v-1 # Mantener compatibilidad control anterior
      return 8
    else:
      # Actualizar el que toca
      if cod[-1] == 'X':
        self._px = val
      else:
        self._py = val
    return 8
      
  def leer_control(self, cod, val):
    # Opciones
    if val:
      return ord(NOMBRES_EVT[cod])
    return 0

  def normalizar_eje(self, val):
    # devuelve [0,255]
    # para obtener [-128,127], remover la suma
    return (val>>8)+128

# Método de "entrada" del archivo; lo que se ejecuta al llamarlo como programa.
def main(args=None):
    # Inicializa el programa en la computadora como un nodo de ROS
    rclpy.init(args=args) 

    # Crea una instancia de Clase que le da vida a nuestro nodo
    nodo = GcsGamepadNode("nodo_gamepad") 

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