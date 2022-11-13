# Sistema de Control del Beebot

Implementación en ROS2 usando el Beaglebone Blue. 

Paquetes de ROS2 en el repositorio:

- beebot_control
- beebot_msgs

El paquete de nodos `beebot_control` está supuesto a correr en el Beaglebone Blue. Contiene los nodos que se ejecutarán en el mismo. 

El paquete `beebot_msgs` existe para definir los mensajes que usaremos para comunicar los diferentes nodos. 

## Sobre los paquetes.

Cada paquete tiene una estructura. El paquete `beebot_control` es un paquete de Python y tiene una carpeta interna con el mismo nombre, que es donde se ubican los archivos que servirán de nodos (y/o ejecutables). También hay una carpeta `launch` que se usa para crear "programas" que efecutan una combinación de nodos al mismo tiempo. También está el archivo `setup.py` que se debe actualizar cada vez que se hace agrega un nuevo nodo al paquete. Las demás carpetas y archivos están mas por caractísticas del paquete que para uso nuestro.


 El paquete `beebot_msgs` es realmente un paquete de C, aunque no tiene código en C, que se compila para generar las interfaces de C y Python para trabajar con los mensajes. En este paquete solo nos interesan los archivos en la carpeta `msg` que tiene nuestra definición de los mensajes que queremos usar. Así que cualquier nuevo mensaje para el repositorio debe estar aquí y luego agregarse al archivo `CMakeLists.txt` del paquete.
``
## Actualización con nuevos nodos

Cada ves que que se crea un nuevo nodo, este debe registrarse en el archivo `setup.py` del paquete. Siguiendo como ejemplo los que ya se muestran.


Tras la creación de un nuevo nodo, hay que recrear el paquete corriendo:

```Bash
colcon build --packages-select [nombre_paquete]
```

desde el directorio raíz del repositorio (donde se encuentra este archivo). Se puede omitir la opción de `--packages-select` si se quieren reconstruir todos los paquetes otra vez (no debería ser necesario).

Si el paquete que se modificó fue `beebot_control`: 

    colcon build --packages-select beebot_control


## Correr un nodo.

Para poder usar la interfaz de ROS2 en nuestros paquetes, primero debemos agregar el directorio de los paquetes al listado de búsqueda de ROS2. Por tanto debemos correr:

`. install/setup.bash`

desde el direcotio raíz del repositorio. 

A partir de este puntom para correr un nodo en específico:

```bash
$ ros2 run beebot_control [nombre_nodo]
```

También se puede correr un conjunto de nodos al mismo tiempo usando el archivo terminado en `*.launch.py` en el directorio `launch` del paquete `beebot_control`:

```bash
$ ros2 launch beebot_control nodos_basicos.launch.py
```


