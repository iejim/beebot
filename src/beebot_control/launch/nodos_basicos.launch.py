"""Inicia los nodos básicos como ejemplo."""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with a component."""

    # Definir los ejecutables a correr
    nodo_beebot_cmd = Node(
        package='beebot_control',
        executable='nodo_beebot',
        arguments=[
            
        ],
        output='screen',
    )

    nodo_gamepad_cmd = Node(
        package='beebot_control',
        executable='nodo_gamepad',
        arguments=[
            
        ],
        output='screen',
    )

    nodo_pca_steppers_cmd =    Node(
        package='beebot_control',
        executable='interfaz_pca',
        name='interfaz_pca_steppers',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'direccion_i2c': 0x40}
        ],
        remappings=[
                ('/comando_pca', 'comando_pca_steppers')
        ]
    )

    nodo_pca_pwm_cmd =  Node(
        package='beebot_control',
        executable='interfaz_pca',
        name='interfaz_pca_pwm',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'direccion_i2c': 0x41}
        ],
        remappings=[
                ('/comando_pca', 'comando_pca_pwm')
        ]
    )

    # start_rqt_gui = Node(
    #     package='rqt_gui',
    #     executable='rqt_gui',
    #     arguments=[
            
    #     ],
    #     output='screen',
    # )

    # Inicializar lanzador
    ld = LaunchDescription()

    ###### Agregar ejecutables al lanzador

    # Nodo de ejemplo
    # ld.add_action(nodo_beebot_cmd) 

    # Nodo para recibir mensajes del gamepad
    ld.add_action(nodo_gamepad_cmd)

    # Nodo para comandar el PCA de los steppers
    ld.add_action(nodo_pca_steppers_cmd)

    # Nodo para comandar el PCA de los servos y motores
    ld.add_action(nodo_pca_pwm_cmd)

    # ld.add_action(start_rqt_gui)
    

    # Otras opciones para el lanzador

 
    return ld

