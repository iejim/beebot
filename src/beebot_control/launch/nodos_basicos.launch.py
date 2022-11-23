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

    # start_rqt_gui = Node(
    #     package='rqt_gui',
    #     executable='rqt_gui',
    #     arguments=[
            
    #     ],
    #     output='screen',
    # )

    # Inicializar lanzador
    ld = LaunchDescription()

    # Agregar ejecutables al lanzador
    ld.add_action(nodo_beebot_cmd)
    ld.add_action(nodo_gamepad_cmd)
    # ld.add_action(start_rqt_gui)

    # Otras opciones para el lanzador

    return ld

