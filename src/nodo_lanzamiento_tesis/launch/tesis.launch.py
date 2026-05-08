from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='nodo_deteccion_lider',
            executable='nodo_deteccion_lider',
            name='nodo_deteccion_lider',
            output='screen'
        ),

        Node(
            package='nodo_control_gimbal',
            executable='nodo_control_gimbal',
            name='nodo_control_gimbal',
            output='screen'
        ),

        Node(
            package='nodo_registro_datos',
            executable='nodo_registro_datos',
            name='nodo_registro_datos',
            output='screen'
        ),

    ])
