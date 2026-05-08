#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mensajes_personalizados.msg import ComandoBase


class NodoControlBaseMovil(Node):
    def __init__(self):
        super().__init__('nodo_control_base_movil')

        self.sub_comando = self.create_subscription(
            ComandoBase,
            '/control/comando_base',
            self.callback_comando,
            10
        )

        self.get_logger().info('Nodo de control de la base móvil iniciado correctamente')

    def callback_comando(self, msg):
        ul = float(msg.ul)
        um = float(msg.um)
        wz = float(msg.wz)

        self.get_logger().info(
            f'Comando base -> ul: {ul:.3f} m/s, um: {um:.3f} m/s, wz: {wz:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    nodo = None

    try:
        nodo = NodoControlBaseMovil()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
