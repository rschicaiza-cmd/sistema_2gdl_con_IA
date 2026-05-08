#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from mensajes_personalizados.msg import DeteccionLider, ComandoGimbal


class NodoRegistroDatos(Node):
    def __init__(self):
        super().__init__('nodo_registro_datos')

        # =========================================================
        # CREACIÓN DEL ARCHIVO CSV
        # =========================================================

        # Carpeta donde se guardarán los datos
        self.carpeta_datos = os.path.expanduser('~/tesis_ws/datos_registrados')
        os.makedirs(self.carpeta_datos, exist_ok=True)

        # Nombre del archivo con fecha y hora
        fecha = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        self.ruta_csv = os.path.join(
            self.carpeta_datos,
            f'registro_tesis_{fecha}.csv'
        )

        # Abre el archivo CSV
        self.archivo_csv = open(self.ruta_csv, mode='w', newline='')
        self.writer = csv.writer(self.archivo_csv)

        # Encabezados del CSV
        self.writer.writerow([
            'tiempo_ros_s',

            'detectado',
            'id_lider',
            'u_px',
            'v_px',
            'z_m',
            'dx_px',
            'dy_px',

            'velocidad_yaw',
            'velocidad_pitch'
        ])

        self.archivo_csv.flush()

        # =========================================================
        # VARIABLES INTERNAS
        # =========================================================

        self.detectado = False
        self.id_lider = -1
        self.u = 0.0
        self.v = 0.0
        self.z = 0.0
        self.dx = 0.0
        self.dy = 0.0

        self.velocidad_yaw = 0.0
        self.velocidad_pitch = 0.0

        # =========================================================
        # SUSCRIPTORES
        # =========================================================

        # Datos publicados por nodo_deteccion_lider
        self.create_subscription(
            DeteccionLider,
            '/lider/deteccion',
            self.callback_deteccion,
            10
        )

        # Datos publicados por nodo_control_gimbal
        self.create_subscription(
            ComandoGimbal,
            '/control/comando_gimbal',
            self.callback_comando_gimbal,
            10
        )

        # Timer para guardar datos a frecuencia fija
        # 0.1 s = 10 Hz
        self.timer = self.create_timer(0.1, self.guardar_datos)

        self.get_logger().info('Nodo de registro de datos iniciado correctamente')
        self.get_logger().info(f'Archivo CSV: {self.ruta_csv}')

    def tiempo_ros_segundos(self):
        """
        Devuelve el tiempo actual de ROS en segundos.
        """
        now = self.get_clock().now()
        return now.nanoseconds / 1e9

    def callback_deteccion(self, msg):
        """
        Recibe los datos de detección del líder.
        """
        self.detectado = msg.detectado
        self.id_lider = msg.id
        self.u = msg.u
        self.v = msg.v
        self.z = msg.z
        self.dx = msg.dx
        self.dy = msg.dy

        self.get_logger().info(
            f'[DETECCION] detectado={msg.detectado} id={msg.id} '
            f'u={msg.u:.2f} v={msg.v:.2f} z={msg.z:.3f} '
            f'dx={msg.dx:.2f} dy={msg.dy:.2f}'
        )

    def callback_comando_gimbal(self, msg):
        """
        Recibe los comandos de velocidad del gimbal.
        """
        self.velocidad_yaw = msg.velocidad_yaw
        self.velocidad_pitch = msg.velocidad_pitch

        self.get_logger().info(
            f'[GIMBAL] yaw={msg.velocidad_yaw:.3f} '
            f'pitch={msg.velocidad_pitch:.3f}'
        )

    def guardar_datos(self):
        """
        Guarda una fila en el archivo CSV con los últimos datos recibidos.
        """
        tiempo = self.tiempo_ros_segundos()

        self.writer.writerow([
            f'{tiempo:.6f}',

            int(self.detectado),
            self.id_lider,
            f'{self.u:.3f}',
            f'{self.v:.3f}',
            f'{self.z:.6f}',
            f'{self.dx:.3f}',
            f'{self.dy:.3f}',

            f'{self.velocidad_yaw:.6f}',
            f'{self.velocidad_pitch:.6f}'
        ])

        self.archivo_csv.flush()

    def destroy_node(self):
        """
        Cierra el archivo CSV antes de apagar el nodo.
        """
        try:
            self.archivo_csv.flush()
            self.archivo_csv.close()
            self.get_logger().info(f'Archivo CSV guardado en: {self.ruta_csv}')
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = None

    try:
        nodo = NodoRegistroDatos()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
