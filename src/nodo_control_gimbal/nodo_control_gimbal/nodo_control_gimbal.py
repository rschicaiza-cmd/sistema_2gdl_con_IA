#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ============================================================
# IMPORTACION DE LIBRERIAS
# ============================================================

# ctypes se usa para convertir enteros con signo a uint32,
# necesario cuando enviamos velocidades negativas al Dynamixel.
import ctypes

# math se usa para conversiones entre grados, radianes y ticks.
import math

# time se usa para medir tiempos, por ejemplo el watchdog de comandos.
import time

# rclpy es la librería principal para crear nodos en ROS 2 con Python.
import rclpy

# Node es la clase base para crear un nodo ROS 2.
from rclpy.node import Node

# PortHandler y PacketHandler son clases del Dynamixel SDK.
# PortHandler maneja el puerto serial.
# PacketHandler maneja los paquetes de comunicación Dynamixel.
from dynamixel_sdk import PortHandler, PacketHandler

# JointState es un mensaje estándar de ROS para publicar posición,
# velocidad y esfuerzo de articulaciones.
from sensor_msgs.msg import JointState

# Mensaje personalizado que recibe comandos de velocidad para yaw y pitch.
from mensajes_personalizados.msg import ComandoGimbal


# ============================================================
# CONFIGURACION GENERAL
# ============================================================

# Puerto donde está conectado el U2D2 o adaptador USB-Dynamixel.
# En Linux normalmente es /dev/ttyUSB0.
# Si cambia, revisar con: ls /dev/ttyUSB*
DEVICENAME = '/dev/ttyUSB0'

# Baudrate configurado en los motores Dynamixel.
BAUDRATE = 57600

# Los XL430 usan protocolo 2.0.
PROTOCOL_VERSION = 2.0

# ID del motor que controla yaw.
ID_YAW = 1

# ID del motor que controla pitch.
ID_PITCH = 2

# Conversión de velocidad Dynamixel a rad/s.
# En XL430:
# 1 unidad de velocidad = 0.229 rpm.
# 0.229 rpm ≈ 0.0239808239 rad/s.
RAD_S_PER_UNIT = 0.0239808239

# Conversión de ticks de posición a radianes.
# Una vuelta completa tiene 4096 ticks.
# Una vuelta completa son 2*pi radianes.
RAD_PER_POS_UNIT = 2.0 * math.pi / 4096.0

# Conversión de grados a ticks.
# 4096 ticks = 360 grados.
TICKS_PER_DEG = 4096.0 / 360.0


# ============================================================
# SENTIDO DE MOTORES
# ============================================================
# Estos signos permiten invertir el sentido lógico de cada motor.
#
# Si mandas velocidad positiva y el eje se mueve en dirección contraria
# a la que tú consideras positiva, cambia el signo a -1.
#
# Por ejemplo:
# MOTOR_SIGN_PITCH = -1
#
# Déjalos en 1 si todo gira correctamente.

MOTOR_SIGN_YAW = 1
MOTOR_SIGN_PITCH = 1


# ============================================================
# CALIBRACION DEL CERO RAW
# ============================================================
# Estos valores representan el centro mecánico del gimbal.
#
# Procedimiento:
# 1. Coloca físicamente el gimbal en una posición central segura.
# 2. Ejecuta el nodo.
# 3. Mira en consola los valores RAW ABS de yaw y pitch.
# 4. Copia esos valores aquí.
#
# Así el código trabajará con posiciones relativas al centro.
#
# Ejemplo:
# Si en el centro mecánico el motor yaw lee 3480,
# entonces:
# ZERO_POS_YAW = 3480

ZERO_POS_YAW = 0
ZERO_POS_PITCH = 0


# ============================================================
# LIMITES ARTICULARES EN RAW RELATIVO AL CERO
# ============================================================
# Estos límites se usan para proteger el gimbal.
# Están expresados en ticks relativos al cero mecánico.
#
# NO pongas estos límites justo donde el mecanismo choca físicamente.
# Deja siempre margen antes del tope físico.
#
# yaw:   -110° a 110°
# pitch: -30° a 60°
#
# Si pitch tiene menos recorrido físico, reduce estos valores.

YAW_MIN_RAW = int(round(-110.0 * TICKS_PER_DEG))
YAW_MAX_RAW = int(round(110.0 * TICKS_PER_DEG))

PITCH_MIN_RAW = int(round(-30.0 * TICKS_PER_DEG))
PITCH_MAX_RAW = int(round(60.0 * TICKS_PER_DEG))


# ============================================================
# PROTECCIONES DE VELOCIDAD
# ============================================================
# Velocidad máxima permitida para yaw.
# Aunque MATLAB mande más velocidad, aquí se limita.
MAX_VEL_YAW_RAD_S = 0.35

# Velocidad máxima permitida para pitch.
# Pitch está más protegido porque suele tener más riesgo mecánico.
MAX_VEL_PITCH_RAD_S = 0.12

# Watchdog de comandos.
# Si no llega ningún comando durante este tiempo, los motores se detienen.
COMMAND_TIMEOUT_S = 0.25

# Periodo del lazo de control.
# 0.01 s = 100 Hz.
# Mientras más pequeño, más rápido reacciona el límite por software.
CONTROL_DT = 0.01

# Periodo de publicación del estado.
# 0.05 s = 20 Hz.
STATE_PUBLISH_DT = 0.05

# Aceleración máxima por software para yaw.
# Evita cambios bruscos de velocidad.
MAX_ACCEL_YAW_RAD_S2 = 0.8

# Aceleración máxima por software para pitch.
# Más baja para proteger pitch.
MAX_ACCEL_PITCH_RAD_S2 = 0.25


# ============================================================
# MARGENES DE SEGURIDAD
# ============================================================
# STOP_MARGIN:
# Si el eje está dentro de este margen antes del límite,
# se bloquea cualquier movimiento hacia ese límite.
#
# SLOW_ZONE:
# Antes de llegar al margen de parada, se reduce la velocidad
# progresivamente.
#
# Pitch tiene márgenes más grandes por seguridad.

# Margen de parada para yaw: 4 grados.
YAW_STOP_MARGIN_RAW = int(round(4.0 * TICKS_PER_DEG))

# Zona lenta para yaw: 18 grados antes del límite.
YAW_SLOW_ZONE_RAW = int(round(18.0 * TICKS_PER_DEG))

# Margen de parada para pitch: 7 grados.
PITCH_STOP_MARGIN_RAW = int(round(7.0 * TICKS_PER_DEG))

# Zona lenta para pitch: 22 grados antes del límite.
PITCH_SLOW_ZONE_RAW = int(round(22.0 * TICKS_PER_DEG))

# Margen extra para detectar que yaw está demasiado fuera de rango.
YAW_HARD_MARGIN_RAW = int(round(2.0 * TICKS_PER_DEG))

# Margen extra para detectar que pitch está demasiado fuera de rango.
PITCH_HARD_MARGIN_RAW = int(round(2.0 * TICKS_PER_DEG))

# Máximo salto permitido entre dos lecturas de yaw.
# Si la posición cambia demasiado de golpe, puede ser una lectura mala,
# un reinicio de referencia o un problema de comunicación.
MAX_RAW_JUMP_YAW = int(round(40.0 * TICKS_PER_DEG))

# Máximo salto permitido entre dos lecturas de pitch.
# Es menor porque pitch se protege más.
MAX_RAW_JUMP_PITCH = int(round(25.0 * TICKS_PER_DEG))

# Si ocurre un fallo grave, por defecto NO se apaga torque.
# Motivo: si pitch sostiene peso, apagar torque puede hacer que caiga.
#
# False = detener motores mandando velocidad 0, pero mantener torque.
# True  = detener motores y apagar torque.
DISABLE_TORQUE_ON_FATAL_FAULT = False


# ============================================================
# REGISTROS DYNAMIXEL XL430
# ============================================================
# Direcciones de la tabla de control del XL430.

# Registro para cambiar modo de operación.
ADDR_OPERATING_MODE = 11

# Registro para activar/desactivar torque.
ADDR_TORQUE_ENABLE = 64

# Registro donde se escribe la velocidad objetivo.
ADDR_GOAL_VELOCITY = 104

# Registro de aceleración de perfil.
ADDR_PROFILE_ACCEL = 108

# Registro donde se lee la velocidad actual.
ADDR_PRESENT_VELOCITY = 128

# Registro donde se lee la posición actual.
ADDR_PRESENT_POSITION = 132

# Valor para activar torque.
TORQUE_ENABLE = 1

# Valor para desactivar torque.
TORQUE_DISABLE = 0

# Modo velocidad en Dynamixel.
VELOCITY_MODE = 1


# ============================================================
# CLASE PRINCIPAL DEL NODO
# ============================================================

class NodoControlGimbal(Node):
    def __init__(self):
        # Inicializa el nodo ROS 2 con este nombre.
        super().__init__('nodo_control_gimbal_seguro')

        # ====================================================
        # VARIABLES DE COMANDO
        # ====================================================

        # Última velocidad deseada para yaw recibida desde MATLAB.
        self.velocidad_yaw_cmd = 0.0

        # Última velocidad deseada para pitch recibida desde MATLAB.
        self.velocidad_pitch_cmd = 0.0

        # Guarda el tiempo en que llegó el último comando.
        # Se usa para el watchdog.
        self.last_command_time = time.monotonic()

        # ====================================================
        # VARIABLES DE ESTADO
        # ====================================================

        # Posición actual yaw en RAW relativo al cero.
        self.raw_yaw_actual = 0

        # Posición actual pitch en RAW relativo al cero.
        self.raw_pitch_actual = 0

        # Lectura anterior de yaw.
        # Se usa para detectar saltos sospechosos.
        self.raw_yaw_anterior = None

        # Lectura anterior de pitch.
        self.raw_pitch_anterior = None

        # Velocidad actual de yaw en rad/s.
        self.yaw_vel_actual_rad_s = 0.0

        # Velocidad actual de pitch en rad/s.
        self.pitch_vel_actual_rad_s = 0.0

        # Última velocidad enviada a yaw.
        # Se usa para limitar aceleración.
        self.last_yaw_sent_rad_s = 0.0

        # Última velocidad enviada a pitch.
        # Se usa para limitar aceleración.
        self.last_pitch_sent_rad_s = 0.0

        # Indica si ya se leyó correctamente el estado del gimbal.
        self.estado_gimbal_valido = False

        # Indica si el sistema entró en fallo de seguridad.
        self.fallo_seguridad = False

        # Mensaje descriptivo del fallo activo.
        self.mensaje_fallo = ''

        # Contador para imprimir estado cada cierto tiempo.
        self.contador_estado = 0

        # Contador de fallos consecutivos de lectura/comunicación.
        self.contador_fallos_lectura = 0

        # ====================================================
        # COMUNICACION DYNAMIXEL
        # ====================================================

        # Crea el manejador del puerto serial.
        self.portHandler = PortHandler(DEVICENAME)

        # Crea el manejador de paquetes para protocolo 2.0.
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Abre el puerto serial.
        if not self.portHandler.openPort():
            raise RuntimeError(f'No se pudo abrir el puerto {DEVICENAME}')

        # Configura el baudrate.
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError(f'No se pudo configurar baudrate {BAUDRATE}')

        # Mensaje informativo.
        self.get_logger().info(f'Puerto abierto en {DEVICENAME} a {BAUDRATE} bps')

        # Configura yaw en modo velocidad.
        self.configure_velocity_mode(ID_YAW)

        # Configura pitch en modo velocidad.
        self.configure_velocity_mode(ID_PITCH)

        # ====================================================
        # SUSCRIPTOR ROS 2
        # ====================================================

        # Suscribe al tópico de comandos.
        # Recibe velocidades deseadas para yaw y pitch.
        self.sub_comando = self.create_subscription(
            ComandoGimbal,
            '/control/comando_gimbal',
            self.callback_comando_gimbal,
            10
        )

        # ====================================================
        # PUBLICADOR ROS 2
        # ====================================================

        # Publica el estado del gimbal como JointState.
        self.pub_estado = self.create_publisher(
            JointState,
            '/gimbal/estado',
            10
        )

        # ====================================================
        # TIMERS
        # ====================================================

        # Timer del lazo de control.
        # Lee estado, aplica límites y manda velocidades.
        self.timer_control = self.create_timer(CONTROL_DT, self.control_loop)

        # Timer de publicación de estado.
        self.timer_estado = self.create_timer(STATE_PUBLISH_DT, self.publish_state)

        # Mensajes iniciales.
        self.get_logger().info('Nodo control gimbal seguro listo.')

        self.get_logger().info(
            f'Límites RAW: '
            f'yaw=[{YAW_MIN_RAW}, {YAW_MAX_RAW}], '
            f'pitch=[{PITCH_MIN_RAW}, {PITCH_MAX_RAW}]'
        )

        self.get_logger().warn(
            'Verifica ZERO_POS_YAW y ZERO_POS_PITCH antes de probar con torque. '
            'Si el cero está mal, los límites también estarán mal.'
        )

    # ============================================================
    # CALLBACK ROS 2
    # ============================================================

    def callback_comando_gimbal(self, msg: ComandoGimbal):
        """
        Esta función se ejecuta cada vez que llega un mensaje al tópico
        /control/comando_gimbal.

        Guarda las velocidades deseadas para yaw y pitch.
        """

        # Guarda comando de yaw.
        self.velocidad_yaw_cmd = float(msg.velocidad_yaw)

        # Guarda comando de pitch.
        self.velocidad_pitch_cmd = float(msg.velocidad_pitch)

        # Actualiza tiempo del último comando recibido.
        self.last_command_time = time.monotonic()

    # ============================================================
    # UTILIDADES BASICAS
    # ============================================================

    def clamp(self, x, xmin, xmax):
        """
        Limita x para que quede dentro del rango [xmin, xmax].
        """
        return max(xmin, min(x, xmax))

    def to_signed_32(self, value):
        """
        Convierte un entero uint32 leído del Dynamixel a int32 con signo.

        Esto es necesario porque el SDK puede devolver valores sin signo,
        pero posición y velocidad pueden ser negativas.
        """
        if value >= 2**31:
            return value - 2**32
        return value

    def raw_to_rad(self, raw_relative):
        """
        Convierte ticks relativos a radianes.

        Se usa para publicar JointState, porque JointState normalmente
        espera posiciones en radianes.
        """
        return raw_relative * RAD_PER_POS_UNIT

    def raw_to_deg(self, raw_relative):
        """
        Convierte ticks relativos a grados.

        Solo se usa para imprimir información entendible en consola.
        """
        return raw_relative * 360.0 / 4096.0

    def rad_s_to_dxl(self, omega_rad_s):
        """
        Convierte rad/s a unidades internas de velocidad Dynamixel.
        """
        return int(round(omega_rad_s / RAD_S_PER_UNIT))

    def dxl_velocity_to_rad_s(self, raw_value):
        """
        Convierte velocidad leída desde el Dynamixel a rad/s.
        """
        signed = self.to_signed_32(raw_value)
        return signed * RAD_S_PER_UNIT

    def limit_rate(self, desired, previous, max_accel, dt):
        """
        Limita la variación de velocidad entre ciclos.

        Esto evita cambios bruscos:
        - Si el comando salta de 0 a mucho valor, se sube poco a poco.
        - Si baja de golpe, también se reduce suavemente.
        """

        # Máximo cambio permitido en este ciclo.
        max_delta = max_accel * dt

        # Diferencia entre velocidad deseada y velocidad enviada anterior.
        delta = desired - previous

        # Si la diferencia positiva es muy grande, limitar.
        if delta > max_delta:
            return previous + max_delta

        # Si la diferencia negativa es muy grande, limitar.
        if delta < -max_delta:
            return previous - max_delta

        # Si está dentro del límite, permitir valor deseado.
        return desired

    # ============================================================
    # FUNCIONES DE ESCRITURA DYNAMIXEL
    # ============================================================

    def write1(self, dxl_id, addr, value):
        """
        Escribe 1 byte en un registro del Dynamixel.
        """

        comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            dxl_id,
            addr,
            value
        )

        # Verifica error de comunicación.
        if comm_result != 0:
            raise RuntimeError(
                f'Error comunicación ID {dxl_id}: '
                f'{self.packetHandler.getTxRxResult(comm_result)}'
            )

        # Verifica error reportado por el servo.
        if dxl_error != 0:
            raise RuntimeError(
                f'Error servo ID {dxl_id}: '
                f'{self.packetHandler.getRxPacketError(dxl_error)}'
            )

    def write4(self, dxl_id, addr, value_signed):
        """
        Escribe 4 bytes en un registro del Dynamixel.

        Se usa para Goal Velocity, Profile Acceleration, etc.

        Si value_signed es negativo, se convierte a uint32,
        porque el SDK escribe valores de 4 bytes sin signo.
        """

        # Convierte entero con signo a uint32.
        value_u32 = ctypes.c_uint32(value_signed).value

        comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler,
            dxl_id,
            addr,
            value_u32
        )

        # Error de comunicación.
        if comm_result != 0:
            raise RuntimeError(
                f'Error comunicación ID {dxl_id}: '
                f'{self.packetHandler.getTxRxResult(comm_result)}'
            )

        # Error interno del Dynamixel.
        if dxl_error != 0:
            raise RuntimeError(
                f'Error servo ID {dxl_id}: '
                f'{self.packetHandler.getRxPacketError(dxl_error)}'
            )

    def read4(self, dxl_id, addr):
        """
        Lee 4 bytes desde un registro del Dynamixel.
        """

        value, comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler,
            dxl_id,
            addr
        )

        # Error de comunicación.
        if comm_result != 0:
            raise RuntimeError(
                f'Error lectura ID {dxl_id}: '
                f'{self.packetHandler.getTxRxResult(comm_result)}'
            )

        # Error interno del motor.
        if dxl_error != 0:
            raise RuntimeError(
                f'Error lectura servo ID {dxl_id}: '
                f'{self.packetHandler.getRxPacketError(dxl_error)}'
            )

        return value

    # ============================================================
    # CONFIGURACION DE MOTORES
    # ============================================================

    def configure_velocity_mode(self, dxl_id):
        """
        Configura un motor Dynamixel en modo velocidad.

        Importante:
        Para cambiar Operating Mode se debe apagar torque primero.
        """

        # Apaga torque antes de cambiar modo.
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Cambia a modo velocidad.
        self.write1(dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)

        # Configura aceleración interna del perfil.
        # Esto ayuda a suavizar, pero no reemplaza la rampa por software.
        self.write4(dxl_id, ADDR_PROFILE_ACCEL, 10)

        # Activa torque nuevamente.
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(f'Motor ID {dxl_id} configurado en modo velocidad')

    # ============================================================
    # LECTURA DE ESTADO
    # ============================================================

    def leer_estado_motores(self):
        """
        Lee posición y velocidad de ambos motores.

        Actualiza:
        - raw_yaw_actual
        - raw_pitch_actual
        - yaw_vel_actual_rad_s
        - pitch_vel_actual_rad_s
        """

        # Lee velocidad cruda de yaw.
        raw_vel_yaw_u32 = self.read4(ID_YAW, ADDR_PRESENT_VELOCITY)

        # Lee velocidad cruda de pitch.
        raw_vel_pitch_u32 = self.read4(ID_PITCH, ADDR_PRESENT_VELOCITY)

        # Lee posición cruda de yaw.
        raw_pos_yaw_u32 = self.read4(ID_YAW, ADDR_PRESENT_POSITION)

        # Lee posición cruda de pitch.
        raw_pos_pitch_u32 = self.read4(ID_PITCH, ADDR_PRESENT_POSITION)

        # Convierte posición yaw a entero con signo.
        raw_pos_yaw_abs = self.to_signed_32(raw_pos_yaw_u32)

        # Convierte posición pitch a entero con signo.
        raw_pos_pitch_abs = self.to_signed_32(raw_pos_pitch_u32)

        # Calcula yaw relativo al cero mecánico.
        raw_yaw_rel = raw_pos_yaw_abs - ZERO_POS_YAW

        # Calcula pitch relativo al cero mecánico.
        raw_pitch_rel = raw_pos_pitch_abs - ZERO_POS_PITCH

        # Convierte velocidad yaw a rad/s.
        yaw_vel = self.dxl_velocity_to_rad_s(raw_vel_yaw_u32)

        # Convierte velocidad pitch a rad/s.
        pitch_vel = self.dxl_velocity_to_rad_s(raw_vel_pitch_u32)

        # Verifica si hubo un salto raro en posición.
        self.validar_salto_raw(raw_yaw_rel, raw_pitch_rel)

        # Guarda posición actual yaw.
        self.raw_yaw_actual = raw_yaw_rel

        # Guarda posición actual pitch.
        self.raw_pitch_actual = raw_pitch_rel

        # Guarda velocidad actual yaw.
        self.yaw_vel_actual_rad_s = yaw_vel

        # Guarda velocidad actual pitch.
        self.pitch_vel_actual_rad_s = pitch_vel

        # Guarda lectura anterior yaw para próxima comparación.
        self.raw_yaw_anterior = raw_yaw_rel

        # Guarda lectura anterior pitch para próxima comparación.
        self.raw_pitch_anterior = raw_pitch_rel

        # Marca estado como válido.
        self.estado_gimbal_valido = True

        # Reinicia contador de fallos porque esta lectura fue exitosa.
        self.contador_fallos_lectura = 0

    def validar_salto_raw(self, raw_yaw, raw_pitch):
        """
        Detecta saltos sospechosos en posición RAW.

        Esto protege contra:
        - reinicios de referencia,
        - lecturas corruptas,
        - cambios inesperados de modo,
        - errores fuertes de comunicación.
        """

        # Si ya existe una lectura anterior de yaw, comparar.
        if self.raw_yaw_anterior is not None:
            salto_yaw = abs(raw_yaw - self.raw_yaw_anterior)

            # Si el salto es demasiado grande, activar fallo.
            if salto_yaw > MAX_RAW_JUMP_YAW:
                self.activar_fallo(
                    f'Salto RAW sospechoso en yaw: {salto_yaw} ticks'
                )

        # Si ya existe una lectura anterior de pitch, comparar.
        if self.raw_pitch_anterior is not None:
            salto_pitch = abs(raw_pitch - self.raw_pitch_anterior)

            # Si el salto es demasiado grande, activar fallo.
            if salto_pitch > MAX_RAW_JUMP_PITCH:
                self.activar_fallo(
                    f'Salto RAW sospechoso en pitch: {salto_pitch} ticks'
                )

    # ============================================================
    # PROTECCION DE LIMITES
    # ============================================================

    def aplicar_limite_eje(
        self,
        nombre,
        raw_actual,
        vel_cmd,
        raw_min,
        raw_max,
        stop_margin,
        slow_zone
    ):
        """
        Aplica límite de seguridad para un eje.

        vel_cmd > 0 significa que el RAW aumenta.
        vel_cmd < 0 significa que el RAW disminuye.

        Protecciones:
        1. Si ya está fuera del límite, solo deja volver.
        2. Si está dentro del margen de parada, bloquea.
        3. Si está dentro de la zona lenta, reduce velocidad.
        """

        # Si ya está en/fuera del límite superior.
        if raw_actual >= raw_max:

            # Si intenta seguir aumentando, bloquear.
            if vel_cmd > 0.0:
                self.get_logger().warn(
                    f'{nombre}: fuera/en límite superior RAW={raw_actual}. '
                    f'Bloqueando movimiento positivo.'
                )
                return 0.0

            # Si se mueve hacia adentro, permitir.
            return vel_cmd

        # Si ya está en/fuera del límite inferior.
        if raw_actual <= raw_min:

            # Si intenta seguir disminuyendo, bloquear.
            if vel_cmd < 0.0:
                self.get_logger().warn(
                    f'{nombre}: fuera/en límite inferior RAW={raw_actual}. '
                    f'Bloqueando movimiento negativo.'
                )
                return 0.0

            # Si se mueve hacia adentro, permitir.
            return vel_cmd

        # Distancia al límite superior.
        dist_to_max = raw_max - raw_actual

        # Si la velocidad va hacia el límite superior.
        if vel_cmd > 0.0:

            # Si está demasiado cerca del límite, bloquear.
            if dist_to_max <= stop_margin:
                self.get_logger().warn(
                    f'{nombre}: margen de parada superior. RAW={raw_actual}.'
                )
                return 0.0

            # Si está en zona lenta, escalar velocidad.
            if dist_to_max <= slow_zone:
                scale = (dist_to_max - stop_margin) / max(1.0, slow_zone - stop_margin)
                scale = self.clamp(scale, 0.0, 1.0)
                vel_cmd *= scale

        # Distancia al límite inferior.
        dist_to_min = raw_actual - raw_min

        # Si la velocidad va hacia el límite inferior.
        if vel_cmd < 0.0:

            # Si está demasiado cerca del límite, bloquear.
            if dist_to_min <= stop_margin:
                self.get_logger().warn(
                    f'{nombre}: margen de parada inferior. RAW={raw_actual}.'
                )
                return 0.0

            # Si está en zona lenta, escalar velocidad.
            if dist_to_min <= slow_zone:
                scale = (dist_to_min - stop_margin) / max(1.0, slow_zone - stop_margin)
                scale = self.clamp(scale, 0.0, 1.0)
                vel_cmd *= scale

        # Devuelve velocidad segura.
        return vel_cmd

    def aplicar_limites_articulares(self, yaw_vel, pitch_vel):
        """
        Aplica todos los límites articulares al comando de velocidad.

        Esta función es una de las más importantes para seguridad.
        """

        # Si todavía no se tiene estado válido, no permitir movimiento.
        if not self.estado_gimbal_valido:
            return 0.0, 0.0

        # Si hay fallo activo, no permitir movimiento.
        if self.fallo_seguridad:
            return 0.0, 0.0

        # Saturar velocidad máxima de yaw.
        yaw_vel = self.clamp(
            yaw_vel,
            -MAX_VEL_YAW_RAD_S,
            MAX_VEL_YAW_RAD_S
        )

        # Saturar velocidad máxima de pitch.
        pitch_vel = self.clamp(
            pitch_vel,
            -MAX_VEL_PITCH_RAD_S,
            MAX_VEL_PITCH_RAD_S
        )

        # Convertir velocidad lógica a velocidad real de motor.
        yaw_vel_motor = yaw_vel * MOTOR_SIGN_YAW
        pitch_vel_motor = pitch_vel * MOTOR_SIGN_PITCH

        # Aplicar límites RAW a yaw.
        yaw_vel_motor = self.aplicar_limite_eje(
            'Yaw',
            self.raw_yaw_actual,
            yaw_vel_motor,
            YAW_MIN_RAW,
            YAW_MAX_RAW,
            YAW_STOP_MARGIN_RAW,
            YAW_SLOW_ZONE_RAW
        )

        # Aplicar límites RAW a pitch.
        pitch_vel_motor = self.aplicar_limite_eje(
            'Pitch',
            self.raw_pitch_actual,
            pitch_vel_motor,
            PITCH_MIN_RAW,
            PITCH_MAX_RAW,
            PITCH_STOP_MARGIN_RAW,
            PITCH_SLOW_ZONE_RAW
        )

        # Detecta si pitch está cerca del límite superior.
        pitch_near_top = self.raw_pitch_actual >= (
            PITCH_MAX_RAW - PITCH_SLOW_ZONE_RAW
        )

        # Detecta si pitch está cerca del límite inferior.
        pitch_near_bottom = self.raw_pitch_actual <= (
            PITCH_MIN_RAW + PITCH_SLOW_ZONE_RAW
        )

        # Protección extra para pitch:
        # si está cerca de cualquier límite, se fuerza velocidad muy pequeña.
        if pitch_near_top or pitch_near_bottom:
            pitch_vel_motor = self.clamp(
                pitch_vel_motor,
                -0.04,
                0.04
            )

        # Convertir de vuelta a convención lógica.
        yaw_vel = yaw_vel_motor * MOTOR_SIGN_YAW
        pitch_vel = pitch_vel_motor * MOTOR_SIGN_PITCH

        return yaw_vel, pitch_vel

    # ============================================================
    # FALLOS DE SEGURIDAD
    # ============================================================

    def activar_fallo(self, mensaje):
        """
        Activa un fallo de seguridad.

        Cuando esto pasa:
        - se marca fallo_seguridad = True,
        - se detienen los motores,
        - opcionalmente se puede apagar torque.
        """

        # Marca fallo activo.
        self.fallo_seguridad = True

        # Guarda descripción del fallo.
        self.mensaje_fallo = mensaje

        # Invalida el estado actual.
        self.estado_gimbal_valido = False

        # Imprime error.
        self.get_logger().error(f'FALLO DE SEGURIDAD: {mensaje}')

        # Detiene motores mandando velocidad 0.
        self.stop_motors()

        # Si está habilitado, apaga torque.
        if DISABLE_TORQUE_ON_FATAL_FAULT:
            self.disable_torque()

    def revisar_fallo_duro_limites(self):
        """
        Revisa si algún eje está demasiado fuera de sus límites.

        Esto es una protección adicional.
        """

        # Verifica yaw fuera de rango duro.
        yaw_fuera = (
            self.raw_yaw_actual < (YAW_MIN_RAW - YAW_HARD_MARGIN_RAW) or
            self.raw_yaw_actual > (YAW_MAX_RAW + YAW_HARD_MARGIN_RAW)
        )

        # Verifica pitch fuera de rango duro.
        pitch_fuera = (
            self.raw_pitch_actual < (PITCH_MIN_RAW - PITCH_HARD_MARGIN_RAW) or
            self.raw_pitch_actual > (PITCH_MAX_RAW + PITCH_HARD_MARGIN_RAW)
        )

        # Si yaw está fuera de límite duro, activar fallo.
        if yaw_fuera:
            self.activar_fallo(
                f'Yaw fuera de límite duro. RAW={self.raw_yaw_actual}'
            )

        # Si pitch está fuera de límite duro, activar fallo.
        if pitch_fuera:
            self.activar_fallo(
                f'Pitch fuera de límite duro. RAW={self.raw_pitch_actual}'
            )

    # ============================================================
    # LOOP DE CONTROL
    # ============================================================

    def control_loop(self):
        """
        Lazo principal de control.

        En cada ciclo:
        1. Lee estado actual de motores.
        2. Revisa límites duros.
        3. Revisa watchdog de comandos.
        4. Aplica límites articulares.
        5. Aplica rampa de aceleración.
        6. Vuelve a aplicar límites.
        7. Envía velocidades a los motores.
        """

        try:
            # Leer posición y velocidad justo antes de mandar comandos.
            self.leer_estado_motores()

            # Revisar si algún eje está fuera de límites duros.
            self.revisar_fallo_duro_limites()

            # Si hay fallo activo, detener y salir.
            if self.fallo_seguridad:
                self.stop_motors()
                return

            # Tiempo transcurrido desde el último comando recibido.
            tiempo_sin_comando = time.monotonic() - self.last_command_time

            # Si MATLAB dejó de mandar comandos, parar motores.
            if tiempo_sin_comando > COMMAND_TIMEOUT_S:
                yaw_vel = 0.0
                pitch_vel = 0.0

            # Si hay comando reciente, usarlo.
            else:
                yaw_vel = self.velocidad_yaw_cmd
                pitch_vel = self.velocidad_pitch_cmd

            # Aplicar límites de velocidad y posición.
            yaw_vel, pitch_vel = self.aplicar_limites_articulares(
                yaw_vel,
                pitch_vel
            )

            # Aplicar rampa de aceleración para yaw.
            yaw_vel = self.limit_rate(
                yaw_vel,
                self.last_yaw_sent_rad_s,
                MAX_ACCEL_YAW_RAD_S2,
                CONTROL_DT
            )

            # Aplicar rampa de aceleración para pitch.
            pitch_vel = self.limit_rate(
                pitch_vel,
                self.last_pitch_sent_rad_s,
                MAX_ACCEL_PITCH_RAD_S2,
                CONTROL_DT
            )

            # Aplicar límites otra vez después de la rampa.
            # Esto evita que la rampa mantenga una velocidad prohibida.
            yaw_vel, pitch_vel = self.aplicar_limites_articulares(
                yaw_vel,
                pitch_vel
            )

            # Guardar última velocidad enviada para próxima rampa.
            self.last_yaw_sent_rad_s = yaw_vel
            self.last_pitch_sent_rad_s = pitch_vel

            # Convertir velocidad yaw a unidades Dynamixel.
            yaw_cmd = self.rad_s_to_dxl(yaw_vel * MOTOR_SIGN_YAW)

            # Convertir velocidad pitch a unidades Dynamixel.
            pitch_cmd = self.rad_s_to_dxl(pitch_vel * MOTOR_SIGN_PITCH)

            # Enviar comando al motor yaw.
            self.write4(ID_YAW, ADDR_GOAL_VELOCITY, yaw_cmd)

            # Enviar comando al motor pitch.
            self.write4(ID_PITCH, ADDR_GOAL_VELOCITY, pitch_cmd)

        except Exception as e:
            # Si algo falla en el control, contar fallo.
            self.contador_fallos_lectura += 1

            # Invalidar estado.
            self.estado_gimbal_valido = False

            # Imprimir error.
            self.get_logger().error(f'Error en control_loop: {e}')

            # Detener motores por seguridad.
            self.stop_motors()

            # Si hay demasiados fallos seguidos, activar fallo permanente.
            if self.contador_fallos_lectura >= 5:
                self.activar_fallo(
                    'Demasiados fallos consecutivos de lectura/comunicación'
                )

    # ============================================================
    # PUBLICACION DE ESTADO
    # ============================================================

    def publish_state(self):
        """
        Publica el estado actual del gimbal.

        Aunque los límites trabajan con RAW/ticks,
        JointState se publica en radianes para mantener estándar ROS.
        """

        try:
            # Convertir yaw RAW a radianes.
            q1_rad = self.raw_to_rad(self.raw_yaw_actual)

            # Convertir pitch RAW a radianes.
            q2_rad = self.raw_to_rad(self.raw_pitch_actual)

            # Crear mensaje JointState.
            msg = JointState()

            # Agregar timestamp actual.
            msg.header.stamp = self.get_clock().now().to_msg()

            # Nombres de las articulaciones.
            msg.name = ['yaw_joint', 'pitch_joint']

            # Posiciones en radianes.
            msg.position = [q1_rad, q2_rad]

            # Velocidades en rad/s.
            msg.velocity = [
                self.yaw_vel_actual_rad_s,
                self.pitch_vel_actual_rad_s
            ]

            # No se publica esfuerzo.
            msg.effort = []

            # Publicar mensaje.
            self.pub_estado.publish(msg)

            # Aumentar contador para imprimir cada cierto tiempo.
            self.contador_estado += 1

            # Imprimir estado aproximadamente cada 1 segundo.
            if self.contador_estado >= int(1.0 / STATE_PUBLISH_DT):
                self.contador_estado = 0

                self.get_logger().info(
                    f'RAW REL yaw={self.raw_yaw_actual} '
                    f'pitch={self.raw_pitch_actual} | '
                    f'DEG yaw={self.raw_to_deg(self.raw_yaw_actual):.2f} '
                    f'pitch={self.raw_to_deg(self.raw_pitch_actual):.2f} | '
                    f'cmd_sent yaw={self.last_yaw_sent_rad_s:.3f} rad/s '
                    f'pitch={self.last_pitch_sent_rad_s:.3f} rad/s | '
                    f'fallo={self.fallo_seguridad}'
                )

                # Si hay fallo activo, imprimir mensaje.
                if self.fallo_seguridad:
                    self.get_logger().error(
                        f'Fallo activo: {self.mensaje_fallo}'
                    )

        except Exception as e:
            self.get_logger().error(f'Error publicando estado: {e}')

    # ============================================================
    # SEGURIDAD
    # ============================================================

    def stop_motors(self):
        """
        Detiene ambos motores enviando Goal Velocity = 0.
        """

        try:
            # Detener yaw.
            self.write4(ID_YAW, ADDR_GOAL_VELOCITY, 0)

            # Detener pitch.
            self.write4(ID_PITCH, ADDR_GOAL_VELOCITY, 0)

            # Reiniciar velocidades enviadas.
            self.last_yaw_sent_rad_s = 0.0
            self.last_pitch_sent_rad_s = 0.0

        except Exception as e:
            self.get_logger().error(f'Error al detener motores: {e}')

    def disable_torque(self):
        """
        Deshabilita torque de ambos motores.

        Cuidado:
        si pitch sostiene peso, apagar torque puede hacer que el eje caiga.
        """

        try:
            # Apagar torque yaw.
            self.write1(ID_YAW, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

            # Apagar torque pitch.
            self.write1(ID_PITCH, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        except Exception as e:
            self.get_logger().error(f'Error al deshabilitar torque: {e}')

    def destroy_node(self):
        """
        Cierre seguro del nodo.

        Se ejecuta cuando el nodo termina.
        """

        self.get_logger().info('Deteniendo nodo y motores...')

        # Detener motores.
        self.stop_motors()

        # Apagar torque al cerrar.
        # Si pitch cae por gravedad, puedes comentar esta línea.
        self.disable_torque()

        # Cerrar puerto serial.
        try:
            self.portHandler.closePort()
        except Exception:
            pass

        # Destruir nodo ROS.
        super().destroy_node()


# ============================================================
# FUNCION PRINCIPAL
# ============================================================

def main(args=None):
    """
    Función principal del programa.
    Inicializa ROS 2, crea el nodo y lo mantiene corriendo.
    """

    # Inicializa ROS 2.
    rclpy.init(args=args)

    # Variable para guardar el nodo.
    node = None

    try:
        # Crear nodo.
        node = NodoControlGimbal()

        # Mantener nodo corriendo.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Permite cerrar con Ctrl+C sin mostrar error.
        pass

    except Exception as e:
        # Imprime error fatal si ocurre.
        print(f'Error fatal: {e}')

    finally:
        # Si el nodo existe, destruirlo de forma segura.
        if node is not None:
            node.destroy_node()

        # Apagar ROS 2 si sigue activo.
        if rclpy.ok():
            rclpy.shutdown()


# ============================================================
# PUNTO DE ENTRADA
# ============================================================

# Esto hace que main() se ejecute solo si corres este archivo directamente.
if __name__ == '__main__':
    main()
