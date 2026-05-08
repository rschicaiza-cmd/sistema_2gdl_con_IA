#!/usr/bin/env python3

# Librerías estándar
import math

# Librerías de visión artificial
import cv2
import numpy as np

# Librerías de ROS 2
import rclpy
from rclpy.node import Node

# Librería de la cámara ZED
import pyzed.sl as sl

# Librería de YOLOv8
from ultralytics import YOLO

# Mensaje personalizado usado para publicar la detección del líder
from mensajes_personalizados.msg import DeteccionLider


# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================

# Modelo YOLO que se va a usar.
# yolov8n.pt es la versión nano: rápida, ligera y suficiente para detección básica.
MODEL_PATH = "yolov8n.pt"

# En COCO, la clase 0 corresponde a "person".
PERSON_CLASS_ID = 0

# Umbral mínimo de confianza para aceptar una detección.
# Si YOLO detecta una persona con confianza menor a 0.40, se descarta.
CONF_THRESHOLD = 0.40


# =========================================================
# CONFIGURACIÓN DE LA CÁMARA ZED
# =========================================================

# Resolución de la cámara ZED.
# HD1080 entrega imagen de alta resolución.
ZED_RESOLUTION = sl.RESOLUTION.HD1080

# Modo de profundidad.
# ULTRA ofrece mayor calidad, aunque puede consumir más recursos.
ZED_DEPTH_MODE = sl.DEPTH_MODE.ULTRA

# Unidad de medida para las coordenadas y profundidad.
# En este caso se trabaja en centímetros.
ZED_UNIT = sl.UNIT.CENTIMETER


# =========================================================
# CONFIGURACIÓN DE ESTABILIZACIÓN
# =========================================================

# Factor de suavizado para los errores en X e Y.
# Valores más altos reaccionan más rápido, valores más bajos suavizan más.
ALPHA_XY = 0.35

# Factor de suavizado para la profundidad Z.
ALPHA_Z = 0.20


# =========================================================
# RANGO VÁLIDO DE PROFUNDIDAD
# =========================================================

# Profundidad mínima válida en centímetros.
MIN_VALID_Z_CM = 20.0

# Profundidad máxima válida en centímetros.
MAX_VALID_Z_CM = 1000.0


# =========================================================
# PERIODO DE MUESTREO
# =========================================================

# Tiempo entre ejecuciones del nodo, en segundos.
# 0.10 segundos equivale aproximadamente a 10 Hz.
SAMPLE_PERIOD = 0.10


# =========================================================
# FUNCIONES YOLO
# =========================================================

def detect_people(model, frame_bgr):
    """
    Detecta personas en una imagen usando YOLOv8.

    Parámetros:
        model: modelo YOLO cargado.
        frame_bgr: imagen de entrada en formato BGR.

    Retorna:
        Lista de detecciones. Cada detección contiene:
        - bbox: coordenadas de la caja delimitadora.
        - center: centro de la caja.
        - conf: confianza de la detección.
        - width: ancho de la caja.
        - height: alto de la caja.
    """

    detections = []

    # Ejecuta YOLO sobre el frame actual.
    # verbose=False evita imprimir información innecesaria en consola.
    results = model(frame_bgr, verbose=False)

    # Recorre los resultados entregados por YOLO.
    for result in results:

        # Si no hay cajas detectadas, se continúa con el siguiente resultado.
        if result.boxes is None:
            continue

        # Recorre cada caja detectada.
        for box in result.boxes:

            # Clase detectada.
            cls = int(box.cls[0].item())

            # Confianza de la detección.
            conf = float(box.conf[0].item())

            # Se filtran detecciones que no sean personas
            # o que tengan baja confianza.
            if cls != PERSON_CLASS_ID or conf < CONF_THRESHOLD:
                continue

            # Coordenadas de la caja en formato:
            # x1, y1: esquina superior izquierda
            # x2, y2: esquina inferior derecha
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            # Centro de la caja detectada.
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Ancho y alto de la caja.
            w = x2 - x1
            h = y2 - y1

            # Se guarda la detección en una lista.
            detections.append({
                "bbox": (x1, y1, x2, y2),
                "center": (cx, cy),
                "conf": conf,
                "width": w,
                "height": h
            })

    return detections


def choose_target(detections, cx_ref, cy_ref):
    """
    Selecciona la persona objetivo.

    En este caso, el líder se escoge como la persona más cercana
    al centro de referencia de la imagen.

    Parámetros:
        detections: lista de personas detectadas.
        cx_ref: coordenada X del centro de referencia.
        cy_ref: coordenada Y del centro de referencia.

    Retorna:
        La detección seleccionada como objetivo.
        Si no hay detecciones, retorna None.
    """

    if not detections:
        return None

    # Se elige la detección cuya distancia al centro de la imagen sea menor.
    return min(
        detections,
        key=lambda d: math.hypot(
            d["center"][0] - cx_ref,
            d["center"][1] - cy_ref
        )
    )


# =========================================================
# FUNCIONES VISUALES
# =========================================================

def draw_reference(img, cx, cy):
    """
    Dibuja una cruz de referencia en el centro óptico de la cámara.
    """

    # Línea vertical.
    cv2.line(img, (int(cx), 0), (int(cx), img.shape[0]), (0, 255, 255), 2)

    # Línea horizontal.
    cv2.line(img, (0, int(cy)), (img.shape[1], int(cy)), (0, 255, 255), 2)

    # Punto central.
    cv2.circle(img, (int(cx), int(cy)), 6, (0, 255, 255), -1)


def draw_detection(img, det, color=(0, 255, 0), label="person"):
    """
    Dibuja la caja de detección, el centro de la persona
    y la confianza del modelo.
    """

    if det is None:
        return

    x1, y1, x2, y2 = det["bbox"]
    cx, cy = det["center"]
    conf = det["conf"]

    # Caja delimitadora.
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

    # Centro de la caja detectada.
    cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)

    # Texto con etiqueta y confianza.
    cv2.putText(
        img,
        f"{label} {conf:.2f}",
        (x1, max(30, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2
    )


def draw_xy_debug(img, obj_x, obj_y, ref_x, ref_y, dx, dy):
    """
    Dibuja información visual para depurar el error en X e Y.

    obj_x, obj_y:
        Centro de la persona detectada.

    ref_x, ref_y:
        Centro de referencia de la imagen.

    dx, dy:
        Error suavizado entre el centro de la persona y el centro óptico.
    """

    # Punto del objeto detectado.
    cv2.circle(img, (int(obj_x), int(obj_y)), 8, (0, 0, 255), -1)

    # Punto de referencia.
    cv2.circle(img, (int(ref_x), int(ref_y)), 8, (0, 255, 255), -1)

    # Flecha desde el centro de referencia hacia la persona.
    cv2.arrowedLine(
        img,
        (int(ref_x), int(ref_y)),
        (int(obj_x), int(obj_y)),
        (255, 255, 0),
        3,
        tipLength=0.05
    )

    # Coordenadas del objeto.
    cv2.putText(
        img,
        f"obj=({int(obj_x)},{int(obj_y)})",
        (int(obj_x) + 10, max(30, int(obj_y) - 15)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    # Coordenadas del centro de referencia.
    cv2.putText(
        img,
        f"ref=({int(ref_x)},{int(ref_y)})",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    # Error en píxeles.
    cv2.putText(
        img,
        f"dx={dx:.1f}px dy={dy:.1f}px",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )


def clamp(value, min_value, max_value):
    """
    Limita un valor dentro de un rango.

    Se usa para evitar saltos bruscos en la profundidad.
    """

    return max(min_value, min(value, max_value))


# =========================================================
# UTILIDADES ZED
# =========================================================

def zed_mat_to_bgr(mat):
    """
    Convierte una imagen de la ZED a formato BGR compatible con OpenCV.
    """

    frame = mat.get_data()

    # Algunas imágenes de ZED vienen en BGRA, es decir, con canal alfa.
    # OpenCV trabaja normalmente en BGR.
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    return frame.copy()


def is_valid_depth(value_cm):
    """
    Verifica si una profundidad es válida.

    Descarta:
    - Valores None.
    - NaN.
    - Infinitos.
    - Profundidades fuera del rango definido.
    """

    if value_cm is None:
        return False

    if isinstance(value_cm, float):
        if math.isnan(value_cm) or math.isinf(value_cm):
            return False

    return MIN_VALID_Z_CM <= value_cm <= MAX_VALID_Z_CM


# =========================================================
# NODO ROS 2
# =========================================================

class NodoDeteccionLider(Node):
    """
    Nodo principal de detección del líder.

    Este nodo:
    - Captura imagen de la cámara ZED2.
    - Detecta personas usando YOLOv8.
    - Selecciona como líder la persona más cercana al centro de la imagen.
    - Calcula el error horizontal y vertical en píxeles.
    - Obtiene la profundidad del líder.
    - Publica los datos en el tópico /lider/deteccion.
    """

    def __init__(self):
        """
        Constructor del nodo.
        Aquí se inicializan YOLO, ZED, publicadores y temporizador.
        """

        # Nombre del nodo en ROS 2.
        super().__init__('nodo_deteccion_lider')

        # Carga del modelo YOLO.
        self.get_logger().info("Cargando YOLO...")
        self.model = YOLO(MODEL_PATH)

        # Creación del objeto cámara ZED.
        self.zed = sl.Camera()

        # Parámetros de inicialización de la cámara.
        init = sl.InitParameters()
        init.camera_resolution = ZED_RESOLUTION
        init.depth_mode = ZED_DEPTH_MODE
        init.coordinate_units = ZED_UNIT

        # Distancias mínima y máxima de profundidad para la ZED.
        # Estos valores están en metros, aunque luego se trabaja en centímetros.
        init.depth_minimum_distance = 0.2
        init.depth_maximum_distance = 10.0

        # Apertura de la cámara.
        status = self.zed.open(init)

        # Si la cámara no abre correctamente, se detiene el nodo.
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Error al abrir ZED: {status}")

        # Obtención de parámetros intrínsecos de la cámara izquierda.
        info = self.zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters

        # Parámetros intrínsecos:
        # fx, fy: distancias focales en píxeles.
        # cx, cy: centro óptico de la cámara.
        self.fx = calib.left_cam.fx
        self.fy = calib.left_cam.fy
        self.cx = calib.left_cam.cx
        self.cy = calib.left_cam.cy

        self.get_logger().info(
            f"Intrínsecos cámara izquierda: fx={self.fx:.3f}, fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, cy={self.cy:.3f}"
        )

        # Parámetros de ejecución de ZED.
        self.runtime = sl.RuntimeParameters()

        # Matrices donde se almacenan imagen, profundidad y nube de puntos.
        self.image_left = sl.Mat()
        self.depth_map = sl.Mat()
        self.point_cloud = sl.Mat()

        # Publicador ROS 2.
        # Publica mensajes personalizados DeteccionLider en /lider/deteccion.
        self.pub_deteccion = self.create_publisher(
            DeteccionLider,
            '/lider/deteccion',
            10
        )

        # Variables filtradas para estabilizar el error visual.
        self.dx_f = 0.0
        self.dy_f = 0.0

        # Profundidad filtrada.
        self.z_f = None

        # Última profundidad válida.
        self.last_valid_z = None

        # Máximo cambio permitido en profundidad por ciclo.
        # Evita saltos bruscos cuando la ZED entrega mediciones inestables.
        self.max_step_cm = 25.0

        # Ventana de visualización con OpenCV.
        cv2.namedWindow("ZED2 + YOLO Personas", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ZED2 + YOLO Personas", 1280, 720)

        # Temporizador que llama a procesar() cada SAMPLE_PERIOD segundos.
        self.timer = self.create_timer(SAMPLE_PERIOD, self.procesar)

        self.get_logger().info("Nodo de detección del líder iniciado correctamente")

    def procesar(self):
        """
        Función principal que se ejecuta periódicamente.

        En cada ciclo:
        - Captura imagen y profundidad.
        - Detecta personas.
        - Selecciona el objetivo.
        - Calcula errores dx, dy.
        - Obtiene profundidad.
        - Publica el mensaje ROS.
        - Muestra la imagen con información visual.
        """

        # Captura un nuevo frame de la ZED.
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        # Recupera la imagen izquierda.
        self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)

        # Recupera el mapa de profundidad.
        self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)

        # Recupera la nube de puntos XYZRGBA.
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

        # Convierte la imagen de la ZED a formato BGR para OpenCV y YOLO.
        frame_bgr = zed_mat_to_bgr(self.image_left)

        # Dibuja el centro de referencia de la cámara.
        draw_reference(frame_bgr, self.cx, self.cy)

        # Detecta personas en la imagen.
        dets = detect_people(self.model, frame_bgr)

        # Selecciona la persona más cercana al centro como líder.
        target = choose_target(dets, self.cx, self.cy)

        # Inicialización de variables para este ciclo.
        dx = 0.0
        dy = 0.0
        z_cm = None
        X = Y = Z = None

        # Creación del mensaje ROS.
        msg = DeteccionLider()

        # Valores por defecto cuando no hay detección.
        msg.detectado = False
        msg.id = -1
        msg.u = 0.0
        msg.v = 0.0
        msg.z = 0.0
        msg.dx = 0.0
        msg.dy = 0.0

        # Si existe una persona objetivo, se calculan sus datos.
        if target is not None:

            # Centro de la persona detectada.
            px, py = target["center"]

            # Error entre el centro de la persona y el centro óptico.
            # dx positivo: persona hacia la derecha.
            # dx negativo: persona hacia la izquierda.
            # dy positivo: persona hacia abajo.
            # dy negativo: persona hacia arriba.
            dx = float(px - self.cx)
            dy = float(py - self.cy)

            # Filtro exponencial para suavizar dx y dy.
            self.dx_f = ALPHA_XY * dx + (1.0 - ALPHA_XY) * self.dx_f
            self.dy_f = ALPHA_XY * dy + (1.0 - ALPHA_XY) * self.dy_f

            # Lee la profundidad en el píxel central de la persona.
            err_depth, depth_value = self.depth_map.get_value(px, py)

            # Verifica que la lectura de profundidad sea correcta y válida.
            if err_depth == sl.ERROR_CODE.SUCCESS and is_valid_depth(float(depth_value)):

                # Profundidad medida por la ZED en centímetros.
                z_candidate = float(depth_value)

                # Si no existe una profundidad previa válida, se toma directamente.
                if self.last_valid_z is None:
                    z_cm = z_candidate
                else:
                    # Diferencia respecto a la última profundidad válida.
                    delta = z_candidate - self.last_valid_z

                    # Se limita el cambio máximo permitido.
                    delta = clamp(delta, -self.max_step_cm, self.max_step_cm)

                    # Nueva profundidad limitada.
                    z_cm = self.last_valid_z + delta

                # Filtro exponencial para suavizar la profundidad.
                if self.z_f is None:
                    self.z_f = z_cm
                else:
                    self.z_f = ALPHA_Z * z_cm + (1.0 - ALPHA_Z) * self.z_f

                # Se guarda la profundidad filtrada como última profundidad válida.
                self.last_valid_z = self.z_f

            # Obtiene coordenadas 3D desde la nube de puntos.
            # X, Y, Z están en centímetros porque se configuró ZED_UNIT como CENTIMETER.
            err_pc, point = self.point_cloud.get_value(px, py)

            if err_pc == sl.ERROR_CODE.SUCCESS:
                X = float(point[0])
                Y = float(point[1])
                Z = float(point[2])

                # Se descartan valores inválidos.
                if math.isnan(X) or math.isinf(X):
                    X = None

                if math.isnan(Y) or math.isinf(Y):
                    Y = None

                if math.isnan(Z) or math.isinf(Z):
                    Z = None

            # Dibuja la caja de la persona detectada.
            draw_detection(frame_bgr, target, (0, 255, 0), "person")

            # Dibuja información visual de dx y dy.
            draw_xy_debug(frame_bgr, px, py, self.cx, self.cy, self.dx_f, self.dy_f)

            # Rellena el mensaje ROS con la detección.
            msg.detectado = True
            msg.id = 1

            # Coordenadas del centro de la persona en píxeles.
            msg.u = float(px)
            msg.v = float(py)

            # Profundidad publicada en metros.
            # self.z_f está en centímetros, por eso se divide entre 100.
            msg.z = float(self.z_f / 100.0) if self.z_f is not None else 0.0

            # Error visual filtrado en píxeles.
            msg.dx = float(self.dx_f)
            msg.dy = float(self.dy_f)

        # Publica el mensaje, haya detección o no.
        self.pub_deteccion.publish(msg)

        # Variable auxiliar para mostrar profundidad en pantalla.
        if self.z_f is not None and self.last_valid_z is not None:
            z_show = self.z_f
        else:
            z_show = None

        # Texto principal mostrado en la imagen.
        if target is not None:
            if z_show is not None:
                info_txt = f"DX:{self.dx_f:.0f}px DY:{self.dy_f:.0f}px Z:{z_show:.1f}cm"
            else:
                info_txt = f"DX:{self.dx_f:.0f}px DY:{self.dy_f:.0f}px Z:---"
        else:
            info_txt = "No detection"

        # Muestra dx, dy y profundidad.
        cv2.putText(
            frame_bgr,
            info_txt,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        # Si las coordenadas 3D son válidas, se muestran en pantalla.
        if X is not None and Y is not None and Z is not None:
            cv2.putText(
                frame_bgr,
                f"X:{X:.1f}cm Y:{Y:.1f}cm Z:{Z:.1f}cm",
                (20, frame_bgr.shape[0] - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

        # Mensaje para cerrar la ventana.
        cv2.putText(
            frame_bgr,
            "Press q to quit",
            (20, frame_bgr.shape[0] - 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        # Muestra la imagen procesada.
        cv2.imshow("ZED2 + YOLO Personas", frame_bgr)

        # Necesario para que OpenCV actualice la ventana.
        cv2.waitKey(1)

    def destroy_node(self):
        """
        Cierra recursos antes de destruir el nodo.
        """

        try:
            # Cierra la cámara ZED.
            self.zed.close()
        except Exception:
            pass

        # Cierra ventanas de OpenCV.
        cv2.destroyAllWindows()

        # Llama al destructor original de Node.
        super().destroy_node()


def main(args=None):
    """
    Función principal del ejecutable ROS 2.
    """

    # Inicializa ROS 2.
    rclpy.init(args=args)

    nodo = None

    try:
        # Crea el nodo.
        nodo = NodoDeteccionLider()

        # Mantiene el nodo activo.
        rclpy.spin(nodo)

    except KeyboardInterrupt:
        # Permite cerrar el nodo con CTRL+C.
        pass

    finally:
        # Destruye el nodo correctamente.
        if nodo is not None:
            nodo.destroy_node()

        # Apaga ROS 2 si sigue activo.
        if rclpy.ok():
            rclpy.shutdown()


# Punto de entrada del script.
if __name__ == "__main__":
    main()
