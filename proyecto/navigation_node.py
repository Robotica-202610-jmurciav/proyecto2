import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo
from .logic.planner_rtt import rrt, suavizar, calcular_longitud_camino, calcular_suma_angular


# ──────────────────────────────────────────────────────────────────────────────
# Constantes del robot y de planificación
# ──────────────────────────────────────────────────────────────────────────────
ROBOT_RADIO   = 0.15   # m  – radio del robot (≈ mitad del lado 0.30 m)
VEL_LINEAL    = 0.5    # m/s
VEL_ANGULAR   = 0.2    # rad/s
TOL_ANGULAR   = 0.15   # rad ≈ 8.6°
DIST_SEGURA   = 0.10   # m  – distancia mínima al obstáculo antes de abortar
CONO_VISION   = 25     # °  – semángulo del cono de detección frontal
NUMERO_ESCENA = 1      # número de escena a ejecutar (1-6)

TOL_POSICION   = 0.15  # m  – radio de captura del waypoint
DRIFT_MAX_REPLAN = 0.25  # m


# ══════════════════════════════════════════════════════════════════════════════

class NavigationNode(Node):
    def __init__(self):
        """
        Nodo ROS2 que planifica y ejecuta un camino geométrico autónomo.
        Máquina de estados
        ──────────────────
        ESPERANDO_COMANDO → ROTANDO → MOVIENDO → (repite por segmento)
            → ROTACION_FINAL → RELOCALIZAR → DONE
        """
        super().__init__('student_navigation')
        # Suscriptores
        self.odom_sub  = self.create_subscription(Odometry,   'odom',     self.odom_callback,  10)
        self.lidar_sub = self.create_subscription(LaserScan,  'scan_raw', self.lidar_callback, 10)
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Pose actual
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.last_scan     = None
        self.odom_recibida = False

        # Estado del movimiento
        self.target_theta_abs  = None   # ángulo absoluto objetivo (rad)
        self.pose_inicial_flag = None   # posición de inicio del segmento actual

        # Datos de la misión
        self.scene_data    = None
        self.positions     = []         # [(x,y), …] waypoints en coordenadas mundo
        self.qf_theta_deg  = 0.0        # orientación final (°)
        self.path_configs  = []         # lista de (x,y,θ°) para el archivo .txt
        self.pos_idx       = 1          # índice del siguiente waypoint a alcanzar
        self.numero_escena = NUMERO_ESCENA
        self.segment_dist      = 0.0    # distancia del segmento actual (m)
        self.segment_target_xy = None   # destino (x,y) del segmento actual

        # ── MÉTRICAS DE EJECUCIÓN ────────────────────────────────────
        # Necesarias para la tabla comparativa del informe.
        self.t_inicio_ejecucion  = None   # time.perf_counter() al iniciar ROTANDO
        self.dist_recorrida_total = 0.0   # suma de distancias de segmentos completados
        self.suma_angular_abs_deg = 0.0   # suma |rotaciones| realizadas
        self._theta_antes_rotar   = None  # guarda el ángulo antes de cada rotación
        self.dt_planificacion     = 0.0   # tiempo que tardó el RRT en planificar
        # Máquina de estados
        self.state = 'ESPERANDO_COMANDO'
        # Temporizadores
        self.create_timer(0.1, self.control_loop)
        self._inicio_timer = self.create_timer(1.0, self._auto_start)
        self.get_logger().info("Proyecto 3 – Nodo RRT iniciado.")
    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACKS DE ROS2
    # ══════════════════════════════════════════════════════════════════════════
    def odom_callback(self, msg):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)

        distancia_nueva_al_origen = math.hypot(x, y)
        distancia_actual_al_origen = math.hypot(self.current_x, self.current_y)
        if distancia_nueva_al_origen < 0.05 and distancia_actual_al_origen > 0.30:
            # Lectura de "reinicio de frame": ignorar silenciosamente
            return
        # Filtro de saltos bruscos (> 0.5 m entre ticks)
        if self.odom_recibida and self.state not in ('ESPERANDO_COMANDO',):
            salto = math.hypot(x - self.current_x, y - self.current_y)
            if salto > 0.5:
                self.get_logger().warn(
                    f"Odom descartada: salto de {salto:.2f} m "
                    f"({self.current_x:.2f},{self.current_y:.2f}) → ({x:.2f},{y:.2f})",
                    throttle_duration_sec=1.0)
                return

        self.current_x     = x
        self.current_y     = y
        self.current_theta = theta
        self.odom_recibida = True 

    def lidar_callback(self, msg):
        self.last_scan = msg

    # ══════════════════════════════════════════════════════════════════════════
    # HELPERS DE MOVIMIENTO
    # ══════════════════════════════════════════════════════════════════════════

    def _rotar_a(self, theta_objetivo_rad: float) -> bool:
        """
        Gira hasta alcanzar un ángulo absoluto en el marco mundo.
        Devuelve True cuando la rotación está completa.
        CAMBIO: registra el ángulo previo la primera vez para medir la rotación.
        """
        # Guardar ángulo de inicio para contabilizar la rotación ejecutada
        if self._theta_antes_rotar is None:
            self._theta_antes_rotar = self.current_theta

        cmd, done = calcular_rotacion(
            self.current_theta, theta_objetivo_rad,
            vel_angular_max=VEL_ANGULAR, tolerancia=TOL_ANGULAR
        )
        self.cmd_pub.publish(cmd)

        if done:
            # Acumular la rotación absoluta ejecutada
            diff = theta_objetivo_rad - self._theta_antes_rotar
            diff = math.atan2(math.sin(diff), math.cos(diff))
            self.suma_angular_abs_deg += abs(math.degrees(diff))
            self._theta_antes_rotar = None   # resetear para la próxima rotación

        return done

    def _mover_adelante(self, target_xy: tuple) -> str:
        """
        Avanza hacia target_xy usando proximidad de posición como criterio
        de llegada.  Antes se medía distancia recorrida desde el
        inicio del segmento, lo que fallaba cuando la odometría tenía drift
        lateral: el robot declaraba 'COMPLETADO' pero estaba en la posición
        equivocada, causando que el siguiente segmento apuntara hacia un
        obstáculo.
        Devuelve: 'EN_RUTA' | 'COMPLETADO' | 'BLOQUEADO'
        """
        if not self.odom_recibida:
            return 'EN_RUTA'

        tx, ty = target_xy

        # ── Criterio de llegada: distancia al waypoint destino ────────────────
        dist_restante = math.hypot(self.current_x - tx, self.current_y - ty)
        if dist_restante < TOL_POSICION:
            self.dist_recorrida_total += self.segment_dist
            self.pose_inicial_flag = None
            self.cmd_pub.publish(Twist())
            return 'COMPLETADO'

        # Log de inicio de segmento
        if self.pose_inicial_flag is None:
            self.pose_inicial_flag = (self.current_x, self.current_y)
            self.get_logger().info(
                f"  Inicio segmento en ({self.current_x:.2f}, {self.current_y:.2f}) "
                f"→ ({tx:.2f}, {ty:.2f}), dist={dist_restante:.2f} m")

        # ── Verificar obstáculos al frente con LiDAR ──────────────────────────
        cono = obtener_distancias_rango(self.last_scan, -CONO_VISION, CONO_VISION)
        distancias_validas = [d for d in cono if 0 < d < float('inf')]
        dist_frente = min(distancias_validas) if distancias_validas else float('inf')

        if dist_frente < DIST_SEGURA:
            self.get_logger().warn(f"  Obstáculo a {dist_frente:.2f} m – BLOQUEADO")
            self.pose_inicial_flag = None
            self.cmd_pub.publish(Twist())
            return 'BLOQUEADO'

        # ── Avanzar con velocidad proporcional a la distancia restante ────────
        # (frena suavemente al acercarse al waypoint)
        vel = min(VEL_LINEAL, max(0.12, dist_restante * 1.2))
        ang_hacia_target = math.atan2(ty - self.current_y, tx - self.current_x)
        error_angular = ang_hacia_target - self.current_theta
        error_angular = math.atan2(math.sin(error_angular), math.cos(error_angular)) # Normalizar

        cmd = Twist()
        cmd.linear.x = vel
        cmd.angular.z = 0.5 * error_angular # Ganancia P simple para mantener el rumbo
        self.cmd_pub.publish(cmd)
        return 'EN_RUTA'

    # ══════════════════════════════════════════════════════════════════════════
    # PARSEO DE ESCENA
    # ══════════════════════════════════════════════════════════════════════════
    def _parsear_escena(self, numero: int) -> dict | None:
        share_dir = get_package_share_directory('proyecto')
        ruta = os.path.join(share_dir, 'data', f'Escena-Problema{numero}.txt')
        datos = {'obstaculos': []}
        try:
            with open(ruta, 'r', encoding='utf-8') as f:
                for linea in f:
                    partes = linea.strip().split(',')
                    if not partes or not partes[0]:
                        continue
                    k = partes[0]

                    if   k == 'Dimensiones':
                        datos['ancho'] = float(partes[1])
                        datos['alto']  = float(partes[2])
                    elif k == 'q0':
                        datos['q0'] = (float(partes[1]), float(partes[2]), float(partes[3]))
                    elif k == 'qf':
                        datos['qf'] = (float(partes[1]), float(partes[2]), float(partes[3]))
                    elif k == 'dFrente':
                        datos['dFrente']  = float(partes[1])
                    elif k == 'dDerecha':
                        datos['dDerecha'] = float(partes[1])
                    elif '_Pto1' in k:
                        n = int(k.replace('Obstaculo', '').replace('_Pto1', ''))
                        while len(datos['obstaculos']) < n:
                            datos['obstaculos'].append({})
                        datos['obstaculos'][n - 1]['p1'] = (float(partes[1]), float(partes[2]))
                    elif '_Pto2' in k:
                        n = int(k.replace('Obstaculo', '').replace('_Pto2', ''))
                        datos['obstaculos'][n - 1]['p2'] = (float(partes[1]), float(partes[2]))

            self.get_logger().info(
                f"Escena {numero} cargada: {len(datos['obstaculos'])} obstáculos, "
                f"q0={datos['q0']}, qf={datos['qf']}"
            )
            return datos

        except FileNotFoundError:
            self.get_logger().error(f"Archivo no encontrado: {ruta}")
        except Exception as e:
            self.get_logger().error(f"Error parseando escena: {e}")
        return None

    # ══════════════════════════════════════════════════════════════════════════
    # RELOCALIZACIÓN CON LiDAR
    # ══════════════════════════════════════════════════════════════════════════

    def _relocalizar(self, scene: dict, qf_teo: tuple) -> tuple | None:
        """
        Calcula la configuración real del robot a partir de lecturas LiDAR
        al frente y a la derecha en qf.
        """
        if self.last_scan is None:
            return None

        d_f = obtener_distancia_angulo(self.last_scan, 0.0)
        d_r = obtener_distancia_angulo(self.last_scan, math.radians(-90.0))

        if not (0 < d_f < float('inf')):
            d_f = scene['dFrente']
        if not (0 < d_r < float('inf')):
            d_r = scene['dDerecha']

        theta = math.radians(qf_teo[2])
        fx, fy = math.cos(theta),              math.sin(theta)
        rx, ry = math.cos(theta - math.pi/2),  math.sin(theta - math.pi/2)

        front_pt = self._raycast(scene, qf_teo[0], qf_teo[1], fx, fy)
        right_pt = self._raycast(scene, qf_teo[0], qf_teo[1], rx, ry)

        if front_pt is None or right_pt is None:
            self.get_logger().warn(
                "Relocalización: no se encontró obstáculo de referencia.")
            return None

        b_f = front_pt[0] * fx + front_pt[1] * fy
        b_r = right_pt[0] * rx + right_pt[1] * ry

        rhs1 = b_f - d_f
        rhs2 = b_r - d_r

        det = fx * ry - fy * rx
        if abs(det) < 1e-6:
            x_act = right_pt[0] - d_r * rx
            y_act = front_pt[1] - d_f * fy
        else:
            x_act = (rhs1 * ry - rhs2 * fy) / det
            y_act = (fx * rhs2 - rx * rhs1) / det

        return (x_act, y_act, qf_teo[2])

    def _raycast(self, scene: dict, ox: float, oy: float,
                 dx: float, dy: float) -> tuple | None:
        W, H   = scene['ancho'], scene['alto']
        best_t = float('inf')
        best_pt = None

        def actualizar(t, px, py):
            nonlocal best_t, best_pt
            if t > 1e-3 and t < best_t:
                best_t  = t
                best_pt = (px, py)

        if abs(dx) > 1e-9:
            for wx in (0.0, W):
                t = (wx - ox) / dx
                actualizar(t, ox + t * dx, oy + t * dy)
        if abs(dy) > 1e-9:
            for wy in (0.0, H):
                t = (wy - oy) / dy
                actualizar(t, ox + t * dx, oy + t * dy)

        for obs in scene['obstaculos']:
            x1, y1 = obs['p1']
            x2, y2 = obs['p2']
            xmin, xmax = min(x1, x2), max(x1, x2)
            ymin, ymax = min(y1, y2), max(y1, y2)

            if abs(dx) > 1e-9:
                for bx in (xmin, xmax):
                    t = (bx - ox) / dx
                    if t > 1e-3:
                        py = oy + t * dy
                        if ymin <= py <= ymax:
                            actualizar(t, ox + t * dx, py)
            if abs(dy) > 1e-9:
                for by in (ymin, ymax):
                    t = (by - oy) / dy
                    if t > 1e-3:
                        px = ox + t * dx
                        if xmin <= px <= xmax:
                            actualizar(t, px, oy + t * dy)

        return best_pt
    # ══════════════════════════════════════════════════════════════════════════
    # SALIDA A ARCHIVO
    # ══════════════════════════════════════════════════════════════════════════
    def _guardar_camino(self, configs, escena, qf_est=None, q_act=None):
        ruta = os.path.expanduser(f'~/Camino-Escena{escena}.txt')
        try:
            with open(ruta, 'w', encoding='utf-8') as f:
                f.write("# Camino geométrico solución – Proyecto 3 (RRT)\n")
                f.write("# Formato: x,y,theta_deg\n")
                for cfg in configs:
                    f.write(f"{cfg[0]:.4f},{cfg[1]:.4f},{cfg[2]:.2f}\n")
                if qf_est:
                    f.write(f"# qf_est,{qf_est[0]:.4f},{qf_est[1]:.4f},{qf_est[2]:.2f}\n")
                if q_act:
                    f.write(f"# q_act,{q_act[0]:.4f},{q_act[1]:.4f},{q_act[2]:.2f}\n")
            self.get_logger().info(f"Camino guardado → {ruta}")
        except Exception as e:
            self.get_logger().error(f"Error guardando camino: {e}")
    # ══════════════════════════════════════════════════════════════════════════
    # PUNTO DE ENTRADA DE LA MISIÓN
    # ══════════════════════════════════════════════════════════════════════════

    def ejecutar_escena(self, numero: int):
        self.numero_escena = numero

        # 1. Parsear escena
        scene = self._parsear_escena(numero)
        if scene is None:
            return
        self.scene_data = scene
        q0, qf = scene['q0'], scene['qf']

        # 2. Planificación RRT
        self.get_logger().info("Planificando con RRT . . .")
        waypoints_raw, dt_plan = rrt(
            q0_xy       = (q0[0], q0[1]),
            qf_xy       = (qf[0], qf[1]),
            scene       = scene,
            robot_radio = ROBOT_RADIO,
        )
        self.dt_planificacion = dt_plan 
        if waypoints_raw is None:
            self.get_logger().error(f"RRT no encontró solución en {dt_plan:.1f} s. Abortando.")
            return

        self.get_logger().info(
            f"RRT: {len(waypoints_raw)} waypoints crudos en {dt_plan:.3f} s.")

        # 3. Suavizado
        waypoints_suaves = suavizar(waypoints_raw, scene, ROBOT_RADIO)
        self.get_logger().info(
            f"Suavizado: {len(waypoints_raw)} → {len(waypoints_suaves)} waypoints")

        # 4. Construir lista de posiciones para la ejecución
        positions = [(q0[0], q0[1])]
        for wp in waypoints_suaves[1:-1]:
            positions.append(wp)
        positions.append((qf[0], qf[1]))
        self.positions    = positions
        self.qf_theta_deg = qf[2]

        self.get_logger().info("Waypoints planificados:")
        for i, pos in enumerate(positions):
            self.get_logger().info(f"  [{i}] x={pos[0]:.3f}, y={pos[1]:.3f}")

        # 5. Construir path_configs para el archivo .txt
        self.path_configs = []
        for i in range(len(positions) - 1):
            x1, y1 = positions[i]
            x2, y2 = positions[i + 1]
            theta_seg = math.degrees(math.atan2(y2 - y1, x2 - x1))
            self.path_configs.append((x1, y1, theta_seg))
            self.path_configs.append((x2, y2, theta_seg))
        self.path_configs.append((qf[0], qf[1], qf[2]))

        self._guardar_camino(self.path_configs, numero)

        # 6. Iniciar ejecución
        self.pos_idx = 1
        x1, y1 = self.positions[0]
        x2, y2 = self.positions[1]
        self.target_theta_abs  = math.atan2(y2 - y1, x2 - x1)
        self.segment_dist      = math.hypot(x2 - x1, y2 - y1)
        self.segment_target_xy = (x2, y2)
        self.get_logger().info(
            f"  Primer segmento: ({x1:.3f},{y1:.3f}) → ({x2:.3f},{y2:.3f}), "
            f"θ={math.degrees(self.target_theta_abs):.1f}°")
        
        self.t_inicio_ejecucion = time.perf_counter()

        self.state = 'ROTANDO'
        self.get_logger().info(
            f"Misión iniciada: {len(positions) - 1} segmentos, "
            f"distancia planificada ≈ "
            f"{sum(math.hypot(positions[i+1][0]-positions[i][0], positions[i+1][1]-positions[i][1]) for i in range(len(positions)-1)):.2f} m"
        )

    def _preparar_siguiente_segmento(self):
        """
        Calcula heading y distancia del segmento actual y guarda el waypoint
        destino para la llegada por proximidad.
        """
        if self.pos_idx >= len(self.positions):
            return

        x1, y1 = self.positions[self.pos_idx - 1]
        x2, y2 = self.positions[self.pos_idx]

        drift = math.hypot(self.current_x - x1, self.current_y - y1)
        pos_es_origen_sin_inicializar = (
            math.hypot(self.current_x, self.current_y) < 0.05 and
            math.hypot(x1, y1) > 0.30
        )
        if drift > DRIFT_MAX_REPLAN and self.odom_recibida and not pos_es_origen_sin_inicializar:
            self.get_logger().warn(
                f"  Drift de {drift:.2f} m detectado en inicio de segmento "
                f"(actual=({self.current_x:.2f},{self.current_y:.2f}), "
                f"esperado=({x1:.2f},{y1:.2f})) – replanificando.")
            self._replanificar()
            return

    # ══════════════════════════════════════════════════════════════════════════
    # BUCLE DE CONTROL PRINCIPAL
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        if self.last_scan is None:
            self.get_logger().warn("Esperando LiDAR...", throttle_duration_sec=2.0)
            return
        if self.state in ('IDLE', 'DONE', 'ESPERANDO_COMANDO'):
            return
        if self.target_theta_abs is None:
            self.get_logger().warn("target_theta_abs es None", throttle_duration_sec=2.0)
            return

        self.get_logger().info(
            f"Estado: {self.state} | "
            f"pos=({self.current_x:.2f},{self.current_y:.2f}) | "
            f"θ={math.degrees(self.current_theta):.1f}° | "
            f"objetivo={math.degrees(self.target_theta_abs):.1f}°",
            throttle_duration_sec=2.0
        )

        # ── ROTANDO ───────────────────────────────────────────────────────────
        if self.state == 'ROTANDO':
            if self._rotar_a(self.target_theta_abs):
                self.get_logger().info(
                    f"  → Rotación OK  θ={math.degrees(self.target_theta_abs):.1f}°  "
                    f"(segmento {self.pos_idx}/{len(self.positions)-1})")
                self.state = 'MOVIENDO'

        # ── MOVIENDO ──────────────────────────────────────────────────────────
        elif self.state == 'MOVIENDO':
            estado = self._mover_adelante(self.segment_target_xy)

            if estado == 'COMPLETADO':
                self.get_logger().info(
                    f"  → Segmento {self.pos_idx} completado ({self.segment_dist:.2f} m).")
                self.pos_idx += 1
                self._avanzar_estado_post_movimiento()

            elif estado == 'BLOQUEADO':
                self.get_logger().warn(
                    f"  ⚠ Segmento {self.pos_idx} BLOQUEADO – replanificando.")
                self._replanificar()

        # ── ROTACION_FINAL ────────────────────────────────────────────────────
        elif self.state == 'ROTACION_FINAL':
            if self._rotar_a(math.radians(self.qf_theta_deg)):
                self.get_logger().info(
                    f"  → Rotación final OK  θ={self.qf_theta_deg:.1f}°")
                self.state = 'RELOCALIZAR'

        # ── RELOCALIZAR ───────────────────────────────────────────────────────
        elif self.state == 'RELOCALIZAR':
            self._reportar_y_relocalizar()
            self.state = 'DONE'

    def _avanzar_estado_post_movimiento(self):
        if self.pos_idx < len(self.positions):
            x1, y1 = self.positions[self.pos_idx - 1]
            x2, y2 = self.positions[self.pos_idx]
            self.target_theta_abs  = math.atan2(y2 - y1, x2 - x1)
            self.segment_dist      = math.hypot(x2 - x1, y2 - y1)
            self.segment_target_xy = (x2, y2)
            self.get_logger().info(
                f"  Siguiente segmento [{self.pos_idx}]: "
                f"({x1:.3f},{y1:.3f}) → ({x2:.3f},{y2:.3f}), "
                f"θ={math.degrees(self.target_theta_abs):.1f}°, "
                f"dist={self.segment_dist:.2f} m")
            self.state = 'ROTANDO'
        else:
            self.state = 'ROTACION_FINAL'

    def _replanificar(self):
        """
        Replantea el camino desde la posición actual hasta qf.
        CAMBIO: reemplaza self.path_configs en lugar de acumular sobre él,
        lo que antes generaba un archivo .txt con configs duplicadas/incorrectas.
        """
        scene = self.scene_data
        if scene is None:
            self.get_logger().error("Replanificación: no hay escena cargada.")
            self.state = 'DONE'
            return

        qf         = scene['qf']
        pos_actual = (self.current_x, self.current_y)

        self.get_logger().info(
            f"  Replanificando desde ({pos_actual[0]:.2f}, {pos_actual[1]:.2f}) "
            f"→ qf=({qf[0]:.2f}, {qf[1]:.2f})")

        try:
            waypoints_raw, dt_plan = rrt(pos_actual, (qf[0], qf[1]),
                                        scene, ROBOT_RADIO)
        except ValueError as e:
            # Posición actual inválida (dentro de obstáculo): usar q0 de la escena
            self.get_logger().error(
                f"RRT rechazó pos_actual={pos_actual}: {e}\n"
                f"  → Reintentando desde q0 de escena ({scene['q0'][0]:.2f}, {scene['q0'][1]:.2f}).")
            q0_scene = scene['q0']
            pos_actual = (q0_scene[0], q0_scene[1])
            try:
                waypoints_raw, dt_plan = rrt(pos_actual, (qf[0], qf[1]),
                                            scene, ROBOT_RADIO)
            except ValueError as e2:
                self.get_logger().error(f"Replanificación fallida incluso desde q0: {e2}. Abortando.")
                self.state = 'DONE'
                return

        waypoints_suave = suavizar(waypoints_raw, scene, ROBOT_RADIO)
        new_pos = [pos_actual] + list(waypoints_suave[1:-1]) + [(qf[0], qf[1])]

        self.positions = new_pos
        self.pos_idx   = 1

        # CAMBIO: resetear path_configs en lugar de append (bug original)
        self.path_configs = []
        for i in range(len(new_pos) - 1):
            x1, y1 = new_pos[i]
            x2, y2 = new_pos[i + 1]
            th = math.degrees(math.atan2(y2 - y1, x2 - x1))
            self.path_configs.append((x1, y1, th))
            self.path_configs.append((x2, y2, th))
        self.path_configs.append((qf[0], qf[1], qf[2]))
        self._guardar_camino(self.path_configs, self.numero_escena)

        self._preparar_siguiente_segmento()
        self.state = 'ROTANDO'
    # ══════════════════════════════════════════════════════════════════════════

    def _reportar_y_relocalizar(self):
        scene  = self.scene_data
        qf_teo = scene['qf']
        qf_est = (self.current_x,
                  self.current_y,
                  math.degrees(self.current_theta))
        q_act = self._relocalizar(scene, qf_teo)

        d = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])
        a = lambda a, b: min(abs(a[2] - b[2]), 360 - abs(a[2] - b[2]))

        # NUEVO: tiempo total de ejecución
        t_ejecucion = (time.perf_counter() - self.t_inicio_ejecucion
                       if self.t_inicio_ejecucion else 0.0)
        sep = "═" * 65
        self.get_logger().info(sep)
        self.get_logger().info(f"  RESULTADOS RRT – Escena {self.numero_escena}")
        self.get_logger().info(sep)
        self.get_logger().info(
            f"  qf teórico  : x={qf_teo[0]:.4f} y={qf_teo[1]:.4f} θ={qf_teo[2]:.2f}°")
        self.get_logger().info(
            f"  qf estimado : x={qf_est[0]:.4f} y={qf_est[1]:.4f} θ={qf_est[2]:.2f}°")
        self.get_logger().info(
            f"  Error teo→est: {d(qf_teo, qf_est):.4f} m / {a(qf_teo, qf_est):.2f}°")
        if q_act:
            self.get_logger().info(
                f"  q_act real  : x={q_act[0]:.4f} y={q_act[1]:.4f} θ={q_act[2]:.2f}°")
            self.get_logger().info(
                f"  Error est→act: {d(qf_est, q_act):.4f} m / {a(qf_est, q_act):.2f}°")
        # NUEVO: bloque de métricas para la tabla del informe
        self.get_logger().info("─" * 65)
        self.get_logger().info("  MÉTRICAS PARA TABLA COMPARATIVA:")
        self.get_logger().info(
            f"  Distancia lineal recorrida    : {self.dist_recorrida_total:.4f} m")
        self.get_logger().info(
            f"  Suma absoluta angular         : {self.suma_angular_abs_deg:.2f}°")
        self.get_logger().info(
            f"  Tiempo generación (RRT)       : {self.dt_planificacion:.4f} s")
        self.get_logger().info(
            f"  Tiempo ejecución (simulación) : {t_ejecucion:.2f} s")
        self.get_logger().info(sep)
        self._guardar_camino(self.path_configs, self.numero_escena, qf_est, q_act)
        self.cmd_pub.publish(Twist())   # frenar el robot
    def _auto_start(self):
        self._inicio_timer.cancel()
        self.ejecutar_escena(self.numero_escena)

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    node.numero_escena = NUMERO_ESCENA
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()