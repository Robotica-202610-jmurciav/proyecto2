import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from collections import deque
import heapq
import math
import threading
import os

from ament_index_python.packages import get_package_share_directory
from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo


# ──────────────────────────────────────────────────────────────────────────────
# Constantes del robot y de planificación
# ──────────────────────────────────────────────────────────────────────────────
ROBOT_RADIO   = 0.15   # m  – mitad del cuadrado de 0.30 m que encierra al robot
CELL_SIZE     = 0.25   # m  – resolución de la cuadrícula
VEL_LINEAL    = 0.3    # m/s
VEL_ANGULAR   = 0.5    # rad/s
TOL_ANGULAR   = 0.04   # rad ≈ 2.3°
DIST_SEGURA   = 0.25   # m  – distancia mínima al obstáculo antes de abortar
CONO_VISION   = 25     # °  – semángulo del cono de detección frontal


# ══════════════════════════════════════════════════════════════════════════════

class NavigationNode(Node):
    def __init__(self):
        """
        Nodo ROS2 que planifica y ejecuta un camino geométrico autónomo.
        Máquina de estados principal
        ────────────────────────────
        ESPERANDO_COMANDO  →  ROTANDO  →  MOVIENDO  →  (repite por cada segmento)
            →  ROTACION_FINAL  →  RELOCALIZAR  →  DONE
        """
        super().__init__('student_navigation')
        
        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables de estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None
        
        # Estado del movimiento
        self.target_theta_abs  = None  # ángulo absoluto objetivo (rad)
        self.tiempo_maniobra   = 0.0   # cronómetro dead-reckoning (s)
        self.pose_inicial_flag = None  # bandera de inicio para mover_adelante
        
        # Datos adicionales
        self.scene_data    = None
        self.positions     = []    # [(x,y), …] waypoints en coordenadas mundo
        self.qf_theta_deg  = 0.0   # orientación final (°)
        self.path_configs  = []    # lista completa de configs para el archivo .txt
        self.pos_idx       = 1     # índice del siguiente waypoint a alcanzar
        self.numero_escena = 1     # número de escena cargada (1-6)
        self.segment_dist  = 0.0   # distancia del segmento actual (m)
        
        # Maquina de estados.
        self.state = 'ESPERANDO_COMANDO'  # 'ESPERANDO_COMANDO', 'EJECUTANDO_COMANDO'
        
        # Temporizadores
        self.create_timer(0.1, self.control_loop)
        self._inicio_timer = self.create_timer(2.0, self._auto_start)
        
        
        self.get_logger().info("Proyecto 2 - Nodo de Planificación Geométrica iniciado . . .")
        
    # =======================================================
    # CALLBACKS DE ROS2
    # =======================================================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def lidar_callback(self, msg):
        self.last_scan = msg



    # ══════════════════════════════════════════════════════════════════════════
    # HELPERS DE MOVIMIENTO
    # ══════════════════════════════════════════════════════════════════════════

    def _rotar_a(self, theta_objetivo_rad: float) -> bool:
        """
        Gira hasta alcanzar un ángulo absoluto en el marco mundo.
        Devuelve True cuando la rotación está completa.
        """
        cmd, done = calcular_rotacion(
            self.current_theta, theta_objetivo_rad,
            vel_angular_max=VEL_ANGULAR, tolerancia=TOL_ANGULAR
        )
        self.cmd_pub.publish(cmd)
        return done

    def _mover_adelante(self, distancia: float) -> str:
        """
        Avanza el robot hacia su frente la distancia indicada (dead-reckoning).
        Devuelve: 'EN_RUTA' | 'COMPLETADO' | 'BLOQUEADO'
        """
        if self.pose_inicial_flag is None:
            self.pose_inicial_flag = True
            self.tiempo_maniobra   = 0.0

        cono = obtener_distancias_rango(self.last_scan, -CONO_VISION, CONO_VISION)

        # dist_x = distancia, dist_y = 0  →  movimiento puro hacia el frente
        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra,
            distancia, 0.0,
            cono,
            dist_segura=DIST_SEGURA,
            vel_lineal=VEL_LINEAL
        )
        self.cmd_pub.publish(cmd)
        self.tiempo_maniobra += 0.1

        if estado in ('COMPLETADO', 'BLOQUEADO'):
            self.pose_inicial_flag = None
            self.tiempo_maniobra   = 0.0

        return estado

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
    # CONSTRUCCIÓN DEL C-SPACE EN CUADRÍCULA
    # ══════════════════════════════════════════════════════════════════════════

    def _construir_grid(self, scene: dict):
        """
        Clasifica cada celda como:
          0  libre      - el robot puede transitar libremente
          1  ocupada    - dentro de ROBOT_RADIO del obstáculo / pared (C-obstáculo)
          2  semi-libre - entre ROBOT_RADIO y 2·ROBOT_RADIO (zona de riesgo)

        Devuelve (grid, filas, columnas).
        """
        W, H  = scene['ancho'], scene['alto']
        cols  = int(math.ceil(W / CELL_SIZE))
        rows  = int(math.ceil(H / CELL_SIZE))
        grid  = [[0] * cols for _ in range(rows)]

        for r in range(rows):
            for c in range(cols):
                cx = (c + 0.5) * CELL_SIZE
                cy = (r + 0.5) * CELL_SIZE

                # ── Distancia a las paredes del escenario ──
                d_min = min(cx, cy, W - cx, H - cy)

                # ── Distancia a cada obstáculo rectangular ──
                for obs in scene['obstaculos']:
                    x1, y1 = obs['p1']
                    x2, y2 = obs['p2']
                    xmin, xmax = min(x1, x2), max(x1, x2)
                    ymin, ymax = min(y1, y2), max(y1, y2)
                    dx = max(xmin - cx, 0.0, cx - xmax)
                    dy = max(ymin - cy, 0.0, cy - ymax)
                    d_min = min(d_min, math.sqrt(dx * dx + dy * dy))

                # ── Clasificación ──
                if   d_min < ROBOT_RADIO:
                    grid[r][c] = 1              # ocupada
                elif d_min < 2 * ROBOT_RADIO:
                    grid[r][c] = 2              # semi-libre

        return grid, rows, cols

    # ── Conversión mundo ↔ celda ──────────────────────────────────────────────

    def _xy_a_celda(self, x: float, y: float):
        return int(y / CELL_SIZE), int(x / CELL_SIZE)

    def _celda_a_xy(self, r: int, c: int):
        return (c + 0.5) * CELL_SIZE, (r + 0.5) * CELL_SIZE

    # ══════════════════════════════════════════════════════════════════════════
    # PLANIFICADOR A*
    # ══════════════════════════════════════════════════════════════════════════

    def _astar(self, grid, rows: int, cols: int,
               start_xy: tuple, goal_xy: tuple) -> list | None:
        """
        A* 8-conectado sobre la cuadrícula del C-space.
        Las celdas semi-libres tienen coste doble para incentivar rutas alejadas
        de los obstáculos (criterio de optimización en seguridad).
        """
        start = self._xy_a_celda(*start_xy)
        goal  = self._xy_a_celda(*goal_xy)
        # Asegurar que los extremos sean válidos
        start = self._celda_libre_mas_cercana(grid, rows, cols, start)
        goal  = self._celda_libre_mas_cercana(grid, rows, cols, goal)

        self.get_logger().info(f"A* → celda inicio {start}, celda meta {goal}")

        DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]

        def h(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        open_set  = [(0.0, start)]
        g_score   = {start: 0.0}
        came_from = {}

        while open_set:
            _, cur = heapq.heappop(open_set)

            if cur == goal:
                # Reconstruir camino
                path = []
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                return list(reversed(path))

            for dr, dc in DIRS:
                nb = (cur[0] + dr, cur[1] + dc)
                if not (0 <= nb[0] < rows and 0 <= nb[1] < cols):
                    continue
                if grid[nb[0]][nb[1]] == 1:     # celda ocupada: no transitable
                    continue

                coste_movimiento = math.sqrt(dr * dr + dc * dc)
                if grid[nb[0]][nb[1]] == 2:
                    coste_movimiento *= 2.0     # penalización zona semi-libre

                ng = g_score[cur] + coste_movimiento
                if nb not in g_score or ng < g_score[nb]:
                    g_score[nb]   = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + h(nb, goal), nb))

        self.get_logger().error("A*: no se encontró camino hacia el objetivo.")
        return None

    def _celda_libre_mas_cercana(self, grid, rows: int, cols: int, celda: tuple):
        """BFS desde 'celda' hasta encontrar la celda libre más cercana."""
        if grid[celda[0]][celda[1]] != 1:
            return celda
        q= deque([celda])
        visited = {celda}
        while q:
            r, c = q.popleft()
            if grid[r][c] != 1:
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nb = (r + dr, c + dc)
                if 0 <= nb[0] < rows and 0 <= nb[1] < cols and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
        return celda

    # ══════════════════════════════════════════════════════════════════════════
    # SUAVIZADO (visibilidad directa con Bresenham)
    # ══════════════════════════════════════════════════════════════════════════

    def _suavizar_camino(self, path: list, grid) -> list:
        """
        Elimina waypoints intermedios cuando el segmento resultante es
        completamente libre (sin celdas ocupadas).
        """
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._vision_directa(path[i], path[j], grid):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def _vision_directa(self, p1: tuple, p2: tuple, grid) -> bool:
        """Bresenham: devuelve True si el segmento p1→p2 no cruza celdas ocupadas."""
        r1, c1 = p1
        r2, c2 = p2
        dr, dc = abs(r2 - r1), abs(c2 - c1)
        sr, sc = (1 if r2 > r1 else -1), (1 if c2 > c1 else -1)
        err    = dr - dc
        r, c   = r1, c1
        while True:
            if grid[r][c] == 1:
                return False
            if r == r2 and c == c2:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r   += sr
            if e2 <  dr:
                err += dr
                c   += sc

    # ══════════════════════════════════════════════════════════════════════════
    # RELOCALIZACIÓN CON LiDAR
    # ══════════════════════════════════════════════════════════════════════════

    def _relocalizar(self, scene: dict, qf_teo: tuple) -> tuple | None:
        """
        Calcula la configuración real del robot a partir de las lecturas LiDAR
        al frente y a la derecha.

        Método:
        -------
        Dado el ángulo teórico θ_f, se calculan los vectores unitarios de la
        dirección 'frente' (f̂) y 'derecha' (r̂) en el marco mundo.
        Con ray-casting se encuentra el obstáculo más cercano en cada dirección.
        Luego se plantea el sistema 2×2:

          [fx  fy] [x_act]   [b_f - d_lidar_frente]
          [rx  ry] [y_act] = [b_r - d_lidar_derecha]

        donde b_f y b_r son las proyecciones escalares del punto de intersección
        sobre sus respectivos ejes, lo que permite determinar (x_act, y_act).
        """
        if self.last_scan is None:
            return None

        # ── Lecturas LiDAR (marco cuerpo: frente=0°, derecha=-90°) ──
        d_f = obtener_distancia_angulo(self.last_scan, 0.0)
        d_r = obtener_distancia_angulo(self.last_scan, math.radians(-90.0))

        # Si el sensor no devuelve dato válido, se usan los valores esperados de la escena
        if not (0 < d_f < float('inf')):
            d_f = scene['dFrente']
        if not (0 < d_r < float('inf')):
            d_r = scene['dDerecha']

        theta = math.radians(qf_teo[2])

        # Vectores unitarios en marco mundo
        fx, fy = math.cos(theta),                  math.sin(theta)                  # frente
        rx, ry = math.cos(theta - math.pi / 2),    math.sin(theta - math.pi / 2)    # derecha

        front_pt = self._raycast(scene, qf_teo[0], qf_teo[1], fx, fy)
        right_pt = self._raycast(scene, qf_teo[0], qf_teo[1], rx, ry)

        if front_pt is None or right_pt is None:
            self.get_logger().warn(
                "Relocalización: no se encontró obstáculo de referencia. "
                "Verificar que qf esté orientada hacia obstáculos conocidos.")
            return None

        # Proyecciones escalares
        b_f = front_pt[0] * fx + front_pt[1] * fy
        b_r = right_pt[0] * rx + right_pt[1] * ry

        rhs1 = b_f - d_f
        rhs2 = b_r - d_r

        # Resolver sistema 2×2  (det = fx·ry − fy·rx)
        det = fx * ry - fy * rx
        if abs(det) < 1e-6:
            # Caso degenerado: usar estimación componente a componente
            x_act = right_pt[0] - d_r * rx
            y_act = front_pt[1] - d_f * fy
        else:
            x_act = (rhs1 * ry - rhs2 * fy) / det
            y_act = (fx * rhs2 - rx * rhs1) / det

        return (x_act, y_act, qf_teo[2])

    def _raycast(self, scene: dict, ox: float, oy: float,
                 dx: float, dy: float) -> tuple | None:
        """
        Ray casting desde (ox, oy) en dirección (dx, dy).
        Devuelve el punto de intersección más cercano con paredes u obstáculos.
        """
        W, H      = scene['ancho'], scene['alto']
        best_t    = float('inf')
        best_pt   = None

        def actualizar(t, px, py):
            nonlocal best_t, best_pt
            if t > 1e-3 and t < best_t:
                best_t  = t
                best_pt = (px, py)

        # Paredes del escenario
        if abs(dx) > 1e-9:
            for wx in (0.0, W):
                t = (wx - ox) / dx
                actualizar(t, ox + t * dx, oy + t * dy)
        if abs(dy) > 1e-9:
            for wy in (0.0, H):
                t = (wy - oy) / dy
                actualizar(t, ox + t * dx, oy + t * dy)

        # Obstáculos rectangulares
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

    def _guardar_camino(self, configs: list, escena: int,
                        qf_est=None, q_act=None):
        base = os.path.dirname(os.path.abspath(__file__))
        ruta = os.path.join(base, '..', 'data', f'Camino-Escena{escena}.txt')
        try:
            with open(ruta, 'w', encoding='utf-8') as f:
                f.write("# Camino geométrico solución – Proyecto 2\n")
                f.write("# Formato: x,y,theta_deg\n")
                for cfg in configs:
                    f.write(f"{cfg[0]:.4f},{cfg[1]:.4f},{cfg[2]:.2f}\n")
                if qf_est:
                    f.write(
                        f"# qf_est,{qf_est[0]:.4f},{qf_est[1]:.4f},{qf_est[2]:.2f}\n")
                if q_act:
                    f.write(
                        f"# q_act,{q_act[0]:.4f},{q_act[1]:.4f},{q_act[2]:.2f}\n")
            self.get_logger().info(f"Camino guardado → {ruta}")
        except Exception as e:
            self.get_logger().error(f"Error guardando camino: {e}")

    # ══════════════════════════════════════════════════════════════════════════
    # PUNTO DE ENTRADA DE LA MISIÓN
    # ══════════════════════════════════════════════════════════════════════════

    def _auto_start(self):
        """Inicia la misión automáticamente tras 2 s (sensores listos)."""
        self._inicio_timer.cancel()
        self.ejecutar_escena(self.numero_escena)

    def ejecutar_escena(self, numero: int):
        """
        Punto de entrada público.  Parsea, planifica y arranca la ejecución.
        """
        self.numero_escena = numero

        # 1. Parsear escena
        scene = self._parsear_escena(numero)
        if scene is None:
            return
        self.scene_data = scene
        q0, qf = scene['q0'], scene['qf']

        # 2. Construir cuadrícula C-space
        grid, rows, cols = self._construir_grid(scene)
        self.get_logger().info(
            f"C-space: cuadrícula {rows}×{cols} celdas "
            f"(resolución {CELL_SIZE} m, margen robot {ROBOT_RADIO} m)")

        # 3. Planificar con A*
        grid_path = self._astar(grid, rows, cols, (q0[0], q0[1]), (qf[0], qf[1]))
        if grid_path is None:
            self.get_logger().error("Planificación fallida. Abortando misión.")
            return
        self.get_logger().info(f"A*: {len(grid_path)} celdas en el camino bruto")

        # 4. Suavizar
        smooth = self._suavizar_camino(grid_path, grid)
        self.get_logger().info(f"Camino suavizado: {len(smooth)} waypoints")

        # 5. Convertir a coordenadas mundo
        positions = [(q0[0], q0[1])]                            # inicio exacto
        for cell in smooth[1:-1]:
            positions.append(self._celda_a_xy(cell[0], cell[1]))
        positions.append((qf[0], qf[1]))                        # fin exacto
        self.positions   = positions
        self.qf_theta_deg = qf[2]

        # 6. Construir lista de configuraciones para el archivo .txt
        # Formato: (pos_rotación, pos_llegada) por segmento + rotación final
        self.path_configs = []
        for i in range(len(positions) - 1):
            x1, y1 = positions[i]
            x2, y2 = positions[i + 1]
            theta_seg = math.degrees(math.atan2(y2 - y1, x2 - x1))
            self.path_configs.append((x1, y1, theta_seg))   # qrot_i-i+1
            self.path_configs.append((x2, y2, theta_seg))   # q_i+1
        self.path_configs.append((qf[0], qf[1], qf[2]))     # qrot_n (orientación final)

        self._guardar_camino(self.path_configs, numero)

        # 7. Iniciar ejecución
        self.pos_idx = 1
        self._preparar_siguiente_segmento()
        self.state = 'ROTANDO'

        self.get_logger().info(
            f"Misión iniciada: {len(positions) - 1} segmentos, "
            f"distancia total ≈ "
            f"{sum(math.sqrt((positions[i+1][0]-positions[i][0])**2+(positions[i+1][1]-positions[i][1])**2) for i in range(len(positions)-1)):.2f} m"
        )

    def _preparar_siguiente_segmento(self):
        """Calcula heading y distancia del segmento actual."""
        if self.pos_idx < len(self.positions):
            x1, y1 = self.positions[self.pos_idx - 1]
            x2, y2 = self.positions[self.pos_idx]
            self.target_theta_abs = math.atan2(y2 - y1, x2 - x1)
            self.segment_dist     = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # ══════════════════════════════════════════════════════════════════════════
    # BUCLE DE CONTROL PRINCIPAL  
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        if self.last_scan is None or self.state in ('IDLE', 'DONE'):
            return

        # ── ROTANDO: alinear hacia el siguiente waypoint ──────────────────────
        if self.state == 'ROTANDO':
            if self._rotar_a(self.target_theta_abs):
                self.get_logger().info(
                    f"  → Rotación OK  θ={math.degrees(self.target_theta_abs):.1f}°  "
                    f"(segmento {self.pos_idx}/{len(self.positions)-1})")
                self.state = 'MOVIENDO'

        # ── MOVIENDO: avanzar al siguiente waypoint ───────────────────────────
        elif self.state == 'MOVIENDO':
            estado = self._mover_adelante(self.segment_dist)

            if estado == 'COMPLETADO':
                self.get_logger().info(
                    f"  → Segmento {self.pos_idx} completado "
                    f"({self.segment_dist:.2f} m).")
                self.pos_idx += 1
                self._avanzar_estado_post_movimiento()

            elif estado == 'BLOQUEADO':
                self.get_logger().warn(
                    f"  ⚠ Segmento {self.pos_idx} BLOQUEADO – saltando al siguiente.")
                self.pos_idx += 1
                self._avanzar_estado_post_movimiento()

        # ── ROTACION FINAL: girar a la orientación de qf ─────────────────────
        elif self.state == 'ROTACION_FINAL':
            if self._rotar_a(math.radians(self.qf_theta_deg)):
                self.get_logger().info(
                    f"  → Rotación final OK  θ={self.qf_theta_deg:.1f}°")
                self.state = 'RELOCALIZAR'

        # ── RELOCALIZAR: se ejecuta una sola vez ──────────────────────────────
        elif self.state == 'RELOCALIZAR':
            self._reportar_y_relocalizar()
            self.state = 'DONE'

    def _avanzar_estado_post_movimiento(self):
        """Decide si continuar con el siguiente segmento o pasar a rotación final."""
        if self.pos_idx < len(self.positions):
            self._preparar_siguiente_segmento()
            self.state = 'ROTANDO'
        else:
            self.state = 'ROTACION_FINAL'

    # ══════════════════════════════════════════════════════════════════════════
    # REPORTE FINAL Y RELOCALIZACIÓN
    # ══════════════════════════════════════════════════════════════════════════

    def _reportar_y_relocalizar(self):
        scene  = self.scene_data
        qf_teo = scene['qf']
        qf_est = (self.current_x,
                  self.current_y,
                  math.degrees(self.current_theta))

        q_act  = self._relocalizar(scene, qf_teo)

        # ── Errores ──────────────────────────────────────────────────────────
        d_teo_est = math.sqrt(
            (qf_teo[0] - qf_est[0]) ** 2 + (qf_teo[1] - qf_est[1]) ** 2)
        a_teo_est = abs(qf_teo[2] - qf_est[2])
        a_teo_est = min(a_teo_est, 360 - a_teo_est)

        sep = "═" * 60
        self.get_logger().info(sep)
        self.get_logger().info("  RESULTADOS  Escena %d" % self.numero_escena)
        self.get_logger().info(sep)
        self.get_logger().info(
            f"  qf teórico  (qf)     : "
            f"x={qf_teo[0]:.4f} m, y={qf_teo[1]:.4f} m, θ={qf_teo[2]:.2f}°")
        self.get_logger().info(
            f"  qf estimado (qf_est) : "
            f"x={qf_est[0]:.4f} m, y={qf_est[1]:.4f} m, θ={qf_est[2]:.2f}°")
        self.get_logger().info(
            f"  Error (qf → qf_est)  : "
            f"{d_teo_est:.4f} m  |  {a_teo_est:.2f}°")

        if q_act:
            d_est_act = math.sqrt(
                (qf_est[0] - q_act[0]) ** 2 + (qf_est[1] - q_act[1]) ** 2)
            a_est_act = abs(qf_est[2] - q_act[2])
            a_est_act = min(a_est_act, 360 - a_est_act)
            self.get_logger().info(
                f"  q_act real  (q_act)  : "
                f"x={q_act[0]:.4f} m, y={q_act[1]:.4f} m, θ={q_act[2]:.2f}°")
            self.get_logger().info(
                f"  Error (qf_est→q_act) : "
                f"{d_est_act:.4f} m  |  {a_est_act:.2f}°")
        else:
            self.get_logger().warn(
                "  Relocalización no disponible – LiDAR sin lecturas válidas.")

        self.get_logger().info(sep)

        # Completar el archivo con configuraciones finales
        self._guardar_camino(self.path_configs, self.numero_escena, qf_est, q_act)

        # Frenar motores
        self.cmd_pub.publish(Twist())


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

# ▼▼▼  CAMBIAR ESTE NÚMERO PARA SELECCIONAR LA ESCENA  ▼▼▼
NUMERO_ESCENA = 1
# ▲▲▲──────────────────────────────────────────────────▲▲▲


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    node.numero_escena = NUMERO_ESCENA     # propagar la escena al nodo
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())      # detener motores al salir
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()