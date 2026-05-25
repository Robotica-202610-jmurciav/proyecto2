"""
planner_rrt.py  -  Proyecto 3, ISIS4826 Robótica Móvil
Planificador RRT con mejoras de calidad de camino (RRT* lite).
"""

import math
import random
import time

# ══════════════════════════════════════════════════════════════════════════════
# INFLADO DE OBSTÁCULOS  (C-space)
# ══════════════════════════════════════════════════════════════════════════════

def _inflar_obstaculo(p1: tuple, p2: tuple, radio: float) -> tuple:
    x1, y1 = p1
    x2, y2 = p2
    return (min(x1, x2) - radio,
            min(y1, y2) - radio,
            max(x1, x2) + radio,
            max(y1, y2) + radio)


def _inflar_paredes(ancho: float, alto: float, radio: float) -> list:
    return [
        (     -1.0,   -1.0,   radio,   alto + 1.0),   # pared izquierda
        (ancho-radio, -1.0, ancho+1.0, alto + 1.0),   # pared derecha
        (     -1.0,   -1.0, ancho+1.0,        radio), # pared inferior
        (     -1.0, alto-radio, ancho+1.0, alto+1.0), # pared superior
    ]


def _construir_obstaculos(scene: dict, robot_radio: float) -> list:
    obs = [_inflar_obstaculo(o['p1'], o['p2'], robot_radio)for o in scene['obstaculos']]
    obs += _inflar_paredes(scene['ancho'], scene['alto'], robot_radio)
    return obs


# ══════════════════════════════════════════════════════════════════════════════
# GEOMETRÍA DE COLISIÓN
# ══════════════════════════════════════════════════════════════════════════════

def _punto_en_rect(px: float, py: float, rect: tuple) -> bool:
    xmin, ymin, xmax, ymax = rect
    return xmin <= px <= xmax and ymin <= py <= ymax


def _segmento_intersecta_rect(p1: tuple, p2: tuple, rect: tuple) -> bool:
    """Determina si el segmento p1-p2 intersecta el rectángulo definido por rect."""
    xmin, ymin, xmax, ymax = rect
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    p_vals = [-dx,  dx, -dy,  dy]
    q_vals = [p1[0] - xmin,
              xmax  - p1[0],
              p1[1] - ymin,
              ymax  - p1[1]]

    t0, t1 = 0.0, 1.0
    for p, q in zip(p_vals, q_vals):
        if p == 0.0:
            if q < 0.0:
                return False
        elif p < 0.0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)
        if t0 > t1:
            return False
    return True


def _es_libre(p1: tuple, p2: tuple, obstaculos_inflados: list) -> bool:
    for rect in obstaculos_inflados:
        if _segmento_intersecta_rect(p1, p2, rect):
            return False
    return True


def _punto_libre(p: tuple, obstaculos_inflados: list) -> bool:
    for rect in obstaculos_inflados:
        if _punto_en_rect(p[0], p[1], rect):
            return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
# UTILIDADES GEOMÉTRICAS
# ══════════════════════════════════════════════════════════════════════════════

def _distancia(a: tuple, b: tuple) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _extender(q_near: tuple, q_rand: tuple, delta: float) -> tuple:
    """
    Avanza hasta delta metros desde q_near hacia q_rand.
    CAMBIO: si q_rand ya está a menos de delta, se llega exactamente
    a q_rand (delta adaptativo). Esto evita sobrepasar la meta.
    """
    d = _distancia(q_near, q_rand)
    if d < 1e-9:
        return q_near
    t = min(delta / d, 1.0)       # t=1.0 si q_rand está más cerca que delta
    return (q_near[0] + t * (q_rand[0] - q_near[0]),
            q_near[1] + t * (q_rand[1] - q_near[1]))


# ══════════════════════════════════════════════════════════════════════════════
# ALGORITMO RRT  (con reconexión local RRT* lite)
# ══════════════════════════════════════════════════════════════════════════════

DELTA      = 0.30    # m  – paso de extensión
TOL_META   = 0.25    # m  – radio de captura de la meta
MAX_ITER   = 15_000  # CAMBIO: aumentado de 12_000 a 15_000 para escenas difíciles
SESGO_META = 0.12    # CAMBIO: aumentado levemente (0.10 → 0.12) para convergencia más rápida
RADIO_REWIRE = 0.60  # m  – radio de vecindad para reconexión RRT* lite


def rrt(q0_xy: tuple,
        qf_xy: tuple,
        scene: dict,
        robot_radio: float,
        delta: float    = DELTA,
        tol_meta: float = TOL_META,
        max_iter: int   = MAX_ITER,
        sesgo: float    = SESGO_META,
        semilla: int    = None) -> tuple:
    """
    Planificador RRT con reconexión local (RRT* lite).

    CAMBIO principal respecto a la versión original:
    ─────────────────────────────────────────────────
    Después de agregar q_new al árbol, se buscan todos los nodos dentro
    de RADIO_REWIRE metros.  Para cada vecino, si el camino
      q0 → ... → q_new → vecino
    es más corto que el camino actual
      q0 → ... → vecino
    se reconecta el vecino para que su padre sea q_new.

    Esto reduce zigzags sin el coste completo de RRT*, y produce
    caminos más cortos que el RRT básico antes del suavizado.

    Parámetros
    ----------
    q0_xy, qf_xy  : (x, y) inicio y fin en metros
    scene         : dict parseado de _parsear_escena()
    robot_radio   : radio de inflado de obstáculos (m)
    delta         : longitud de paso máximo por iteración (m)
    tol_meta      : distancia para considerar que llegamos a qf (m)
    max_iter      : número máximo de iteraciones
    sesgo         : fracción de muestras dirigidas directamente a qf
    semilla       : para reproducibilidad (None = aleatorio, recomendado)

    Retorna
    -------
    (waypoints, tiempo_s)  — waypoints es lista de (x, y), o (None, t)
    """
    if semilla is not None:
        random.seed(semilla)

    ancho = scene['ancho']
    alto  = scene['alto']
    t0    = time.perf_counter()

    obs_inf = _construir_obstaculos(scene, robot_radio)

    for q, nombre in [(q0_xy, 'q0'), (qf_xy, 'qf')]:
        if not _punto_libre(q, obs_inf):
            raise ValueError(
                f"[RRT] {nombre}={q} cae dentro de un obstáculo inflado. "
                f"Reducir robot_radio ({robot_radio} m) o corregir la escena.")

    # Árbol
    nodos  = [q0_xy]
    padres = {0: None}
    costos = {0: 0.0}    # NUEVO: costo acumulado desde q0 para cada nodo

    for _ in range(max_iter):

        # ── 1. Muestreo con sesgo hacia la meta ──────────────────────────────
        if random.random() < sesgo:
            q_rand = qf_xy
        else:
            q_rand = (random.uniform(0.0, ancho),
                      random.uniform(0.0, alto))

        # ── 2. Nodo más cercano ───────────────────────────────────────────────
        idx_near = min(range(len(nodos)),
                       key=lambda i: _distancia(nodos[i], q_rand))
        q_near   = nodos[idx_near]

        # ── 3. Extensión (delta adaptativo) ───────────────────────────────────
        q_new = _extender(q_near, q_rand, delta)

        # ── 4. Verificación de colisión ───────────────────────────────────────
        if not _es_libre(q_near, q_new, obs_inf):
            continue

        # ── 5. Agregar al árbol ───────────────────────────────────────────────
        idx_new  = len(nodos)
        nodos.append(q_new)

        # Elegir el mejor padre dentro del radio de rewire
        # (puede ser q_near u otro vecino más barato)
        mejor_padre = idx_near
        mejor_costo = costos[idx_near] + _distancia(q_near, q_new)

        for idx_v, q_v in enumerate(nodos[:-1]):   # sin incluir q_new
            if _distancia(q_v, q_new) < RADIO_REWIRE:
                costo_via_v = costos[idx_v] + _distancia(q_v, q_new)
                if costo_via_v < mejor_costo and _es_libre(q_v, q_new, obs_inf):
                    mejor_padre = idx_v
                    mejor_costo = costo_via_v

        padres[idx_new] = mejor_padre
        costos[idx_new] = mejor_costo

        # ── 6. Reconexión (rewire) de vecinos ─────────────────────────────────
        # NUEVO: si pasar por q_new mejora el costo de algún vecino, reconectar
        for idx_v, q_v in enumerate(nodos[:-1]):
            if idx_v == mejor_padre:
                continue
            if _distancia(q_new, q_v) < RADIO_REWIRE:
                costo_via_new = costos[idx_new] + _distancia(q_new, q_v)
                if costo_via_new < costos.get(idx_v, float('inf')):
                    if _es_libre(q_new, q_v, obs_inf):
                        padres[idx_v] = idx_new
                        costos[idx_v] = costo_via_new

        # ── 7. Comprobar si alcanzamos la meta ────────────────────────────────
        if (_distancia(q_new, qf_xy) <= tol_meta
                and _es_libre(q_new, qf_xy, obs_inf)):
            idx_qf = len(nodos)
            nodos.append(qf_xy)
            padres[idx_qf] = idx_new
            costos[idx_qf] = costos[idx_new] + _distancia(q_new, qf_xy)
            dt     = time.perf_counter() - t0
            camino = _reconstruir(nodos, padres, idx_qf)
            return camino, dt

    dt = time.perf_counter() - t0
    return None, dt


def _reconstruir(nodos: list, padres: dict, idx_final: int) -> list:
    camino, idx = [], idx_final
    while idx is not None:
        camino.append(nodos[idx])
        idx = padres[idx]
    camino.reverse()
    return camino


# ══════════════════════════════════════════════════════════════════════════════
# SUAVIZADO  (greedy shortcut)
# ══════════════════════════════════════════════════════════════════════════════

def suavizar(waypoints: list,
             scene: dict,
             robot_radio: float,
             max_pasadas: int = 500) -> list:
    """
    Elimina waypoints intermedios cuando el atajo directo es libre.
    Equivalente continuo al suavizado Bresenham del proyecto 2.
    """
    obs_inf = _construir_obstaculos(scene, robot_radio)
    suave   = list(waypoints)

    for _ in range(max_pasadas):
        cambio = False
        i = 0
        while i < len(suave) - 2:
            if _es_libre(suave[i], suave[i + 2], obs_inf):
                suave.pop(i + 1)
                cambio = True
            else:
                i += 1
        if not cambio:
            break

    return suave


# ══════════════════════════════════════════════════════════════════════════════
# MÉTRICAS DE CAMINO  
# ══════════════════════════════════════════════════════════════════════════════

def calcular_longitud_camino(waypoints: list) -> float:
    """
    Retorna la longitud total del camino geométrico en metros.

    NUEVO: necesario para la columna 'Distancia lineal recorrida' del informe.
    """
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += _distancia(waypoints[i], waypoints[i + 1])
    return total


def calcular_suma_angular(waypoints: list, theta_inicial_deg: float = 0.0,
                          theta_final_deg: float = 0.0) -> float:
    """
    Suma absoluta de todas las rotaciones del camino (en grados).

    Incluye:
    - La rotación inicial desde theta_inicial_deg hasta el heading del
      primer segmento.
    - Cada giro entre segmentos consecutivos.
    - La rotación final desde el último heading hasta theta_final_deg.

    NUEVO: necesario para la columna 'Suma absoluta angular' del informe.
    """
    if len(waypoints) < 2:
        return 0.0

    # Headings de cada segmento (en grados)
    headings = []
    for i in range(len(waypoints) - 1):
        dx = waypoints[i + 1][0] - waypoints[i][0]
        dy = waypoints[i + 1][1] - waypoints[i][1]
        headings.append(math.degrees(math.atan2(dy, dx)))

    suma = 0.0

    # Rotación inicial (de la orientación de partida al primer heading)
    diff_ini = headings[0] - theta_inicial_deg
    diff_ini = math.degrees(math.atan2(math.sin(math.radians(diff_ini)),
                                       math.cos(math.radians(diff_ini))))
    suma += abs(diff_ini)

    # Giros entre segmentos consecutivos
    for i in range(len(headings) - 1):
        diff = headings[i + 1] - headings[i]
        diff = math.degrees(math.atan2(math.sin(math.radians(diff)),
                                       math.cos(math.radians(diff))))
        suma += abs(diff)

    # Rotación final (del último heading a la orientación destino)
    diff_fin = theta_final_deg - headings[-1]
    diff_fin = math.degrees(math.atan2(math.sin(math.radians(diff_fin)),
                                       math.cos(math.radians(diff_fin))))
    suma += abs(diff_fin)

    return suma