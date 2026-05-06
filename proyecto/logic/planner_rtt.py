"""
planner_rrt.py  -  Proyecto 3, ISIS4826 Robótica Móvil
Planificador RRT en espacio continuo.
"""

import math
import random
import time


def _inflar_obstaculo(p1: tuple, p2: tuple, radio: float) -> tuple:
    """
    Recibe las dos esquinas de un obstáculo rectangular del proyecto 2
    y devuelve (xmin, ymin, xmax, ymax) inflado por 'radio'.
    """
    x1, y1 = p1
    x2, y2 = p2
    return (min(x1, x2) - radio,
            min(y1, y2) - radio,
            max(x1, x2) + radio,
            max(y1, y2) + radio)


def _inflar_paredes(ancho: float, alto: float, radio: float) -> list:
    """
    Devuelve 4 rectángulos inflados que representan las paredes del escenario.
    Se modelan como franjas que invaden el espacio libre por 'radio'.
    """
    return [
        (     -1.0,   -1.0,   radio,   alto + 1.0),   # pared izquierda
        (ancho-radio, -1.0, ancho+1.0, alto + 1.0),   # pared derecha
        (     -1.0,   -1.0, ancho+1.0,        radio), # pared inferior
        (     -1.0, alto-radio, ancho+1.0, alto+1.0), # pared superior
    ]


def _construir_obstaculos(scene: dict, robot_radio: float) -> list:
    """
    Devuelve la lista de todos los rectángulos inflados
    (obstáculos + paredes) como tuplas (xmin, ymin, xmax, ymax).
    """
    obs = [_inflar_obstaculo(o['p1'], o['p2'], robot_radio)
           for o in scene['obstaculos']]
    obs += _inflar_paredes(scene['ancho'], scene['alto'], robot_radio)
    return obs


# ══════════════════════════════════════════════════════════════════════════════
# GEOMETRÍA DE COLISIÓN — AABB puro, sin librerías externas
# ══════════════════════════════════════════════════════════════════════════════

def _punto_en_rect(px: float, py: float, rect: tuple) -> bool:
    """True si el punto (px, py) está dentro del rectángulo."""
    xmin, ymin, xmax, ymax = rect
    return xmin <= px <= xmax and ymin <= py <= ymax


def _segmento_intersecta_rect(p1: tuple, p2: tuple, rect: tuple) -> bool:
    """
    Detecta si el segmento p1→p2 intersecta el rectángulo (AABB).

    Algoritmo: Liang-Barsky.
    Recorta paramétricamente el segmento contra los 4 semiplanos del
    rectángulo. Si el intervalo paramétrico [t0, t1] no queda vacío,
    hay intersección.

    Ventaja: maneja correctamente los casos en que uno o ambos extremos
    están dentro del rectángulo, o el segmento lo atraviesa.
    """
    xmin, ymin, xmax, ymax = rect
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    # p·t <= q  para cada semiplano
    # Orden: izquierda, derecha, abajo, arriba
    p_vals = [-dx,  dx, -dy,  dy]
    q_vals = [p1[0] - xmin,
              xmax  - p1[0],
              p1[1] - ymin,
              ymax  - p1[1]]

    t0, t1 = 0.0, 1.0

    for p, q in zip(p_vals, q_vals):
        if p == 0.0:
            # Segmento paralelo a este borde
            if q < 0.0:
                return False   # completamente fuera de este semiplano
        elif p < 0.0:
            t0 = max(t0, q / p)
        else:
            t1 = min(t1, q / p)

        if t0 > t1:
            return False   # intervalo vacío → sin intersección

    return True


def _es_libre(p1: tuple, p2: tuple, obstaculos_inflados: list) -> bool:
    """
    True si el segmento p1→p2 no intersecta ningún obstáculo inflado.
    """
    for rect in obstaculos_inflados:
        if _segmento_intersecta_rect(p1, p2, rect):
            return False
    return True


def _punto_libre(p: tuple, obstaculos_inflados: list) -> bool:
    """True si el punto p no está dentro de ningún obstáculo inflado."""
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
    """Avanza δ metros desde q_near en dirección a q_rand."""
    d = _distancia(q_near, q_rand)
    if d < 1e-9:
        return q_near
    t = min(delta / d, 1.0)
    return (q_near[0] + t * (q_rand[0] - q_near[0]),
            q_near[1] + t * (q_rand[1] - q_near[1]))


# ══════════════════════════════════════════════════════════════════════════════
# ALGORITMO RRT
# ══════════════════════════════════════════════════════════════════════════════

# Constantes por defecto
DELTA      = 0.30    # m  – paso de extensión
TOL_META   = 0.25    # m  – radio de captura de la meta
MAX_ITER   = 12_000  # iteraciones máximas
SESGO_META = 0.10    # fracción de muestras hacia qf


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
    Planificador RRT.

    Parámetros
    ----------
    q0_xy, qf_xy  : (x, y) inicio y fin en metros
    scene         : dict de _parsear_escena() — necesita 'ancho','alto','obstaculos'
    robot_radio   : radio de inflado de obstáculos (m)
    delta         : longitud de paso máximo por iteración (m)
    tol_meta      : distancia para considerar que llegamos a qf (m)
    max_iter      : número máximo de iteraciones
    sesgo         : fracción de muestras dirigidas directamente a qf
    semilla       : para reproducibilidad (None = aleatoria)

    Retorna
    -------
    (waypoints, tiempo_s)  — waypoints es lista de (x, y), o (None, t)
    """
    if semilla is not None:
        random.seed(semilla)

    ancho = scene['ancho']
    alto  = scene['alto']
    t0    = time.perf_counter()

    # Construir todos los obstáculos inflados una sola vez
    obs_inf = _construir_obstaculos(scene, robot_radio)

    # Validar que q0 y qf estén en espacio libre
    for q, nombre in [(q0_xy, 'q0'), (qf_xy, 'qf')]:
        if not _punto_libre(q, obs_inf):
            raise ValueError(
                f"[RRT] {nombre}={q} cae dentro de un obstáculo inflado. "
                f"Reducir robot_radio ({robot_radio} m) o corregir la escena.")

    # Árbol: lista de nodos y tabla de padres
    nodos  = [q0_xy]
    padres = {0: None}    # índice → índice del padre

    for it in range(max_iter):

        # ── 1. Muestreo con sesgo hacia la meta ──────────────────────────────
        if random.random() < sesgo:
            q_rand = qf_xy
        else:
            q_rand = (random.uniform(0.0, ancho),
                      random.uniform(0.0, alto))

        # ── 2. Nodo más cercano del árbol ─────────────────────────────────────
        idx_near = min(range(len(nodos)),
                       key=lambda i: _distancia(nodos[i], q_rand))
        q_near   = nodos[idx_near]

        # ── 3. Extensión ──────────────────────────────────────────────────────
        q_new = _extender(q_near, q_rand, delta)

        # ── 4. Verificación de colisión ───────────────────────────────────────
        if not _es_libre(q_near, q_new, obs_inf):
            continue

        # ── 5. Agregar al árbol ───────────────────────────────────────────────
        idx_new = len(nodos)
        nodos.append(q_new)
        padres[idx_new] = idx_near

        # ── 6. Comprobar si alcanzamos la meta ────────────────────────────────
        if (_distancia(q_new, qf_xy) <= tol_meta
                and _es_libre(q_new, qf_xy, obs_inf)):
            idx_qf = len(nodos)
            nodos.append(qf_xy)
            padres[idx_qf] = idx_new
            dt     = time.perf_counter() - t0
            camino = _reconstruir(nodos, padres, idx_qf)
            return camino, dt

    dt = time.perf_counter() - t0
    return None, dt


def _reconstruir(nodos: list, padres: dict, idx_final: int) -> list:
    """Recorre el árbol hacia atrás para obtener el camino desde q0 a qf."""
    camino, idx = [], idx_final
    while idx is not None:
        camino.append(nodos[idx])
        idx = padres[idx]
    camino.reverse()
    return camino


# ══════════════════════════════════════════════════════════════════════════════
# SUAVIZADO GREEDY SHORTCUT
# ══════════════════════════════════════════════════════════════════════════════

def suavizar(waypoints: list,
             scene: dict,
             robot_radio: float,
             max_pasadas: int = 500) -> list:
    """
    Elimina waypoints intermedios cuando el atajo directo es libre de obstáculos.
    Equivalente continuo al suavizado Bresenham del proyecto 2.

    Complejidad por pasada: O(n) — converge rápido para caminos RRT típicos.
    """
    obs_inf = _construir_obstaculos(scene, robot_radio)
    suave   = list(waypoints)

    for _ in range(max_pasadas):
        cambio = False
        i = 0
        while i < len(suave) - 2:
            if _es_libre(suave[i], suave[i + 2], obs_inf):
                suave.pop(i + 1)   # eliminar waypoint intermedio redundante
                cambio = True
            else:
                i += 1
        if not cambio:
            break   # convergió, no hay más mejoras posibles

    return suave