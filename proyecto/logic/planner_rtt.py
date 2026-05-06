"""
planner_rrt.py  –  Proyecto 3, ISIS4826 Robótica Móvil
Planificador RRT para espacio continuo (sin cuadrícula).

Colocar en:  proyecto/logic/planner_rrt.py
Dependencia: pip install shapely
"""
import math
import random
import time
from shapely.geometry import LineString, Polygon, Point


# ── Constantes (ajustables) ───────────────────────────────────────────────────
DELTA        = 0.30   # m  – longitud de extensión por iteración
TOL_META     = 0.25   # m  – radio de captura de la meta
MAX_ITER     = 12_000 # iteraciones máximas
SESGO_META   = 0.10   # fracción de muestras dirigidas a qf (10 %)


# ── Geometría auxiliar ────────────────────────────────────────────────────────

def _inflar_obstaculo(p1: tuple, p2: tuple, radio: float) -> Polygon:
    """
    Recibe las dos esquinas de un obstáculo rectangular del proyecto 2
    y devuelve el polígono inflado por 'radio' (suma de Minkowski aprox).
    """
    x1, y1 = p1
    x2, y2 = p2
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    rect = Polygon([(xmin, ymin), (xmax, ymin),
                    (xmax, ymax), (xmin, ymax)])
    return rect.buffer(radio, resolution=8)


def _inflar_paredes(ancho: float, alto: float, radio: float) -> list:
    """
    Devuelve 4 polígonos (franjas) que representan las paredes del escenario
    infladas hacia el interior por 'radio'.
    """
    # izquierda, derecha, abajo, arriba
    return [
        Polygon([(-1, -1),      (radio, -1),       (radio, alto+1),  (-1, alto+1)]),
        Polygon([(ancho-radio, -1), (ancho+1, -1),  (ancho+1, alto+1), (ancho-radio, alto+1)]),
        Polygon([(-1, -1),      (ancho+1, -1),     (ancho+1, radio),  (-1, radio)]),
        Polygon([(-1, alto-radio), (ancho+1, alto-radio), (ancho+1, alto+1), (-1, alto+1)]),
    ]


def _distancia(a: tuple, b: tuple) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _extender(q_near: tuple, q_rand: tuple, delta: float) -> tuple:
    d = _distancia(q_near, q_rand)
    if d < 1e-9:
        return q_near
    t = min(delta / d, 1.0)
    return (q_near[0] + t * (q_rand[0] - q_near[0]),
            q_near[1] + t * (q_rand[1] - q_near[1]))


def _es_libre(p1: tuple, p2: tuple, obs_inflados: list) -> bool:
    """True si el segmento p1→p2 no intersecta ningún obstáculo inflado."""
    seg = LineString([p1, p2])
    return not any(seg.intersects(o) for o in obs_inflados)


# ── Algoritmo RRT ─────────────────────────────────────────────────────────────

def rrt(q0_xy: tuple, qf_xy: tuple, scene: dict,
        robot_radio: float,
        delta: float      = DELTA,
        tol_meta: float   = TOL_META,
        max_iter: int     = MAX_ITER,
        sesgo: float      = SESGO_META,
        semilla: int      = None) -> tuple:
    """
    Parámetros
    ----------
    q0_xy, qf_xy  : (x, y) inicio y fin en metros
    scene         : dict devuelto por _parsear_escena() del proyecto 2
    robot_radio   : radio de inflado (usar ROBOT_RADIO del nodo)

    Retorna
    -------
    (waypoints, tiempo_s)  donde waypoints es lista de (x,y), o (None, t)
    """
    if semilla is not None:
        random.seed(semilla)

    ancho, alto = scene['ancho'], scene['alto']
    t0 = time.perf_counter()

    # Construir obstáculos inflados (obstáculos + paredes)
    obs_inf = [_inflar_obstaculo(o['p1'], o['p2'], robot_radio)
               for o in scene['obstaculos']]
    obs_inf += _inflar_paredes(ancho, alto, robot_radio)

    # Verificar que q0 y qf estén en espacio libre
    for q, nombre in [(q0_xy, 'q0'), (qf_xy, 'qf')]:
        pt = Point(q)
        if any(pt.within(o) for o in obs_inf):
            raise ValueError(
                f"[RRT] {nombre}={q} cae dentro de un obstáculo inflado. "
                f"Verificar radio_robot ({robot_radio} m).")

    nodos  = [q0_xy]
    padres = {0: None}

    for it in range(max_iter):
        # 1. Muestreo con sesgo hacia la meta
        if random.random() < sesgo:
            q_rand = qf_xy
        else:
            q_rand = (random.uniform(0, ancho),
                      random.uniform(0, alto))

        # 2. Nodo más cercano
        idx_near = min(range(len(nodos)),
                       key=lambda i: _distancia(nodos[i], q_rand))
        q_near   = nodos[idx_near]

        # 3. Extender
        q_new = _extender(q_near, q_rand, delta)

        # 4. Verificar colisión
        if not _es_libre(q_near, q_new, obs_inf):
            continue

        # 5. Agregar al árbol
        idx_new = len(nodos)
        nodos.append(q_new)
        padres[idx_new] = idx_near

        # 6. ¿Llegamos a la meta?
        if (_distancia(q_new, qf_xy) <= tol_meta
                and _es_libre(q_new, qf_xy, obs_inf)):
            idx_qf = len(nodos)
            nodos.append(qf_xy)
            padres[idx_qf] = idx_new
            dt = time.perf_counter() - t0

            camino = _reconstruir(nodos, padres, idx_qf)
            return camino, dt

    dt = time.perf_counter() - t0
    return None, dt


def _reconstruir(nodos, padres, idx_final) -> list:
    camino, idx = [], idx_final
    while idx is not None:
        camino.append(nodos[idx])
        idx = padres[idx]
    camino.reverse()
    return camino


# ── Suavizado greedy shortcut ─────────────────────────────────────────────────

def suavizar(waypoints: list, scene: dict,
             robot_radio: float, max_pasadas: int = 500) -> list:
    """
    Elimina waypoints intermedios cuando el atajo directo es libre.
    Equivalente continuo al suavizado Bresenham del proyecto 2.
    """
    obs_inf = [_inflar_obstaculo(o['p1'], o['p2'], robot_radio)
               for o in scene['obstaculos']]
    obs_inf += _inflar_paredes(scene['ancho'], scene['alto'], robot_radio)

    suave = list(waypoints)
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