"""
Planificador de caminos geometricos usando A* sobre una cuadricula.

La cuadricula tiene resolucion CELL_SIZE metros. Cada celda se clasifica como:
  - LIBRE: el robot puede ocuparla
  - SEMI_LIBRE: el robot toca el C-obstaculo (margen)
  - OCUPADA: el robot colisiona con un obstaculo

El robot se modela como un cuadrado de 0.30 m de lado. El radio de expansion
del C-obstaculo es ROBOT_RADIO = 0.15 m.
"""
import math
import heapq

CELL_SIZE = 0.25   # metros por celda
ROBOT_RADIO = 0.20  # metros (expansion conservadora para ejecucion real)
MARGEN_SEMI = 0.10  # metros extra para semi-libre


# ---------------------------------------------------------------------------
# Construccion del C-espacio
# ---------------------------------------------------------------------------

def _world_to_grid(x, y):
    return int(x / CELL_SIZE), int(y / CELL_SIZE)


def _grid_to_world(col, row):
    return (col + 0.5) * CELL_SIZE, (row + 0.5) * CELL_SIZE


def construir_cgrid(escena):
    """
    Construye la cuadricula del C-espacio.

    Retorna grid[col][row] con valores:
      0 = LIBRE, 1 = SEMI_LIBRE, 2 = OCUPADA
    """
    ancho = escena['ancho']
    alto = escena['alto']
    cols = math.ceil(ancho / CELL_SIZE)
    rows = math.ceil(alto / CELL_SIZE)

    grid = [[0] * rows for _ in range(cols)]

    # Marcar C-obstaculos (expansion de obstaculos reales)
    for obs in escena['obstaculos']:
        x1, y1 = obs['pto1']
        x2, y2 = obs['pto2']
        # Asegurar orden correcto
        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)

        # Expansion C-obstaculo (ocupada)
        exp = ROBOT_RADIO
        xmin_c = xmin - exp
        xmax_c = xmax + exp
        ymin_c = ymin - exp
        ymax_c = ymax + exp

        # Expansion adicional para semi-libre
        exp_s = ROBOT_RADIO + MARGEN_SEMI
        xmin_s = xmin - exp_s
        xmax_s = xmax + exp_s
        ymin_s = ymin - exp_s
        ymax_s = ymax + exp_s

        for c in range(cols):
            for r in range(rows):
                cx, cy = _grid_to_world(c, r)
                if xmin_c <= cx <= xmax_c and ymin_c <= cy <= ymax_c:
                    grid[c][r] = 2  # OCUPADA
                elif xmin_s <= cx <= xmax_s and ymin_s <= cy <= ymax_s:
                    if grid[c][r] < 1:
                        grid[c][r] = 1  # SEMI_LIBRE

    # Paredes del escenario (margen desde los bordes)
    for c in range(cols):
        for r in range(rows):
            cx, cy = _grid_to_world(c, r)
            if (cx < ROBOT_RADIO or cx > ancho - ROBOT_RADIO or
                    cy < ROBOT_RADIO or cy > alto - ROBOT_RADIO):
                grid[c][r] = 2

    return grid, cols, rows


# ---------------------------------------------------------------------------
# A*
# ---------------------------------------------------------------------------

def _heuristica(c1, r1, c2, r2):
    # Distancia euclidea en celdas
    return math.sqrt((c2 - c1) ** 2 + (r2 - r1) ** 2)


VECINOS_8 = [
    (-1, 0), (1, 0), (0, -1), (0, 1),
    (-1, -1), (-1, 1), (1, -1), (1, 1)
]


def _astar(grid, cols, rows, start, goal):
    """A* sobre la cuadricula. Evita celdas OCUPADAS (valor 2)."""
    sc, sr = start
    gc, gr = goal

    # Cola de prioridad: (f, g, col, row, parent)
    open_set = []
    heapq.heappush(open_set, (0.0, 0.0, sc, sr, None))

    came_from = {}
    g_score = {(sc, sr): 0.0}
    closed_set = set()

    while open_set:
        f, g, c, r, parent = heapq.heappop(open_set)

        if (c, r) in closed_set:
            continue
        closed_set.add((c, r))
        came_from[(c, r)] = parent

        if (c, r) == (gc, gr):
            # Reconstruir camino
            path = []
            node = (gc, gr)
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path

        for dc, dr in VECINOS_8:
            nc, nr = c + dc, r + dr
            if not (0 <= nc < cols and 0 <= nr < rows):
                continue
            if grid[nc][nr] == 2:  # ocupada
                continue
            if (nc, nr) in closed_set:
                continue

            # Penalizar semi-libre fuertemente para alejarse de paredes
            extra = 5.0 if grid[nc][nr] == 1 else 1.0
            paso = math.sqrt(dc ** 2 + dr ** 2) * extra
            ng = g + paso

            if ng < g_score.get((nc, nr), float('inf')):
                g_score[(nc, nr)] = ng
                h = _heuristica(nc, nr, gc, gr)
                heapq.heappush(open_set, (ng + h, ng, nc, nr, (c, r)))

    return None  # sin solucion


# ---------------------------------------------------------------------------
# Conversion de ruta de celdas a camino geometrico
# ---------------------------------------------------------------------------

def _angulo_entre(c1, r1, c2, r2):
    """Angulo en grados del vector (c1,r1) -> (c2,r2) en el plano XY."""
    dx = (c2 - c1) * CELL_SIZE
    dy = (r2 - r1) * CELL_SIZE
    return math.degrees(math.atan2(dy, dx))


def _simplificar_camino(path_cells):
    """Elimina puntos colineales consecutivos."""
    if len(path_cells) <= 2:
        return path_cells
    resultado = [path_cells[0]]
    for i in range(1, len(path_cells) - 1):
        c0, r0 = path_cells[i - 1]
        c1, r1 = path_cells[i]
        c2, r2 = path_cells[i + 1]
        dir1 = math.atan2(r1 - r0, c1 - c0)
        dir2 = math.atan2(r2 - r1, c2 - c1)
        if abs(math.atan2(math.sin(dir2 - dir1), math.cos(dir2 - dir1))) > 0.01:
            resultado.append(path_cells[i])
    resultado.append(path_cells[-1])
    return resultado


def planificar(escena):
    """
    Planifica el camino de q0 a qf en la escena dada.

    Retorna lista de (x, y, theta_grados) que alterna:
      configuracion de rotacion + configuracion de traslacion.
    El primer elemento es q0, el ultimo es qf (con la orientacion final).
    """
    grid, cols, rows = construir_cgrid(escena)

    q0 = escena['q0']
    qf = escena['qf']

    start = _world_to_grid(q0[0], q0[1])
    goal = _world_to_grid(qf[0], qf[1])

    # Si start o goal estan en celda ocupada, ajustar al centro libre mas cercano
    start = _celda_libre_mas_cercana(grid, cols, rows, start)
    goal = _celda_libre_mas_cercana(grid, cols, rows, goal)

    path_cells = _astar(grid, cols, rows, start, goal)

    if path_cells is None:
        return None

    path_cells = _simplificar_camino(path_cells)

    # Convertir a configuraciones (x, y, theta)
    waypoints = []

    # q0 con su orientacion inicial
    theta_actual = q0[2]
    waypoints.append((q0[0], q0[1], theta_actual))

    for i in range(1, len(path_cells)):
        c_prev, r_prev = path_cells[i - 1]
        c_curr, r_curr = path_cells[i]

        # Angulo necesario para ir de la celda anterior a la actual
        theta_nueva = _angulo_entre(c_prev, r_prev, c_curr, r_curr)

        # Posicion del centro de la celda actual
        x_curr, y_curr = _grid_to_world(c_curr, r_curr)

        # Configuracion de rotacion (mismo x,y, nuevo theta)
        waypoints.append((waypoints[-1][0], waypoints[-1][1], theta_nueva))

        # Configuracion de traslacion (nuevo x,y, mismo theta)
        waypoints.append((x_curr, y_curr, theta_nueva))

    # Ajustar el ultimo punto a las coordenadas exactas de qf
    waypoints[-1] = (qf[0], qf[1], waypoints[-1][2])

    # Rotacion final hasta la orientacion de qf
    waypoints.append((qf[0], qf[1], qf[2]))

    return waypoints


def _celda_libre_mas_cercana(grid, cols, rows, celda):
    """Si la celda esta ocupada busca la celda libre mas cercana."""
    c0, r0 = celda
    if grid[c0][r0] != 2:
        return celda
    radio_max = max(cols, rows)
    for radio in range(1, radio_max):
        for dc in range(-radio, radio + 1):
            for dr in range(-radio, radio + 1):
                if abs(dc) != radio and abs(dr) != radio:
                    continue
                nc, nr = c0 + dc, r0 + dr
                if 0 <= nc < cols and 0 <= nr < rows and grid[nc][nr] != 2:
                    return (nc, nr)
    return celda
