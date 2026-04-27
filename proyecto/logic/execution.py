"""
Ejecutor de caminos geométricos (Persona 2).

PathExecutor es una máquina de estados que consume una lista de waypoints
(x, y, theta_grados) y llama a los métodos rotar_relativo / mover_relativo
del nodo ROS2 en cada tick del control_loop (10 Hz).

Secuencia por waypoint
-----------------------
1. ROTATE  – rota hasta la orientación del waypoint
2. TRANSLATE – avanza en línea recta hasta la posición del waypoint
Cuando se agotan todos los waypoints pasa a DONE.
"""
import math


class PathExecutor:

    _ROTATE = 'ROTATE'
    _TRANSLATE = 'TRANSLATE'
    _DONE = 'DONE'

    def __init__(self, waypoints):
        """
        waypoints : lista de (x, y, theta_grados)
        """
        self.waypoints = waypoints
        self.idx = 0
        self.phase = self._ROTATE
        self._translate_dist = None  # distancia capturada al inicio del movimiento

    # ------------------------------------------------------------------
    def tick(self, current_x, current_y, current_theta_rad, node):
        """
        Llama a las primitivas del nodo para avanzar un paso.

        Retorna: 'EN_RUTA' | 'COMPLETADO' | 'BLOQUEADO'
        """
        if self.idx >= len(self.waypoints):
            return 'COMPLETADO'

        tx, ty, ttheta_deg = self.waypoints[self.idx]
        ttheta_rad = math.radians(ttheta_deg)

        # ---- FASE ROTACIÓN ----
        if self.phase == self._ROTATE:
            delta_rad = ttheta_rad - current_theta_rad
            # Normalizar a [-π, π]
            delta_rad = math.atan2(math.sin(delta_rad), math.cos(delta_rad))
            delta_deg = math.degrees(delta_rad)

            completado = node.rotar_relativo(delta_deg, tolerancia=0.04)
            if completado:
                self.phase = self._TRANSLATE
            return 'EN_RUTA'

        # ---- FASE TRASLACIÓN ----
        if self.phase == self._TRANSLATE:
            dx = tx - current_x
            dy = ty - current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < 0.06:  # ya llegamos
                self._advance()
                return 'EN_RUTA'

            # Capturar la distancia inicial solo una vez
            if self._translate_dist is None:
                self._translate_dist = dist

            # El robot ya está alineado con el waypoint: avanzar recto (dist_x=dist, dist_y=0)
            estado = node.mover_relativo(self._translate_dist, 0.0,
                                         dist_segura=0.18, vel_lineal=0.3)
            if estado == 'COMPLETADO':
                self._translate_dist = None
                self._advance()
            elif estado == 'BLOQUEADO':
                self._translate_dist = None
                return 'BLOQUEADO'

            return 'EN_RUTA'

        return 'COMPLETADO'

    # ------------------------------------------------------------------
    def _advance(self):
        self.idx += 1
        self.phase = self._ROTATE
        self._translate_dist = None

    @property
    def terminado(self):
        return self.idx >= len(self.waypoints)

    @property
    def progreso(self):
        return f'{self.idx}/{len(self.waypoints)}'
