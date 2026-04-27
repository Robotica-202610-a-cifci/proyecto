import math
from geometry_msgs.msg import Twist

VEL_LINEAL = 0.20
VEL_ANGULAR = 0.5
TOL_DIST = 0.20
TOL_ANGLE = 0.05


class PathExecutor:
    _ROTATE = 'ROTATE'
    _TRANSLATE = 'TRANSLATE'

    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.idx = 0
        self.phase = self._ROTATE

    def tick(self, current_x, current_y, current_theta_rad, node):
        if self.idx >= len(self.waypoints):
            self._detener(node)
            return 'COMPLETADO'

        tx, ty, ttheta_deg = self.waypoints[self.idx]
        ttheta_rad = math.radians(ttheta_deg)

        # ---- ROTACION ----
        if self.phase == self._ROTATE:
            error_ang = math.atan2(math.sin(ttheta_rad - current_theta_rad),
                                   math.cos(ttheta_rad - current_theta_rad))
            if abs(error_ang) < TOL_ANGLE:
                self._detener(node)
                self.phase = self._TRANSLATE
                return 'EN_RUTA'
            cmd = Twist()
            cmd.angular.z = max(-VEL_ANGULAR, min(VEL_ANGULAR, 2.0 * error_ang))
            node.cmd_pub.publish(cmd)
            return 'EN_RUTA'

        # ---- TRASLACION con correccion de rumbo ----
        if self.phase == self._TRANSLATE:
            dx = tx - current_x
            dy = ty - current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < TOL_DIST:
                self._detener(node)
                self._advance()
                return 'EN_RUTA'

            # Correccion de rumbo dinamica hacia el waypoint
            angulo_objetivo = math.atan2(dy, dx)
            error_ang = math.atan2(math.sin(angulo_objetivo - current_theta_rad),
                                   math.cos(angulo_objetivo - current_theta_rad))
            vel = max(0.06, min(VEL_LINEAL, 0.4 * dist))
            cmd = Twist()
            cmd.linear.x = vel
            cmd.angular.z = max(-0.3, min(0.3, 1.0 * error_ang))
            node.cmd_pub.publish(cmd)
            return 'EN_RUTA'

        return 'COMPLETADO'

    def _detener(self, node):
        node.cmd_pub.publish(Twist())

    def _advance(self):
        self.idx += 1
        self.phase = self._ROTATE

    @property
    def terminado(self):
        return self.idx >= len(self.waypoints)

    @property
    def progreso(self):
        return f'{self.idx}/{len(self.waypoints)}'
