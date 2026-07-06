#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _wrap(a: float) -> float:
    # Mantiene los ángulos entre -pi y pi para evitar saltos extraños en las matemáticas
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class ObstacleGuardNode(Node):

    def __init__(self):
        super().__init__('obstacle_guard')

        # Distancias clave de seguridad
        self.declare_parameter('stop_distance_m',        0.30)
        self.declare_parameter('slowdown_distance_m',    0.80)
       
        self.declare_parameter('arc_half_width_deg',     60.0)
        self.declare_parameter('front_angle_offset_rad', 0.0)
        
        # Agresividad del giro
        self.declare_parameter('k_avoid_rad_s',          0.6)
        
 
        # Cuántos escaneos limpios necesitamos ver antes de quitar el freno de emergencia
        # (evita que el robot avance y frene tiritando cuando un objeto está al límite)
        self.declare_parameter('clear_scans_required',   3)
        
  
        self.declare_parameter('max_linear_x_m_s',       0.30)
        
        # Filtro de ruido cercano
        # Ignoramos lecturas sospechosamente cerca 
        self.declare_parameter('min_valid_range_m',      0.05)
        
        # Maniobra de escape autónomo
        # Si el usuario suelta el mando y el robot está atrapado muy cerca de algo,
        # retrocede solo para ganar una distancia segura de confort.
        self.declare_parameter('autonomous_enabled',     True)
        self.declare_parameter('silent_threshold_sec',   1.0)  
        self.declare_parameter('back_away_speed_m_s',    0.10) 
        self.declare_parameter('autonomous_rate_hz',     10.0)

        self.stop_d        = self.get_parameter('stop_distance_m').value
        self.slow_d        = self.get_parameter('slowdown_distance_m').value
        self.arc_half      = math.radians(self.get_parameter('arc_half_width_deg').value)
        self.front_off     = self.get_parameter('front_angle_offset_rad').value
        self.k_avoid       = self.get_parameter('k_avoid_rad_s').value
        self.clear_needed  = self.get_parameter('clear_scans_required').value
        self.max_lin_x     = self.get_parameter('max_linear_x_m_s').value
        self.min_valid     = self.get_parameter('min_valid_range_m').value
        self.auto_enabled  = self.get_parameter('autonomous_enabled').value
        self.silent_thr    = self.get_parameter('silent_threshold_sec').value
        self.back_speed    = self.get_parameter('back_away_speed_m_s').value
        self.auto_rate     = self.get_parameter('autonomous_rate_hz').value

        
        if self.slow_d <= self.stop_d:
            self.get_logger().error(
                f"slowdown_distance_m ({self.slow_d}) debe ser mayor que stop_distance_m "
                f"({self.stop_d}). Forzando slow = stop + 0.5"
            )
            self.slow_d = self.stop_d + 0.5

        # Variables de estado 
        self.scan_received        = False
        self.emergency_stop       = True
        self.consecutive_clear    = 0
        self.front_nearest_dist   = float('inf')
        self.front_nearest_angle  = 0.0
        
        # Registramos también el punto más cercano en los 360° para el escape autónomo
        self.nearest_360_dist     = float('inf')
        self.nearest_360_angle    = 0.0

        self.last_cmd_vel_in_t    = 0.0

     
        # Si nos topamos algo justo en el centro, elegimos un lado (izq/der) y lo recordamos.
        # Si no lo guardáramos, el robot temblaría dudando entre girar a un lado u otro.
        self.head_on_sign         = None
        self._head_on_clear_thr   = 0.30   

        self.last_log_t           = 0.0

       
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self.create_subscription(Twist, '/cmd_vel_in', self._on_cmd_vel_in, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        
        if self.auto_enabled:
            self.create_timer(1.0 / self.auto_rate, self._autonomous_tick)

        self.get_logger().info(
            f"Escudo anti-colisiones activado. "
            f"Parada={self.stop_d:.2f}m  Freno={self.slow_d:.2f}m "
            f"(±{math.degrees(self.arc_half):.0f}°)  "
            f"k_avoid={self.k_avoid:.2f}  max_lin={self.max_lin_x:.2f}m/s  "
            f"Scans limpios requeridos={self.clear_needed}  "
            f"AUTO={'ON' if self.auto_enabled else 'OFF'} "
            f"(inactivo>{self.silent_thr:.1f}s → escape@{self.back_speed:.2f}m/s)"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _user_is_silent(self) -> bool:
      
        return (self._now() - self.last_cmd_vel_in_t) > self.silent_thr

    def _on_scan(self, msg: LaserScan):
 
        self.scan_received = True

        nearest_360         = float('inf')
        nearest_360_ang     = 0.0
        nearest_front       = float('inf')
        nearest_front_angle = 0.0

        for i, r in enumerate(msg.ranges):
            # Limpiamos basura del sensor: valores infinitos, rebotes ultra cercanos o fuera de rango
            if not math.isfinite(r):  continue
            if r < self.min_valid:    continue
            if r > msg.range_max:     continue

            angle = _wrap(msg.angle_min + i * msg.angle_increment)
            angle_from_front = _wrap(angle - self.front_off)

        
            if r < nearest_360:
                nearest_360     = r
                nearest_360_ang = angle_from_front

            # Rastreando solo nuestro frente
            if abs(angle_from_front) <= self.arc_half:
                if r < nearest_front:
                    nearest_front       = r
                    nearest_front_angle = angle_from_front

        self.nearest_360_dist    = nearest_360
        self.nearest_360_angle   = nearest_360_ang
        self.front_nearest_dist  = nearest_front
        self.front_nearest_angle = nearest_front_angle

        # Si el obstáculo ya se apartó suficiente del centro, olvidamos hacia qué lado habíamos
        # decidido esquivarlo para empezar limpios el siguiente encuentro.
        if math.isfinite(nearest_front):
            if abs(nearest_front_angle) > self._head_on_clear_thr:
                self.head_on_sign = None

        if nearest_360 < self.stop_d:
            self.emergency_stop    = True
            self.consecutive_clear = 0
        else:
            self.consecutive_clear += 1
            if self.consecutive_clear >= self.clear_needed:
                self.emergency_stop = False

    def _on_cmd_vel_in(self, msg: Twist):
        
        self.last_cmd_vel_in_t = self._now()

        out = Twist()
        if not self.scan_received:
            self._publish_zero(out, "Esperando el primer /scan del LiDAR...")
            return

        
        if self.emergency_stop:
            self._publish_zero(out,
                f"PARADA DE EMERGENCIA (obstáculo a <{self.stop_d:.2f}m)")
            return

      
        out.linear  = msg.linear
        out.angular = msg.angular

        #  Si el usuario intenta avanzar y hay algo enfrente, intervenimos
        if msg.linear.x > 0.01 and math.isfinite(self.front_nearest_dist):
            d = self.front_nearest_dist
            if d < self.slow_d:
                # Si estamos dentro de la zona de advertencia, reducimos la marcha proporcionalmente
                if d <= self.stop_d:
                    scale = 0.0
                else:
                    scale = (d - self.stop_d) / (self.slow_d - self.stop_d)
                out.linear.x = msg.linear.x * scale

                # Calculamos el giro de esquiva: cuanto más cerca el obstáculo, más fuerte giramos
                
                proximity = 1.0 - scale
                a = self.front_nearest_angle
                if abs(a) > 0.05:
                    sign = math.copysign(1.0, a)
                    bias = -sign * self.k_avoid * proximity
                else:
                    
                    if abs(msg.angular.z) > 0.01:
                        sign = math.copysign(1.0, msg.angular.z)
                    else:
                        sign = self._get_random_head_on_sign()
                    bias = sign * self.k_avoid * proximity

                out.angular.z = msg.angular.z + bias

                self._log_throttled(
                    f"Esquivando: d={d:.2f}m ángulo={math.degrees(a):+5.0f}° "
                    f"escala={scale:.2f} sesgo={bias:+.2f} "
                    f"lin.x={out.linear.x:+.2f} ang.z={out.angular.z:+.2f}"
                )

        if out.linear.x >  self.max_lin_x: out.linear.x =  self.max_lin_x
        if out.linear.x < -self.max_lin_x: out.linear.x = -self.max_lin_x

        self.pub_cmd_vel.publish(out)

    def _autonomous_tick(self):
       
        if not self.scan_received:
            return
        if not self._user_is_silent(): 
            return                                    
        if not math.isfinite(self.nearest_360_dist):
            return
        if self.nearest_360_dist >= self.stop_d:
            return                                    
        
        d     = self.nearest_360_dist
        angle = self.nearest_360_angle               

        # Decidimos si avanzar o retroceder mirando en qué mitad del robot está el problema:
        # cos(angle) > 0 -> Peligro en la mitad delantera -> Retrocedemos (linear.x < 0)
        # cos(angle) < 0 -> Peligro en la mitad trasera   -> Avanzamos    (linear.x > 0)
        cos_a = math.cos(angle)
        if abs(cos_a) < 0.05:
            # El obstáculo está casi a 90° (en un costado). Por seguridad, retrocedemos despacio
            linear_sign = -1.0
        else:
            linear_sign = -math.copysign(1.0, cos_a)

        # Calculamos la velocidad
        proximity = max(0.5, min(1.5, (self.stop_d - d) / self.stop_d + 1.0))
        linear_x  = linear_sign * self.back_speed * proximity

        
        sin_a = math.sin(angle)
        if abs(sin_a) > 0.15:
            angular_z = -math.copysign(1.0, sin_a) * self.k_avoid * 0.5
        else:
          
            angular_z = self._get_random_head_on_sign() * self.k_avoid * 0.5

        twist = Twist()
        twist.linear.x  = max(-self.max_lin_x, min(self.max_lin_x, linear_x))
        twist.angular.z = angular_z
        self.pub_cmd_vel.publish(twist)

        self._log_throttled(
            f"AUTO-ESCAPE: d={d:.2f}m ángulo={math.degrees(angle):+5.0f}° "
            f"lin.x={twist.linear.x:+.2f} ang.z={twist.angular.z:+.2f}"
        )

    def _get_random_head_on_sign(self) -> float:
        """Devuelve el sentido de giro (-1 o +1) para el encuentro actual.
        Si es la primera vez que lo preguntamos en este obstáculo, lo echa a suertes."""
        if self.head_on_sign is None:
            self.head_on_sign = random.choice([-1.0, 1.0])
            self.get_logger().info(
                f"Obstáculo frontal detectado: decidiéndose por esquivar hacia el sentido {self.head_on_sign:+.0f}"
            )
        return self.head_on_sign

    def _publish_zero(self, out: Twist, reason: str):
        out.linear.x = out.linear.y = out.linear.z = 0.0
        out.angular.x = out.angular.y = out.angular.z = 0.0
        self.pub_cmd_vel.publish(out)
        self._log_throttled(reason)

    def _log_throttled(self, text: str):
        now = self._now()
        if (now - self.last_log_t) > 0.5:
            self.last_log_t = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
