#!/usr/bin/env python3

import csv
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


BALANCE_STATE_LABELS = [
    'pitch_deg',
    'dynamic_target_deg',
    'tilt_error_deg',
    'commanded_torque_nm',
    'robot_velocity_mps',
    'pitch_offset_deg',
    'loop_hz',
    'engaged',
    'target_velocity_mps',
    'target_yaw_rate_dps',
    'gyro_yaw_dps',
]


class PIDExperimentNode(Node):

    def __init__(self):
        super().__init__('pid_experiment')

        self.declare_parameter('exp', 1)
        self.declare_parameter('out_root', './pid_data')
        self.declare_parameter('step_velocity_mps',     0.20)
        self.declare_parameter('tracking_velocity_mps', 0.15)
        self.declare_parameter('pre_roll_sec',          3.0)
        self.declare_parameter('step_hold_sec',         5.0)
        self.declare_parameter('tracking_hold_sec',    10.0)
        self.declare_parameter('post_roll_sec',         3.0)
        self.declare_parameter('disturbance_total_sec', 20.0)
        self.declare_parameter('cmd_publish_rate_hz',  20.0)

        self.exp_num   = int(self.get_parameter('exp').value)
        self.out_root  = self.get_parameter('out_root').value
        self.step_v    = self.get_parameter('step_velocity_mps').value
        self.track_v   = self.get_parameter('tracking_velocity_mps').value
        self.pre_t     = self.get_parameter('pre_roll_sec').value
        self.step_t    = self.get_parameter('step_hold_sec').value
        self.track_t   = self.get_parameter('tracking_hold_sec').value
        self.post_t    = self.get_parameter('post_roll_sec').value
        self.dist_t    = self.get_parameter('disturbance_total_sec').value
        self.cmd_hz    = self.get_parameter('cmd_publish_rate_hz').value

        if self.exp_num not in (1, 2, 3):
            self.get_logger().fatal(f"Valor de experimento inválido (exp={self.exp_num}). Debe ser 1, 2 o 3.")
            raise SystemExit(1)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(self.out_root, f'exp{self.exp_num}_{timestamp}')
        os.makedirs(self.out_dir, exist_ok=True)

        self.balance_data = []
        self.cmd_vel_data = []

        self.create_subscription(Float32MultiArray, '/balance/state',
                                 self._on_balance, 50)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 50)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # La publicación continua evita que se active el timeout de seguridad de cmd_vel
        # en el nodo de balance (por defecto a 1.0 s) a mitad del experimento.
        self.current_setpoint   = 0.0
        self.publishing_active  = False
        self.create_timer(1.0 / self.cmd_hz, self._publish_setpoint_tick)

        self.get_logger().info(
            f"Experimento {self.exp_num} listo para iniciar. Directorio de salida: {self.out_dir}"
        )

    def _on_balance(self, msg: Float32MultiArray):
        t = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.data) >= 11:
            self.balance_data.append((t, list(msg.data[:11])))

    def _on_cmd_vel(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_vel_data.append((t, msg.linear.x, msg.angular.z))

    def _publish_setpoint_tick(self):
        if not self.publishing_active:
            return
        twist = Twist()
        twist.linear.x = float(self.current_setpoint)
        self.pub_cmd_vel.publish(twist)

    def run_experiment(self) -> bool:
        self.get_logger().info("Esperando telemetría en /balance/state...")
        if not self._wait_for_first_balance(timeout_sec=5.0):
            self.get_logger().error(
                "No se han recibido datos en /balance/state. ¿Está en ejecución el nodo cascade_balance_node?"
            )
            return False

        engaged = self.balance_data[-1][1][7] > 0.5
        if not engaged:
            self.get_logger().warn(
                "El control de balance está DESACTIVADO. Pulsa el Triángulo en el mando  primero."
            )

        if self.exp_num == 1:
            descr = f"Respuesta al escalón: 0 → {self.step_v:.2f} m/s → 0"
            total = self.pre_t + self.step_t + self.post_t
        elif self.exp_num == 2:
            descr = "Rechazo a perturbaciones (empujar el robot manualmente durante el registro)"
            total = self.dist_t
        else:
            descr = f"Seguimiento de velocidad constante a {self.track_v:.2f} m/s durante {self.track_t:.0f}s"
            total = self.pre_t + self.track_t + self.post_t

        print()
        print("=" * 60)
        print(f"  EXPERIMENTO {self.exp_num}: {descr}")
        print(f"  Duración estimada: ~{total:.0f} s")
        print(f"  Directorio salida: {self.out_dir}")
        print("-" * 60)
        print("  Comprueba el robot.")
        print("  Botón x en el mando = desconexión de emergencia.")
        print("=" * 60)
        try:
            input("  Pulsa ENTER para comenzar (o Ctrl+C para cancelar): ")
        except (KeyboardInterrupt, EOFError):
            return False

        if   self.exp_num == 1: self._run_step()
        elif self.exp_num == 2: self._run_disturbance()
        elif self.exp_num == 3: self._run_tracking()
        return True

    def _run_step(self):
        t0 = time.time()
        self.publishing_active = True
        self.current_setpoint  = 0.0
        self._log_phase(t0, f"Estabilización previa {self.pre_t:.1f}s @ 0 m/s")
        self._wait(self.pre_t)

        self.current_setpoint = self.step_v
        self._log_phase(t0, f"ESCALÓN ARRIBA → {self.step_v:+.2f} m/s ({self.step_t:.1f}s)")
        self._wait(self.step_t)

        self.current_setpoint = 0.0
        self._log_phase(t0, f"ESCALÓN ABAJO  → 0 m/s ({self.post_t:.1f}s)")
        self._wait(self.post_t)

        self.publishing_active = False
        self._log_phase(t0, "Prueba finalizada.")

    def _run_disturbance(self):
        t0 = time.time()
        self.publishing_active = True
        self.current_setpoint  = 0.0
        self._log_phase(t0,
            f"Registrando durante {self.dist_t:.1f}s — puedes empujar el robot ahora "
            "(hacia adelante, hacia atrás, etc., dejando ~3s entre cada empuje)."
        )
        self._wait(self.dist_t)
        self.publishing_active = False
        self._log_phase(t0, "Prueba finalizada.")

    def _run_tracking(self):
        t0 = time.time()
        self.publishing_active = True
        self.current_setpoint  = 0.0
        self._log_phase(t0, f"Estabilización previa {self.pre_t:.1f}s @ 0 m/s")
        self._wait(self.pre_t)

        self.current_setpoint = self.track_v
        self._log_phase(t0, f"SEGUIMIENTO → {self.track_v:+.2f} m/s ({self.track_t:.1f}s)")
        self._wait(self.track_t)

        self.current_setpoint = 0.0
        self._log_phase(t0, f"PARADA → 0 m/s ({self.post_t:.1f}s)")
        self._wait(self.post_t)

        self.publishing_active = False
        self._log_phase(t0, "Prueba finalizada.")

    def _log_phase(self, t0: float, msg: str):
        self.get_logger().info(f"[{time.time() - t0:5.1f}s] {msg}")

    def _wait(self, duration: float):
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def _wait_for_first_balance(self, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and (time.time() - start < timeout_sec):
            if len(self.balance_data) > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def write_data(self):
        bal_path = os.path.join(self.out_dir, 'balance_state.csv')
        with open(bal_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['timestamp_sec'] + BALANCE_STATE_LABELS)
            for t, row in self.balance_data:
                w.writerow([f"{t:.6f}"] + [f"{v:.6f}" for v in row])

        cmd_path = os.path.join(self.out_dir, 'cmd_vel.csv')
        with open(cmd_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['timestamp_sec', 'linear_x', 'angular_z'])
            for t, lx, az in self.cmd_vel_data:
                w.writerow([f"{t:.6f}", f"{lx:.6f}", f"{az:.6f}"])

        meta = {
            'experiment': self.exp_num,
            'description': {
                1: 'Respuesta al escalón: 0 → velocidad_escalon → 0',
                2: 'Rechazo a perturbaciones (empujes manuales)',
                3: 'Seguimiento de velocidad constante',
            }[self.exp_num],
            'parameters': {
                'step_velocity_mps':     self.step_v,
                'tracking_velocity_mps': self.track_v,
                'pre_roll_sec':          self.pre_t,
                'step_hold_sec':         self.step_t,
                'tracking_hold_sec':     self.track_t,
                'post_roll_sec':         self.post_t,
                'disturbance_total_sec': self.dist_t,
                'cmd_publish_rate_hz':   self.cmd_hz,
            },
            'samples': {
                'balance_state': len(self.balance_data),
                'cmd_vel':       len(self.cmd_vel_data),
            },
            'recording_started_sec': self.balance_data[0][0]  if self.balance_data else None,
            'recording_ended_sec':   self.balance_data[-1][0] if self.balance_data else None,
        }
        with open(os.path.join(self.out_dir, 'metadata.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(
            f"Se han guardado {len(self.balance_data)} muestras de balance_state y "
            f"{len(self.cmd_vel_data)} muestras de cmd_vel en {self.out_dir}"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PIDExperimentNode()
    except SystemExit:
        rclpy.shutdown()
        return

    interrupted = False
    try:
        node.run_experiment()
    except KeyboardInterrupt:
        node.get_logger().warn("Experimento interrumpido por el usuario. Guardando los datos recopilados...")
        interrupted = True
        node.publishing_active = False
        node.pub_cmd_vel.publish(Twist())
    finally:
        try:
            node.write_data()
        except Exception as e:
            node.get_logger().error(f"Error al escribir los archivos de datos: {e}")
        node.destroy_node()
        rclpy.shutdown()
        if interrupted:
            print("\nExperimento cancelado — se han guardado los datos parciales.")


if __name__ == '__main__':
    main()
