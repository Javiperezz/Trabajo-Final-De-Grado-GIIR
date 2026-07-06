#!/usr/bin/env python3
import math
import struct
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

import odrive
from odrive.enums import (
    CONTROL_MODE_TORQUE_CONTROL,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
)
from smbus2 import SMBus


BNO055_ADDRESS = 0x28
SERIAL_0 = "3672386B3131"        # rueda izquierda
SERIAL_1 = "368838693131"        # rueda derecha

PITCH_EULER_INDEX = 2
GYRO_PITCH_INDEX  = 0
GYRO_YAW_INDEX    = 2


DIR_ODRV0 =  1.0
DIR_ODRV1 = -1.0

# direccion encoder
ENC_DIR_LEFT  = -1.0
ENC_DIR_RIGHT =  1.0


WHEEL_RADIUS_M = 0.08
WHEEL_BASE_M   = 0.35
TPS_TO_MPS     = 2.0 * math.pi * WHEEL_RADIUS_M  # turns/sec → m/s


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class CascadeBalanceNode(Node):

    def __init__(self):
        super().__init__('chappie_balance')

        self._declare_parameters()
        self._read_parameters()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.bus = SMBus(1)
        self.odrv0 = None
        self.odrv1 = None
        self._init_imu()
        self._connect_odrives()

        # empieza en el suelo 
        self.engaged = False
        self._set_motors_idle()
        self.get_logger().info(
            
        )

        # Sensor
        self.acc_err_tilt = 0.0
        self.acc_err_vel  = 0.0
        self.acc_err_yaw  = 0.0
        self.last_balance_time = self.get_clock().now().nanoseconds * 1e-9

      
        self.cmd_linear_x  = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

        # Lectua sensores
        self.pitch_deg     = 0.0
        self.gyro_pitch    = 0.0
        self.gyro_yaw      = 0.0
        self.robot_velocity = 0.0
        self.dynamic_target = self.target_pitch_deg
        self.pitch_offset   = 0.0
        self.commanded_torque = 0.0
        self.target_velocity = 0.0
        self.target_yaw_rate = 0.0
        
        # Wheel state for /joint_states + /odom
        self.left_wheel_angle  = 0.0
        self.right_wheel_angle = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = self.last_balance_time

     
        self._hz_count = 0
        self._hz_last  = self.last_balance_time
        self.loop_hz   = 0.0
        
        #filtro para la velocidad 
        
        self.vel_filter_alpha = self.get_parameter('velocity_filter_alpha').value
        self._vel_mean_filt = 0.0
        self._vel_diff_filt = 0.0
      
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_imu          = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.pub_odom         = self.create_publisher(Odometry, '/odom', sensor_qos)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_balance      = self.create_publisher(Float32MultiArray, '/balance/state', 10)

        self.create_service(Trigger, '/chappie/engage',    self._on_engage)
        self.create_service(Trigger, '/chappie/disengage', self._on_disengage)

        self.create_timer(1.0 / self.balance_rate_hz, self._balance_step)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_step)
        self.create_timer(0.1, self._cmd_vel_timeout_check)

        self.get_logger().info(
            f"Ready.  TILT Kp={self.kp_tilt:.3f} Ki={self.ki_tilt:.3f} Kd={self.kd_tilt:.3f}  "
            f"VEL Kp={self.kp_vel:.2f} Ki={self.ki_vel:.2f}  "
            f"YAW Kp={self.kp_yaw:.3f}  "
            f"target_pitch={self.target_pitch_deg:.2f}°"
        )

    #valores de la cascada
    def _declare_parameters(self):

        self.declare_parameter('velocity_filter_alpha', 0.10)

       
        self.declare_parameter('kp_tilt', 0.19)
        self.declare_parameter('ki_tilt', 0.070)
        self.declare_parameter('kd_tilt', 0.023)
        self.declare_parameter('max_i_sum_tilt', 50.0)

        # PI de velocidad
        self.declare_parameter('kp_vel', 1.5)
        self.declare_parameter('ki_vel', 0.5)
        self.declare_parameter('max_i_sum_vel', 10.0)
        self.declare_parameter('max_pitch_offset_deg', 5.0)

        # poicion horizontal PI
        self.declare_parameter('kp_yaw', 0.02)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('max_i_sum_yaw', 50.0)
        self.declare_parameter('max_steering_torque', 0.8)

        
        self.declare_parameter('target_pitch_deg', -9.25)

       
        self.declare_parameter('max_torque', 3.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('max_target_velocity', 1.0)        # m/s
        self.declare_parameter('cmd_vel_timeout_sec', 1.0)

        # Frecuencias
        self.declare_parameter('balance_rate_hz', 200.0)
        self.declare_parameter('publish_rate_hz', 50.0)

      
        self.declare_parameter('imu_calibration_path', 'imu_calibration.json')

    def _read_parameters(self):
        g = lambda n: self.get_parameter(n).value
        self.kp_tilt = g('kp_tilt')
        self.ki_tilt = g('ki_tilt')
        self.kd_tilt = g('kd_tilt')
        self.max_i_sum_tilt = g('max_i_sum_tilt')

        self.kp_vel = g('kp_vel')
        self.ki_vel = g('ki_vel')
        self.max_i_sum_vel = g('max_i_sum_vel')
        self.max_pitch_offset_deg = g('max_pitch_offset_deg')

        self.kp_yaw = g('kp_yaw')
        self.ki_yaw = g('ki_yaw')
        self.max_i_sum_yaw = g('max_i_sum_yaw')
        self.max_steering_torque = g('max_steering_torque')

        self.target_pitch_deg = g('target_pitch_deg')
        self.max_torque = g('max_torque')
        self.max_angle_deg = g('max_angle_deg')
        self.max_target_velocity = g('max_target_velocity')
        self.cmd_vel_timeout_sec = g('cmd_vel_timeout_sec')
        self.balance_rate_hz = g('balance_rate_hz')
        self.publish_rate_hz = g('publish_rate_hz')


    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        try:
            self._read_parameters()
            changed = ", ".join(f"{p.name}={p.value}" for p in params)
            self.get_logger().info(f"Parámetros actualizados en vivo: {changed}")
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f"Fallo al actualizar parámetros: {e}")
            return SetParametersResult(successful=False, reason=str(e))

   

    def _init_imu(self):
        calib_path = self.get_parameter('imu_calibration_path').value
        if os.path.exists(calib_path):
            try:
                with open(calib_path, 'r') as f:
                    calib = json.load(f)
                self.bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x00)
                time.sleep(0.05)
                self.bus.write_i2c_block_data(BNO055_ADDRESS, 0x55, calib)
                time.sleep(0.05)
                self.get_logger().info("IMU calibration loaded.")
            except Exception as e:
                self.get_logger().warn(f"IMU calibration load failed: {e}")
        try:
            self.bus.write_byte_data(BNO055_ADDRESS, 0x3D, 0x08)
            time.sleep(0.6)
            self.get_logger().info("BNO055 ready (IMU mode 0x08).")
        except Exception as e:
            self.get_logger().error(f"IMU init failed: {e}")

    def _connect_odrives(self):
        self.get_logger().info("Connecting to ODrives...")
        self.odrv0 = odrive.find_any(serial_number=SERIAL_0)
        self.odrv1 = odrive.find_any(serial_number=SERIAL_1)
        for odrv in (self.odrv0, self.odrv1):
            try:
                odrv.axis1.motor.config.current_lim = 40.0
                odrv.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            except Exception as e:
                self.get_logger().error(f"ODrive configure error: {e}")
        self.get_logger().info("ODrives connected (will engage on request).")

    def _set_motors_engaged(self):
        for odrv in (self.odrv0, self.odrv1):
            try:
                odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            except Exception as e:
                self.get_logger().error(f"engage error: {e}")

    def _set_motors_idle(self):
        for odrv in (self.odrv0, self.odrv1):
            try:
                odrv.axis1.controller.input_torque = 0.0
                odrv.axis1.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass


    def _on_engage(self, request, response):
        if self.engaged:
            response.success = False
            response.message = "Already engaged."
            return response
       
        sensors = self._read_imu()
        if sensors is None:
            response.success = False
            response.message = "Lectura de la IMu fallida , no podemos realizar engage."
            return response
        pitch, _, _ = sensors
   
        if abs(pitch - self.target_pitch_deg) > 45.0:
            response.success = False
            response.message = (
                f"Robot pitch es {pitch:.1f}°, objetivo es {self.target_pitch_deg:.1f}°. "
                "Ponlo depie antes de activarlo"
            )
            return response

      
        self.acc_err_tilt = 0.0
        self.acc_err_vel  = 0.0
        self.acc_err_yaw  = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

        self._set_motors_engaged()
        self.engaged = True
        response.success = True
        response.message = f"Engaged. pitch={pitch:.2f}°"
        self.get_logger().info(response.message)
        return response

    def _on_disengage(self, request, response):
        if not self.engaged:
            response.success = False
            response.message = "Already disengaged."
            return response
        self.engaged = False
        self._set_motors_idle()
        response.success = True
        response.message = "Disengaged — motors idle."
        self.get_logger().info(response.message)
        return response

   
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_linear_x  = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

  
    def _read_imu(self):
        try:
            raw_e = self.bus.read_i2c_block_data(BNO055_ADDRESS, 0x1A, 6)
            euler = struct.unpack('<hhh', bytes(raw_e))
            pitch = -(euler[PITCH_EULER_INDEX] / 16.0)
            raw_g = self.bus.read_i2c_block_data(BNO055_ADDRESS, 0x14, 6)
            gyros = struct.unpack('<hhh', bytes(raw_g))
            return pitch, gyros[GYRO_PITCH_INDEX] / 16.0, gyros[GYRO_YAW_INDEX] / 16.0
        except Exception:
            return None

    def _read_wheel_velocity(self):
        """Returns (filtered_mean_mps, filtered_diff_mps).
        Mean = (vL+vR)/2 = forward velocity (positive = forward).
        Diff = vL - vR = yaw-rate-related quantity (signed).
        Both are low-pass filtered with a single-pole IIR to suppress
        encoder quantization noise. Filter time constant ≈ 1/(alpha·f_loop)."""
        try:
            v_left_tps  = self.odrv0.axis1.encoder.vel_estimate * ENC_DIR_LEFT
            v_right_tps = self.odrv1.axis1.encoder.vel_estimate * ENC_DIR_RIGHT
        except Exception:
            # On hardware error, fade filter state toward zero (don't snap)
            self._vel_mean_filt *= (1.0 - self.vel_filter_alpha)
            self._vel_diff_filt *= (1.0 - self.vel_filter_alpha)
            return self._vel_mean_filt, self._vel_diff_filt

        v_left_mps  = v_left_tps  * TPS_TO_MPS
        v_right_mps = v_right_tps * TPS_TO_MPS
        raw_mean = 0.5 * (v_left_mps + v_right_mps)
        raw_diff = v_left_mps - v_right_mps

        a = self.vel_filter_alpha
        self._vel_mean_filt = (1.0 - a) * self._vel_mean_filt + a * raw_mean
        self._vel_diff_filt = (1.0 - a) * self._vel_diff_filt + a * raw_diff
        return self._vel_mean_filt, self._vel_diff_filt


    def _cmd_vel_timeout_check(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.last_cmd_time) > self.cmd_vel_timeout_sec:
            if self.cmd_linear_x != 0.0 or self.cmd_angular_z != 0.0:
                self.cmd_linear_x  = 0.0
                self.cmd_angular_z = 0.0
                self.get_logger().warn("/cmd_vel timeout — zeroing commands.")

    def _balance_step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_balance_time
        self.last_balance_time = now
        if dt <= 0:
            dt = 1.0 / self.balance_rate_hz

        # Loop-rate bookkeeping
        self._hz_count += 1
        if now - self._hz_last >= 0.5:
            self.loop_hz = self._hz_count / (now - self._hz_last)
            self._hz_count = 0
            self._hz_last = now

        sensors = self._read_imu()
        if sensors is None:
            if self.engaged:
                self._set_motors_idle()
                self.engaged = False
                self.get_logger().error("Lectura de la Imu fallida.")
            return
        pitch, gyro_pitch, gyro_yaw = sensors
        self.pitch_deg  = pitch
        self.gyro_pitch = gyro_pitch
        self.gyro_yaw   = gyro_yaw

        self.robot_velocity, _ = self._read_wheel_velocity()

        
        if not self.engaged:
            return


        target_velocity = max(min(self.cmd_linear_x, self.max_target_velocity),
                              -self.max_target_velocity)
        # angular.z (rad/s) → target_yaw_rate (deg/s)
        target_yaw_rate = math.degrees(self.cmd_angular_z)
        self.target_velocity = target_velocity
        self.target_yaw_rate = target_yaw_rate

        # PI de velocidad
        vel_error = target_velocity - self.robot_velocity
        if abs(vel_error) < 2.0:
            self.acc_err_vel += vel_error * dt
        self.acc_err_vel = max(min(self.acc_err_vel, self.max_i_sum_vel),
                               -self.max_i_sum_vel)
        pitch_offset = self.kp_vel * vel_error + self.ki_vel * self.acc_err_vel
        pitch_offset = max(min(pitch_offset, self.max_pitch_offset_deg),
                           -self.max_pitch_offset_deg)
        self.pitch_offset = pitch_offset

        dynamic_target = self.target_pitch_deg + pitch_offset
        self.dynamic_target = dynamic_target

        # PID equilibrio
        tilt_error = dynamic_target - pitch
        if   tilt_error >  180: tilt_error -= 360
        elif tilt_error < -180: tilt_error += 360

        if abs(tilt_error) > self.max_angle_deg:
            self._motor_out(0.0, 0.0)
            self.acc_err_tilt = 0.0
            self.commanded_torque = 0.0
            return

        if abs(tilt_error) < 10.0:
            self.acc_err_tilt += tilt_error * dt
        self.acc_err_tilt = max(min(self.acc_err_tilt, self.max_i_sum_tilt),
                                -self.max_i_sum_tilt)

        p_tilt = tilt_error * self.kp_tilt
        i_tilt = self.acc_err_tilt * self.ki_tilt
        d_tilt = -gyro_pitch * self.kd_tilt
        balance_torque = p_tilt + i_tilt + d_tilt
        self.commanded_torque = balance_torque

        # Horizontal PI 
        yaw_error = target_yaw_rate - gyro_yaw
        if abs(yaw_error) < 90.0:
            self.acc_err_yaw += yaw_error * dt
        self.acc_err_yaw = max(min(self.acc_err_yaw, self.max_i_sum_yaw),
                               -self.max_i_sum_yaw)
        steering_torque = self.kp_yaw * yaw_error + self.ki_yaw * self.acc_err_yaw
        steering_torque = max(min(steering_torque, self.max_steering_torque),
                              -self.max_steering_torque)


        motor_l = balance_torque + steering_torque
        motor_r = balance_torque - steering_torque
        motor_l = max(min(motor_l, self.max_torque), -self.max_torque)
        motor_r = max(min(motor_r, self.max_torque), -self.max_torque)

        self._motor_out(motor_l, motor_r)

    def _motor_out(self, motor_l: float, motor_r: float):
        try:
            self.odrv0.axis1.controller.input_torque = motor_l * DIR_ODRV0
            self.odrv1.axis1.controller.input_torque = motor_r * DIR_ODRV1
        except Exception as e:
            self.get_logger().error(f"ODrive write error: {e}")
            self._set_motors_idle()
            self.engaged = False

    def _publish_step(self):
        now = self.get_clock().now().to_msg()
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        pitch_rad = math.radians(self.pitch_deg)
        gyro_pitch_rad = math.radians(self.gyro_pitch)
        gyro_yaw_rad = math.radians(self.gyro_yaw)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation = quaternion_from_euler(0.0, pitch_rad, 0.0)
        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 1e6,
        ]
        imu.angular_velocity.x = gyro_pitch_rad
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = gyro_yaw_rad
        imu.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self.pub_imu.publish(imu)

        # odometria
        dt = now_sec - self.last_odom_time
        self.last_odom_time = now_sec
        self.odom_yaw += gyro_yaw_rad * dt

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
     
        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.odom_yaw)
        odom.pose.covariance = [1.0 if i in (0, 7, 14, 21, 28, 35) else 0.0
                                for i in range(36)]
        odom.twist.twist.linear.x  = self.robot_velocity
        odom.twist.twist.angular.z = gyro_yaw_rad
        odom.twist.covariance = [0.1 if i in (0, 7, 14, 21, 28, 35) else 0.0
                                 for i in range(36)]
        self.pub_odom.publish(odom)

        #joint_states 
        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        js.velocity = [0.0, 0.0]
        js.effort   = [0.0, 0.0]
        self.pub_joint_states.publish(js)

        # /balance/state 
        bal = Float32MultiArray()
        bal.data = [
            float(self.pitch_deg),
            float(self.dynamic_target),
            float(self.dynamic_target - self.pitch_deg),
            float(self.commanded_torque),
            float(self.robot_velocity),
            float(self.pitch_offset),
            float(self.loop_hz),
            float(1.0 if self.engaged else 0.0),
            float(self.target_velocity),
            float(self.target_yaw_rate),
            float(self.gyro_yaw),
        ]
        self.pub_balance.publish(bal)

    def destroy_node(self):
        self._set_motors_idle()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CascadeBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
