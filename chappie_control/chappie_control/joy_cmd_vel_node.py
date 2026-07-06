#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import pygame



AXIS_THROTTLE     = 1        # stick izquierdo Y
AXIS_STEERING     = 2        # stick derecho X
ENABLE_BUTTON     = 7        # R2 
ENGAGE_BUTTON     = 3        # Triángulo
DISENGAGE_BUTTON  = 0        # Cruz
DEADZONE          = 0.10
PUBLISH_RATE_HZ   = 30.0
MAX_LINEAR_MPS    = 0.5
MAX_ANGULAR_RPS   = 1.0


ENGAGE_SRV    = '/chappie/engage'
DISENGAGE_SRV = '/chappie/disengage'


class JoyCmdVelNode(Node):

    def __init__(self):
        super().__init__('joy_cmd_vel')

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No se ha detectado ningun mando.")
            raise SystemExit(1)

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(
            f"Controller: {self.joy.get_name()}  "
            f"axes={self.joy.get_numaxes()} buttons={self.joy.get_numbuttons()}"
        )
        self.get_logger().info(
            f"Manten R2(button {ENABLE_BUTTON}) para qeu funcione. "
            f"Triangulo (btn {ENGAGE_BUTTON}) = Activado. "
            f"Cruz (btn {DISENGAGE_BUTTON}) = Desactivado ."
        )

        # Publishers 
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_in', 10)
        self.engage_cli    = self.create_client(Trigger, ENGAGE_SRV)
        self.disengage_cli = self.create_client(Trigger, DISENGAGE_SRV)

        
        if not self.engage_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"{ENGAGE_SRV} not available at startup — will retry on press."
            )
        if not self.disengage_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"{DISENGAGE_SRV} not available at startup — will retry on press."
            )

     
        self.was_enabled       = False   
        self.prev_engage_btn   = False   
        self.prev_disengage_btn = False  

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

   
    def _tick(self):
        pygame.event.pump()

        # ── Read buttons defensively ──
        def safe_btn(idx: int) -> bool:
            try:
                return bool(self.joy.get_button(idx))
            except (pygame.error, IndexError):
                return False

        engage_btn    = safe_btn(ENGAGE_BUTTON)
        disengage_btn = safe_btn(DISENGAGE_BUTTON)
        enabled       = safe_btn(ENABLE_BUTTON)

        
        if engage_btn and not self.prev_engage_btn:
            self._call_trigger(self.engage_cli, ENGAGE_SRV, "ENGAGE (△)")
        if disengage_btn and not self.prev_disengage_btn:
            self._call_trigger(self.disengage_cli, DISENGAGE_SRV, "DISENGAGE (✕)")
        self.prev_engage_btn    = engage_btn
        self.prev_disengage_btn = disengage_btn

        # Twist publishing 
        twist = Twist()

        if enabled:
            try:
                raw_throttle = self.joy.get_axis(AXIS_THROTTLE)
                raw_steering = self.joy.get_axis(AXIS_STEERING)
            except (pygame.error, IndexError):
                raw_throttle = 0.0
                raw_steering = 0.0

       
            t = 0.0 if abs(raw_throttle) < DEADZONE else -raw_throttle
            s = 0.0 if abs(raw_steering) < DEADZONE else  raw_steering

            twist.linear.x  = t * MAX_LINEAR_MPS
            twist.angular.z = s * MAX_ANGULAR_RPS

            self.pub_cmd_vel.publish(twist)
            self.was_enabled = True
        else:
            if self.was_enabled:
                self.pub_cmd_vel.publish(twist)
                self.was_enabled = False

 
    def _call_trigger(self, client, srv_name: str, label: str)
        if not client.service_is_ready():
            self.get_logger().warn(
                f"{label}: {srv_name} not available, ignoring press."
            )
            return
        self.get_logger().info(f"{label} → {srv_name}")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self._on_response(f, label)
        )

    def _on_response(self, future, label: str):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"{label} ok: {resp.message}")
            else:
                self.get_logger().warn(f"{label} failed: {resp.message}")
        except Exception as e:
            self.get_logger().warn(f"{label} exception: {e}")

    def destroy_node(self):
        try:
            pygame.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoyCmdVelNode()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
