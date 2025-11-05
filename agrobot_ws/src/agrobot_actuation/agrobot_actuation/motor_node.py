#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Try GPIO so you can still run on a dev machine
try:
    import RPi.GPIO as GPIO
    HW = True
except Exception:
    HW = False

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # ---- pins (BCM) ----
        self.STEP = 18; self.DIR = 23; self.EN = 24  # stepper
        self.SERVO = 12                               # servo (PWM-capable)

        # ---- stepper config ----
        self.STEPS_PER_REV = 200
        self.MICROSTEP = 1
        self.STEP_DELAY = 0.002  # s

        # ---- servo config ----
        self.PWM_HZ = 50
        self.MIN_US, self.MAX_US = 1000, 2000
        self.STEP_US, self.DWELL = 20, 0.02

        self.servo_pwm = None
        if HW:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.STEP, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.DIR,  GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.EN,   GPIO.OUT, initial=GPIO.HIGH)  # HIGH=disabled (typical)
            GPIO.setup(self.SERVO, GPIO.OUT)
            self.servo_pwm = GPIO.PWM(self.SERVO, self.PWM_HZ)
            self.servo_pwm.start(0)
        else:
            self.get_logger().warn("GPIO not available; simulating only.")

        # Service: empty request → do motion
        self.srv = self.create_service(Trigger, 'do_motion', self.handle_motion)
        self.get_logger().info("Service '/do_motion' (std_srvs/Trigger) ready.")

    # ---- helpers ----
    def _en(self, on: bool):
        if HW: GPIO.output(self.EN, GPIO.LOW if on else GPIO.HIGH)
        else:  self.get_logger().info(f"[SIM] stepper enable={on}")

    def _dir(self, fwd: bool):
        if HW: GPIO.output(self.DIR, GPIO.HIGH if fwd else GPIO.LOW)
        else:  self.get_logger().info(f"[SIM] dir={'FWD' if fwd else 'REV'}")

    def _steps(self, n, d):
        if HW:
            for _ in range(n):
                GPIO.output(self.STEP, GPIO.HIGH); time.sleep(d/2)
                GPIO.output(self.STEP, GPIO.LOW);  time.sleep(d/2)
        else:
            self.get_logger().info(f"[SIM] stepping {n} at ~{1/d:.0f} sps")
            time.sleep(n * d)

    def _servo_us(self, us):
        us = max(self.MIN_US, min(self.MAX_US, us))
        if HW:
            duty = (us / 20000.0) * 100.0  # 20ms period
            self.servo_pwm.ChangeDutyCycle(duty)
        else:
            self.get_logger().info(f"[SIM] servo {us}us")

    def _sweep_servo(self):
        for us in range(self.MIN_US, self.MAX_US+1, self.STEP_US):
            self._servo_us(us); time.sleep(self.DWELL)
        time.sleep(0.3)
        for us in range(self.MAX_US, self.MIN_US-1, -self.STEP_US):
            self._servo_us(us); time.sleep(self.DWELL)
        self._servo_us((self.MIN_US + self.MAX_US)//2)

    # ---- service callback ----
    def handle_motion(self, _req, resp):
        try:
            self.get_logger().info("Running motion sequence…")
            self._en(True)
            self._dir(True);  self._steps(self.STEPS_PER_REV*self.MICROSTEP, self.STEP_DELAY)
            time.sleep(0.5)
            self._dir(False); self._steps(self.STEPS_PER_REV*self.MICROSTEP, self.STEP_DELAY)
            self._en(False)
            self._sweep_servo()
            resp.success = True
            resp.message = "Motion complete"
        except Exception as e:
            resp.success = False
            resp.message = f"Error: {e}"
        return resp

    def destroy_node(self):
        if HW:
            if self.servo_pwm: self.servo_pwm.stop()
            GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()