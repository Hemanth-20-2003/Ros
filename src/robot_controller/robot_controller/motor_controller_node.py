#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
import sys
import time

# ================================
# L298N ⇄ Raspberry Pi pin mapping
# (BCM numbering; matches your working keyboard node)
# ================================
IN1 = 6
IN2 = 5
IN3 = 19
IN4 = 26
ENA = 12   # PWM0
ENB = 13   # PWM1

PWM_FREQ_HZ = 1000  # 1 kHz, same as your working code

# ================================
# Open GPIO chip (chip 4 typically user-accessible)
# ================================
try:
    CHIP = lgpio.gpiochip_open(4)
    print("Successfully opened GPIO chip 4")
except lgpio.error as e:
    print(f"Failed to open GPIO chip 4: {e}")
    print("Error: Could not open GPIO chip. Check permissions.")
    print("Try running with sudo or add user to gpio group:")
    print("  sudo usermod -a -G gpio $USER")
    sys.exit(1)


def _claim_outputs_or_die(handle, pins):
    try:
        for p in pins:
            lgpio.gpio_claim_output(handle, p)
    except lgpio.error as e:
        print(f"Error setting up GPIO pins: {e}")
        try:
            lgpio.gpiochip_close(handle)
        except Exception:
            pass
        sys.exit(1)


def _pwm_setup_or_die(handle):
    try:
        lgpio.tx_pwm(handle, ENA, PWM_FREQ_HZ, 0)  # start with 0% duty
        lgpio.tx_pwm(handle, ENB, PWM_FREQ_HZ, 0)
        print("PWM configured successfully")
    except lgpio.error as e:
        print(f"Error setting up PWM: {e}")
        try:
            lgpio.gpiochip_close(handle)
        except Exception:
            pass
        sys.exit(1)


class MotorController(Node):
    """ROS2 node for controlling robot motors via L298N using lgpio"""

    def __init__(self):
        super().__init__('motor_controller_lgpio')

        # Robot physical parameters
        self.wheel_separation = 0.3  # meters
        self.max_speed = 0.5         # m/s

        # Claim outputs and init PWM (already global handle opened)
        _claim_outputs_or_die(CHIP, [IN1, IN2, IN3, IN4, ENA, ENB])
        _pwm_setup_or_die(CHIP)

        # Initialize all direction pins LOW, PWM 0%
        self._all_stop()

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Safety timer
        self.last_command_time = time.time()
        self.safety_timeout = 2.0  # seconds
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Motor Controller (lgpio) Started - L298N Driver')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation} m')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(
            f'Pins (BCM): ENA={ENA}, IN1={IN1}, IN2={IN2}; ENB={ENB}, IN3={IN3}, IN4={IN4}'
        )

    # --------------- Low-level helpers ---------------

    def _set_left_dir(self, forward: bool | None):
        """forward=True sets IN1=1, IN2=0; forward=False sets IN1=0, IN2=1; None sets both LOW (brake/coast)."""
        if forward is True:
            lgpio.gpio_write(CHIP, IN1, 1)
            lgpio.gpio_write(CHIP, IN2, 0)
        elif forward is False:
            lgpio.gpio_write(CHIP, IN1, 0)
            lgpio.gpio_write(CHIP, IN2, 1)
        else:
            lgpio.gpio_write(CHIP, IN1, 0)
            lgpio.gpio_write(CHIP, IN2, 0)

    def _set_right_dir(self, forward: bool | None):
        """forward=True sets IN3=1, IN4=0; forward=False sets IN3=0, IN4=1; None sets both LOW."""
        if forward is True:
            lgpio.gpio_write(CHIP, IN3, 1)
            lgpio.gpio_write(CHIP, IN4, 0)
        elif forward is False:
            lgpio.gpio_write(CHIP, IN3, 0)
            lgpio.gpio_write(CHIP, IN4, 1)
        else:
            lgpio.gpio_write(CHIP, IN3, 0)
            lgpio.gpio_write(CHIP, IN4, 0)

    def _set_left_pwm(self, duty_percent: float):
        """0–100"""
        duty = max(0.0, min(100.0, duty_percent))
        lgpio.tx_pwm(CHIP, ENA, PWM_FREQ_HZ, duty)

    def _set_right_pwm(self, duty_percent: float):
        """0–100"""
        duty = max(0.0, min(100.0, duty_percent))
        lgpio.tx_pwm(CHIP, ENB, PWM_FREQ_HZ, duty)

    def _all_stop(self):
        """Stop both motors and set direction pins LOW"""
        self._set_left_pwm(0)
        self._set_right_pwm(0)
        self._set_left_dir(None)
        self._set_right_dir(None)

    # --------------- ROS callback & safety ---------------

    def cmd_vel_callback(self, msg: Twist):
        # Update command timestamp
        self.last_command_time = time.time()

        # Extract velocities with limits
        linear_x = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
        angular_z = max(-2.0, min(2.0, float(msg.angular.z)))

        # Differential drive kinematics
        half_w = self.wheel_separation / 2.0
        left_velocity = linear_x - (angular_z * half_w)
        right_velocity = linear_x + (angular_z * half_w)

        # Convert to PWM duty (0–100)
        left_pwm = min(abs(left_velocity) / self.max_speed * 100.0, 100.0)
        right_pwm = min(abs(right_velocity) / self.max_speed * 100.0, 100.0)

        # Direction control
        if left_velocity > 0.01:
            self._set_left_dir(True)
        elif left_velocity < -0.01:
            self._set_left_dir(False)
        else:
            self._set_left_dir(None)
            left_pwm = 0.0

        if right_velocity > 0.01:
            self._set_right_dir(True)
        elif right_velocity < -0.01:
            self._set_right_dir(False)
        else:
            self._set_right_dir(None)
            right_pwm = 0.0

        # Apply PWM
        self._set_left_pwm(left_pwm)
        self._set_right_pwm(right_pwm)

        # Log if anything is moving
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(
                f'Motor speeds: L={left_velocity:.2f} m/s ({left_pwm:.1f}%), '
                f'R={right_velocity:.2f} m/s ({right_pwm:.1f}%)'
            )

    def safety_check(self):
        """Stop robot if no recent commands"""
        if time.time() - self.last_command_time > self.safety_timeout:
            self._all_stop()

    # --------------- Cleanup ---------------

    def destroy_node(self):
        self.get_logger().info('Shutting down motor controller (lgpio)...')
        try:
            self._all_stop()
        except Exception:
            pass
        # Free pins and close chip
        try:
            for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
                lgpio.gpio_free(CHIP, p)
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(CHIP)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
