#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class MotorController(Node):
    """ROS2 node for controlling robot motors via GPIO"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # L298N Motor Driver GPIO pins
        # Standard L298N wiring:
        # - ENA (Enable A) -> PWM for left motor speed
        # - IN1, IN2 -> Direction control for left motor
        # - ENB (Enable B) -> PWM for right motor speed
        # - IN3, IN4 -> Direction control for right motor
        self.left_motor_pins = {
            'ena': 12,   # ENA (PWM) -> GPIO 12, Pin 32
            'in1': 6,    # IN1 -> GPIO 6, Pin 31
            'in2': 5     # IN2 -> GPIO 5, Pin 29
        }
        self.right_motor_pins = {
            'enb': 13,   # ENB (PWM) -> GPIO 13, Pin 33
            'in3': 19,   # IN3 -> GPIO 19, Pin 35
            'in4': 26    # IN4 -> GPIO 26, Pin 37
        }

        
        # Robot physical parameters
        self.wheel_separation = 0.3  # Distance between wheels in meters (adjust for your robot)
        self.max_speed = 0.5         # Maximum speed in m/s (start conservative)
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timer - stop robot if no commands received
        self.last_command_time = time.time()
        self.safety_timeout = 2.0  # seconds
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Motor Controller Node Started - L298N Driver')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Max speed: {self.max_speed}m/s')
        self.get_logger().info(f'Left motor: ENA={self.left_motor_pins["ena"]}, IN1={self.left_motor_pins["in1"]}, IN2={self.left_motor_pins["in2"]}')
        self.get_logger().info(f'Right motor: ENB={self.right_motor_pins["enb"]}, IN3={self.right_motor_pins["in3"]}, IN4={self.right_motor_pins["in4"]}')
    
    def setup_gpio(self):
        """Initialize GPIO pins for L298N motor control"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup all motor control pins
        all_pins = [
            self.left_motor_pins['ena'], self.left_motor_pins['in1'], self.left_motor_pins['in2'],
            self.right_motor_pins['enb'], self.right_motor_pins['in3'], self.right_motor_pins['in4']
        ]
        
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Initialize all pins to LOW
        
        # Initialize PWM on enable pins (ENA and ENB)
        self.left_pwm = GPIO.PWM(self.left_motor_pins['ena'], 1000)  # 1kHz frequency
        self.right_pwm = GPIO.PWM(self.right_motor_pins['enb'], 1000)
        
        self.left_pwm.start(0)   # Start with 0% duty cycle (stopped)
        self.right_pwm.start(0)
        
        self.get_logger().info('L298N GPIO initialized successfully')
    
    def cmd_vel_callback(self, msg):
        """Process incoming velocity commands for L298N driver"""
        
        # Update command timestamp
        self.last_command_time = time.time()
        
        # Extract velocities
        linear_x = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        angular_z = max(-2.0, min(2.0, msg.angular.z))  # Limit rotation rate
        
        # Differential drive kinematics
        # Left wheel velocity = linear - angular * wheel_separation / 2
        # Right wheel velocity = linear + angular * wheel_separation / 2
        left_velocity = linear_x - (angular_z * self.wheel_separation / 2)
        right_velocity = linear_x + (angular_z * self.wheel_separation / 2)
        
        # Convert to PWM values (0-100)
        left_pwm_value = min(abs(left_velocity) / self.max_speed * 100, 100)
        right_pwm_value = min(abs(right_velocity) / self.max_speed * 100, 100)
        
        # L298N direction control:
        # Forward: IN1=HIGH, IN2=LOW (left) or IN3=HIGH, IN4=LOW (right)
        # Backward: IN1=LOW, IN2=HIGH (left) or IN3=LOW, IN4=HIGH (right)
        # Stop: IN1=LOW, IN2=LOW (left) or IN3=LOW, IN4=LOW (right)
        
        # Left motor direction
        if left_velocity > 0.01:  # Forward
            GPIO.output(self.left_motor_pins['in1'], GPIO.HIGH)
            GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        elif left_velocity < -0.01:  # Backward
            GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
            GPIO.output(self.left_motor_pins['in2'], GPIO.HIGH)
        else:  # Stop
            GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
            GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        
        # Right motor direction
        if right_velocity > 0.01:  # Forward
            GPIO.output(self.right_motor_pins['in3'], GPIO.HIGH)
            GPIO.output(self.right_motor_pins['in4'], GPIO.LOW)
        elif right_velocity < -0.01:  # Backward
            GPIO.output(self.right_motor_pins['in3'], GPIO.LOW)
            GPIO.output(self.right_motor_pins['in4'], GPIO.HIGH)
        else:  # Stop
            GPIO.output(self.right_motor_pins['in3'], GPIO.LOW)
            GPIO.output(self.right_motor_pins['in4'], GPIO.LOW)
        
        # Apply PWM values to enable pins
        self.left_pwm.ChangeDutyCycle(left_pwm_value)
        self.right_pwm.ChangeDutyCycle(right_pwm_value)
        
        # Log the command
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(
                f'Motor speeds: L={left_velocity:.2f}m/s ({left_pwm_value:.1f}%), '
                f'R={right_velocity:.2f}m/s ({right_pwm_value:.1f}%)'
            )
    
    def safety_check(self):
        """Safety function to stop robot if no recent commands"""
        if time.time() - self.last_command_time > self.safety_timeout:
            # Stop motors by setting PWM to 0 and direction pins to LOW
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(0)
            GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
            GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
            GPIO.output(self.right_motor_pins['in3'], GPIO.LOW)
            GPIO.output(self.right_motor_pins['in4'], GPIO.LOW)
    
    def destroy_node(self):
        """Clean up GPIO on shutdown"""
        self.get_logger().info('Shutting down motor controller...')
        
        # Stop all motors
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)
        
        # Set all direction pins to LOW
        GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in3'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in4'], GPIO.LOW)
        
        # Clean up PWM
        self.left_pwm.stop()
        self.right_pwm.stop()
        
        # Clean up GPIO
        GPIO.cleanup()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


