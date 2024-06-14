import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import serial
import threading
import sys

class ArduinoSimulatorSerial(Node):
    def __init__(self, port):
        super().__init__('arduino_simulator_serial')
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.publisher = self.create_publisher(Float32, 'encoder', 10)
        self.subscription = self.create_subscription(
            Float32,
            'pos_cmd',
            self.pos_cmd_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.setpoint = 7.5
        self.current_position = 0.0
        self.motor_speed = 0.0

        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        self.ITerm = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

        self.running = True

        self.get_logger().info("Arduino Simulator started")
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.write_thread = threading.Thread(target=self.write_to_serial)

        self.read_thread.start()
        self.write_thread.start()

    def pos_cmd_callback(self, msg):
        self.setpoint = msg.data
        self.get_logger().info(f"Setpoint updated to: {self.setpoint} mm")

    def read_from_serial(self):
        while self.running:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                try:
                    self.setpoint = float(line)
                    self.get_logger().info(f"Setpoint updated to: {self.setpoint} mm")
                except ValueError:
                    self.get_logger().warn(f"Invalid input received: {line}")

    def write_to_serial(self):
        while self.running:
            current_time = time.time()
            elapsed_time = current_time - self.last_time
            self.last_time = current_time

            error = self.setpoint - self.current_position
            self.ITerm += error * elapsed_time
            dError = (error - self.last_error) / elapsed_time if elapsed_time > 0 else 0.0

            output = self.kp * error + self.ki * self.ITerm + self.kd * dError
            self.last_error = error

            self.motor_speed = output
            self.current_position += self.motor_speed * elapsed_time

            self.ser.write(f"{self.current_position:.2f}\n".encode('utf-8'))
            self.get_logger().info(f"Current position: {self.current_position:.2f} mm, Motor speed: {self.motor_speed:.2f} mm/s")
            time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        self.read_thread.join()
        self.write_thread.join()
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run arduino_simulator_serial arduino_simulator_node <serial_port>")
        return

    port = sys.argv[1]
    arduino_simulator = ArduinoSimulatorSerial(port)

    try:
        rclpy.spin(arduino_simulator)
    except KeyboardInterrupt:
        arduino_simulator.get_logger().info("Shutting down Arduino Simulator.")
    finally:
        arduino_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
