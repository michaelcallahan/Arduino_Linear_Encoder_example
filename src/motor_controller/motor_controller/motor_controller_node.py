import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Float32,
            'encoder',
            self.encoder_callback,
            10)
        self.publisher = self.create_publisher(Float32, 'pos_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.setpoint = 7.5

        self.get_logger().info("MotorController node has been started.")
        self.get_logger().info("Enter new setpoint values (in mm) to change the position:")

    def encoder_callback(self, msg):
        self.get_logger().info('Encoder value: %f' % msg.data)

    def timer_callback(self):
        # Publish the desired setpoint position
        msg = Float32()
        msg.data = self.setpoint
        self.publisher.publish(msg)

    def user_input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter new setpoint (mm): ")
                new_setpoint = float(user_input)
                self.set_setpoint(new_setpoint)
            except ValueError:
                self.get_logger().info("Invalid input. Please enter a valid number.")

    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
        self.get_logger().info(f"Setpoint updated to: {self.setpoint} mm")

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    import threading
    input_thread = threading.Thread(target=motor_controller.user_input_loop)
    input_thread.start()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Shutting down MotorController node.")

    motor_controller.destroy_node()
    rclpy.shutdown()
    input_thread.join()

if __name__ == '__main__':
    main()

