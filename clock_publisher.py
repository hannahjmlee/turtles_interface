import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Int32, 'waypoint_time', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_time = 0
        self.turtlebots_status = {}
        self.create_subscription(Bool, 'turtlebot_status', self.status_callback, 10)

    def status_callback(self, msg, bot_id):
        self.turtlebots_status[bot_id] = msg.data

    def timer_callback(self):
        if all(self.turtlebots_status.values()):
            self.current_time += 1
            msg = Int32()
            msg.data = self.current_time
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published waypoint time: {self.current_time}')

def main(args=None):
    rclpy.init(args=args)
    clock_publisher = ClockPublisher()
    rclpy.spin(clock_publisher)
    clock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
