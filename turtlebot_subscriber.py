import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import hostname
import re

class TurtlebotSubscriber(Node):
    def __init__(self):
        super().__init__('turtlebot_subscriber')
        self.subscription = self.create_subscription(Int32, 'waypoint_time', self.listener_callback, 10)
        self.current_waypoint = None
        self.waypoints = self.load_waypoints()
        self.distance_threshold = 0.25  # Set your desired threshold
        self.create_timer(1.0, self.check_position)

    def load_waypoints(self):
        hostname = socket.gethostname()  # Get the hostname
        match = re.search(r'turtlebot(\d+)', hostname)
        num = match.group(1)
        waypoints_filename = f'waypoints{num}.txt'

        waypoints = []
        try:
            with open(waypoints_filename, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    coords = tuple(map(float, line.strip().split(',')))
                    waypoints.append(coords)

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {waypoints_filename}')

        return waypoints

    def scale_waypoint(self, waypoint):
        total_length = 11
        num_vertices = 11

        start_x, start_y = (0, 0)
        x, y = waypoint

        leg_length = total_length / (num_vertices - 1)
        new_x = start_x + x * leg_length
        new_y = start_y + y * leg_length

        return (new_x, new_y)

    def listener_callback(self, msg):
        self.current_waypoint = self.waypoints.get(msg.data)
        if self.current_waypoint:
            self.move_to_waypoint(self.current_waypoint)

    def move_to_waypoint(self, waypoint):
        # Call the movement code with the waypoint
        self.get_logger().info(f'Moving to waypoint: {waypoint}')
        # Example call: self.move_to(waypoint)

    def check_position(self):
        if self.current_waypoint:
            # Replace with actual position checking logic
            current_position = self.get_current_position()
            if self.distance(current_position, self.current_waypoint) < self.distance_threshold:
                self.stop_moving()
                self.get_logger().info(f'Reached waypoint: {self.current_waypoint}')
                self.current_waypoint = None

    def get_current_position(self):
        # Replace with actual logic to get current position
        return (0, 0)

    def distance(self, pos1, pos2):
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def stop_moving(self):
        # Call the stop function of the movement code
        self.get_logger().info('Stopped moving')

def main(args=None):
    rclpy.init(args=args)
    turtlebot_subscriber = TurtlebotSubscriber()
    rclpy.spin(turtlebot_subscriber)
    turtlebot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

