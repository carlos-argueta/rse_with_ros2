import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ControlCenterNode(Node):

    def __init__(self):
        super().__init__('control_center_node')
        self.publisher_ = self.create_publisher(String, 'command', 10)
        self.get_logger().info('Command publisher started')
       
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    control_center_node = ControlCenterNode()

    while rclpy.ok():
        command = input("Local change Enter command (r=rgb / b=bgr / g=gray / f=flip / q=quit): ") # Get commands from the user
        if command in ['r', 'b', 'g', 'f', 'q']:
            control_center_node.publish_command(command)
        else:
            print("Invalid command. Use r, g, f, b or q.")

        if command == 'q':
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_center_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()