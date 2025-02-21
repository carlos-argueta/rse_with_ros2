import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

class DashcamNode(Node):
    def __init__(self):
        super().__init__('dashcam_node')
        self.subscription = self.create_subscription(
            String,
            'command',  # Same topic name as publisher
            self.listener_callback,
            10)
        
        self.video_path = "/home/carlos/Documents/camera/dash1.mp4"
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video at {self.video_path}")
            exit(1)
        
        self.transformation = None
        # Create a timer to call the video update method every 30ms
        self.timer = self.create_timer(0.03, self.timer_callback)

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        if command == 'q':
            self.transformation = "quit"
        elif command == 'r':
            self.transformation = "rgb"
        elif command == 'b':
            self.transformation = "bgr"
        elif command == 'g':
            self.transformation = "gray"
        elif command == 'f':
            self.transformation = "flip"

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video reached.")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        if self.transformation == "rgb":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif self.transformation == "gray":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        elif self.transformation == "flip":
            frame = cv2.flip(frame, 1)
        elif self.transformation == "quit":
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
        

        cv2.imshow('Video Player', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DashcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
