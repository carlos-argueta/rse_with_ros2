import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter # Import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult # Optional: for better description/validation

from std_msgs.msg import String
from rse_fsd_msgs.msg import ClassCount, ClassCountArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D

import cv2

# Import the YOLO model from Ultralytics
from ultralytics import YOLO

class DashcamNode(Node):
    def __init__(self):
        super().__init__('dashcam_node')

        # === Parameter Declarations ===
        # 1. Resolution Parameter
        resolution_descriptor = ParameterDescriptor(
            name='display_resolution',
            type=ParameterType.PARAMETER_INTEGER,
            description='Set the display resolution height (1080, 720, or 480).',
        )
        self.declare_parameter('display_resolution', 720, resolution_descriptor) # Default to 720p

        # 2. Show Detections Toggle Parameter
        show_detections_descriptor = ParameterDescriptor(
            name='show_detections',
            type=ParameterType.PARAMETER_BOOL,
            description='Toggle display of bounding boxes on/off.'
        )
        self.declare_parameter('show_detections', True, show_detections_descriptor) # Default to True (show boxes)

        # 3. Bounding Box Color Parameter
        bbox_color_descriptor = ParameterDescriptor(
            name='bbox_color',
            type=ParameterType.PARAMETER_STRING,
            description="Set bounding box color ('green' or 'blue')."
        )
        self.declare_parameter('bbox_color', 'green', bbox_color_descriptor) # Default to green

        self.subscription = self.create_subscription(
            String,
            'command',  # Same topic name as publisher
            self.listener_callback,
            10)
        
        self.video_path = "/home/carlos/Documents/rse_with_ros2/lesson1/dash1.mp4"
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video at {self.video_path}")
            exit(1)
        
        self.transformation = None
        # Create a timer to call the video update method every 30ms
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Load the YOLO model – update the model weights path if needed
        self.model = YOLO('yolo11n.pt')

        # Create a publisher for detection messages
        self.detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)

        self.get_logger().info("Dashcam node started.")
        self.get_logger().info("Use 'ros2 param set /dashcam_node <param_name> <value>' to change parameters.")
        self.get_logger().info(f" Example: ros2 param set {self.get_name()} display_resolution 1080")
        self.get_logger().info(f" Example: ros2 param set {self.get_name()} show_detections false")
        self.get_logger().info(f" Example: ros2 param set {self.get_name()} bbox_color '\"blue\"'") 


    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")
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
        # --- Read Parameters ---
        resolution_param = self.get_parameter('display_resolution').get_parameter_value().integer_value
        show_dets_param = self.get_parameter('show_detections').get_parameter_value().bool_value
        color_param = self.get_parameter('bbox_color').get_parameter_value().string_value

        # Define resolution mappings (width, height)
        resolutions = {
            1080: (1920, 1080),
            720: (1280, 720),
            480: (640, 480) # Added a lower resolution option
        }
        target_size = resolutions.get(resolution_param) # Returns None if key not found

        # Define color mappings (BGR format for OpenCV)
        color_map = {
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        # Get the color tuple, default to green if invalid string parameter
        bbox_color = color_map.get(color_param.lower(), (0, 255, 0))
        # --- End Read Parameters ---

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video or cannot read frame.")
            self.cap.release()
            cv2.destroyAllWindows()
            # Ensure ROS shutdown is clean
            if rclpy.ok():
                 self.destroy_node() # Clean up the node itself
                 rclpy.shutdown()
            return

        # Keep a copy of the original frame for detection (YOLO expects BGR)
        orig_frame = frame.copy()
        display_frame = frame # Start with the current frame for display modifications

        # Apply any command-based transformations for display
        # Note: Grayscale might interfere with detection if applied before YOLO
        # Note: RGB will make colors look wrong in cv2.imshow unless handled
        if self.transformation == "rgb":
            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        elif self.transformation == "gray":
             # Convert to grayscale, then back to BGR if needed for drawing color boxes
             gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
             display_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) # Keep 3 channels
        elif self.transformation == "flip":
            display_frame = cv2.flip(display_frame, 1)
        elif self.transformation == "bgr":
             self.transformation = None # Reset BGR is the default, no change needed
             pass # Already BGR
        elif self.transformation == "quit":
            self.get_logger().info("Quit command received.")
            self.cap.release()
            cv2.destroyAllWindows()
            if rclpy.ok():
                self.destroy_node()
                rclpy.shutdown()
            return

        # Run object detection on the original frame using YOLO
        results = self.model(orig_frame)
        detections_array = Detection2DArray()
        detections_array.header.stamp = self.get_clock().now().to_msg()
        detections_array.header.frame_id = "camera_frame"

        if results and len(results) > 0:
            result = results[0]
            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()  # shape: (N, 4)
                confs = result.boxes.conf.cpu().numpy()    # shape: (N,)
                classes = result.boxes.cls.cpu().numpy()   # shape: (N,)
                
                for i in range(len(boxes)):
                    xmin, ymin, xmax, ymax = boxes[i]
                    center_x = (xmin + xmax) / 2.0
                    center_y = (ymin + ymax) / 2.0
                    width = xmax - xmin
                    height = ymax - ymin

                    detection = Detection2D()
                    detection.header.stamp = detections_array.header.stamp
                    detection.header.frame_id = detections_array.header.frame_id
                    
                    # Fill in the bounding box
                    bbox = BoundingBox2D()
                    center = Pose2D()
                    center.position.x = center_x
                    center.position.y = center_y
                    center.theta = 0.0
                    bbox.center = center

                    # Set the dimensions using size_x and size_y fields
                    bbox.size_x = float(width)
                    bbox.size_y = float(height)
                    detection.bbox = bbox

                    # Use ObjectHypothesisWithPose for detection result
                    obj_hyp = ObjectHypothesisWithPose()
                    if hasattr(self.model, 'names') and self.model.names is not None:
                        obj_hyp.hypothesis.class_id = self.model.names[int(classes[i])] 
                    else:
                        obj_hyp.hypothesis.class_id = str(int(classes[i]))
                         
                    obj_hyp.hypothesis.score = float(confs[i])
                    detection.results.append(obj_hyp)

                    detections_array.detections.append(detection)

                    # === Draw detection based on parameter ===
                    if show_dets_param:
                        # Use the selected color
                        cv2.rectangle(display_frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), bbox_color, 2)
                        label_text = f"{obj_hyp.hypothesis.class_id}: {obj_hyp.hypothesis.score:.2f}"
                        # Put text slightly above the box
                        cv2.putText(display_frame, label_text, (int(xmin), int(ymin) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bbox_color, 2)

        # Publish the detections message
        self.detections_pub.publish(detections_array)

        # --- Resize the display frame based on parameter ---
        if target_size:
            # Use INTER_AREA for shrinking, it's generally good
            display_frame = cv2.resize(display_frame, target_size, interpolation=cv2.INTER_AREA)
        
        # Display the video frame (with drawn detections, if any)
        cv2.imshow('Video Player', display_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DashcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
