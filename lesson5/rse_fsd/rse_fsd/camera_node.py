import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rse_fsd_msgs.msg import ClassCount, ClassCountArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D

from rse_fsd_msgs.srv import GetClassCount

import cv2

# Import the YOLO model from Ultralytics
from ultralytics import YOLO




class DashcamNode(Node):
    def __init__(self):
        super().__init__('dashcam_node')
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

        # Create a publisher for class count messages
        self.class_count_pub = self.create_publisher(ClassCountArray, 'class_counts', 10)

        # Create the service
        self.srv = self.create_service(GetClassCount, 'get_class_count', self.get_class_count_callback)

        self.detections_dict = {}
    
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
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video reached.")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        # Keep a copy of the original frame for detection (YOLO expects BGR)
        orig_frame = frame.copy()

        # Apply any transformation for display (detection uses original frame)
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
                        class_name = self.model.names[int(classes[i])]
                        
                    else:
                        obj_hyp.hypothesis.class_id = str(int(classes[i]))
                        class_name = str(int(classes[i]))

                    if class_name in self.detections_dict:
                        self.detections_dict[class_name] += 1
                    else:
                        self.detections_dict[class_name] = 1

                    obj_hyp.hypothesis.score = float(confs[i])
                    detection.results.append(obj_hyp)

                    detections_array.detections.append(detection)

                    # Optionally, draw the detection on the display frame
                    cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                    label_text = f"{obj_hyp.hypothesis.class_id}: {obj_hyp.hypothesis.score:.2f}"
                    cv2.putText(frame, label_text, (int(xmin), int(ymin)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the detections message
        self.detections_pub.publish(detections_array)

        # Publish the class counts message
        self.build_and_publish_class_counts()
    
        # Display the video frame (with drawn detections, if any)
        cv2.imshow('Video Player', frame)
        cv2.waitKey(1)

    def build_and_publish_class_counts(self):
        class_count_array = ClassCountArray()
        class_count_array.header.stamp = self.get_clock().now().to_msg()
        class_count_array.header.frame_id = "camera_frame"

        for class_name, count in self.detections_dict.items():
            class_count = ClassCount()
            class_count.class_name = class_name
            class_count.count = count
            class_count_array.counts.append(class_count)

        self.class_count_pub.publish(class_count_array)   

    def get_class_count_callback(self, request, response):
        if request.class_name == 'all':
            for class_name, count in self.detections_dict.items():
                class_count = ClassCount()
                class_count.class_name = class_name
                class_count.count = count
                response.counts.append(class_count)
        else:
            if request.class_name in self.detections_dict:
                class_count = ClassCount()
                class_count.class_name = request.class_name
                class_count.count = self.detections_dict[request.class_name]
                response.counts.append(class_count)
            else:
                response.counts = []

        
        self.get_logger().info('Incoming request for counts for class : %s:' % request.class_name)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DashcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
