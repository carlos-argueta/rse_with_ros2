import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rse_fsd_msgs.action import FindObject

import cv2

# Import the YOLO model from Ultralytics
from ultralytics import YOLO

class FindObjectActionServer(Node):

    def __init__(self):
        super().__init__('find_object_action_server')
        self._action_server = ActionServer(
            self,
            FindObject,
            'find_object',
            self.execute_callback)
        
        # Load the YOLO model – update the model weights path if needed
        self.model = YOLO('yolo11x.pt')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        frame_count = 0
        real_frame_count = 0
        success = False
        
        result = FindObject.Result()

        cap = cv2.VideoCapture(goal_handle.request.video_path)

        if not cap.isOpened():
            print(f"Error: Could not open video at {goal_handle.request.video_path}")
            
            goal_handle.abort()

            result.success = success
            result.frames_processed = frame_count

            return result
        
        while frame_count < goal_handle.request.max_frames:
            
            ret, frame = cap.read()
            real_frame_count += 1
            if not ret:
                break  # End of video

            current_frame = frame.copy() # important to copy the frame, otherwise the transformations will accumulate
            
            # Run object detection on the original frame using YOLO
            if real_frame_count % 10 != 0:
                continue
                
            results = self.model(current_frame)
            frame_count += 1

            feedback_msg = FindObject.Feedback()
            feedback_msg.frames_processed = frame_count    
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.frames_processed))
            goal_handle.publish_feedback(feedback_msg)        

            if results and len(results) > 0:
                det_result = results[0]
                if det_result.boxes is not None:
                    classes = det_result.boxes.cls.cpu().numpy()   # shape: (N,)
                    
                    boxes = det_result.boxes.xyxy.cpu().numpy()  # shape: (N, 4)
                    confs = det_result.boxes.conf.cpu().numpy()    # shape: (N,)
                    classes = det_result.boxes.cls.cpu().numpy()   # shape: (N,)
                    
                    for i in range(len(boxes)):
                        xmin, ymin, xmax, ymax = boxes[i]
                        center_x = (xmin + xmax) / 2.0
                        center_y = (ymin + ymax) / 2.0
                        width = xmax - xmin
                        height = ymax - ymin

                        if hasattr(self.model, 'names') and self.model.names is not None:
                            class_name = self.model.names[int(classes[i])] 
                        else:
                            class_name = str(int(classes[i]))

                        if class_name == goal_handle.request.class_name and confs[i] >= goal_handle.request.confidence_threshold:
                            self.get_logger().info(f"Found object '{goal_handle.request.class_name}' in frame {frame_count}")

                            # Optionally, draw the detection on the display frame
                            cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                            label_text = f"{class_name}: {float(confs[i]):.2f}"
                            cv2.putText(frame, label_text, (int(xmin), int(ymin)-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            success = True
                            break
            
            # Display the video frame (with drawn detections, if any)
            cv2.imshow('Video Player', frame)
            cv2.waitKey(1)

            if success:
                break

        if success:
            time.sleep(5.0)  # Pause for 5 seconds to show the final frame

        cap.release()
        cv2.destroyAllWindows()

        goal_handle.succeed()

        result.success = success
        result.frames_processed = frame_count

        return result


def main(args=None):
    rclpy.init(args=args)

    find_object_action_server = FindObjectActionServer()

    rclpy.spin(find_object_action_server)


if __name__ == '__main__':
    main()