from rse_fsd_msgs.srv import GetClassCount
from rse_fsd_msgs.msg import ClassCount
from vision_msgs.msg import Detection2DArray

import rclpy
from rclpy.node import Node

import time


class GetClassCountsServer(Node):

    def __init__(self):
        super().__init__('get_class_count_server')

        # Create detections subscriber
        self.subscription = self.create_subscription(
            Detection2DArray,
            'detections',
            self.detections_callback,
            10)
        
        # Create the service
        self.srv = self.create_service(GetClassCount, 'get_class_count', self.get_class_count_callback)

        self.detections_dict = {}

    def detections_callback(self, msg):
        for detection in msg.detections:
            class_name = detection.results[0].hypothesis.class_id
            if class_name in self.detections_dict:
                self.detections_dict[class_name] += 1
            else:
                self.detections_dict[class_name] = 1

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

        # Lesson 6
        # Sleep for 5 seconds to simulate a long-running service
        for i in range(5):
            self.get_logger().info(f"Sleeping for {5-i} seconds...")
            time.sleep(1)
            
        return response


def main():
    rclpy.init()

    class_counts_server = GetClassCountsServer()

    rclpy.spin(class_counts_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()