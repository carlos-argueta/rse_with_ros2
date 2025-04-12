import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rse_fsd_msgs.action import FindObject


class FindObjectActionClient(Node):

    def __init__(self):
        super().__init__('find_object_action_client')
        self._action_client = ActionClient(self, FindObject, 'find_object')

    def send_goal(self, video_path, max_frames, class_name, confidence_threshold = 0.9):
        goal_msg = FindObject.Goal()
        goal_msg.video_path = video_path
        goal_msg.max_frames = max_frames
        goal_msg.class_name = class_name
        goal_msg.confidence_threshold = confidence_threshold

        self.get_logger().info('Waiting for an action server ...')
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Object found? {0}'.format(result.success))
        self.get_logger().info('Frames processed: {0}'.format(result.frames_processed
        ))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.frames_processed))


def main(args=None):
    rclpy.init(args=args)

    action_client = FindObjectActionClient()

    video_path = '/home/carlos/ros2_ws/src/rse_fsd/videos/m3-res_480p.mp4'
    max_frames = 100
    class_name = 'truck'
    action_client.send_goal(video_path, max_frames, class_name)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()