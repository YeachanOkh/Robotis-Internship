import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from camera_pkg.mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark
import pyrealsense2 as rs
import time
import asyncio
import websockets

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'gesture_image', 10)
        self.actions = np.array(
            ['Good Job', 'Hello', 'Fist Bump', 'High Five', 'Hungry', 'Thirsty',
             'Congratulations', 'Take Care', 'Handshake'])

        self.model = load_model('action.h5')

        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        self.sequence = []
        self.threshold = 0.8

        self.gesture_duration = 8
        self.rest_duration = 6
        self.gesture_start_time = None
        self.rest_start_time = None
        self.detecting_gesture = False
        self.last_predicted_gesture = None

        self.cap = cv2.VideoCapture(6)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device.")
            return

        self.loop = asyncio.get_event_loop()
        self.timer = self.create_timer(0.01, self.process_frame)
        self.loop.create_task(self.conn())

    async def conn(self):
        async with websockets.connect("ws://rgs.bansheeuav.tech:8080") as websocket:
            await self.send_frame(websocket)

    async def send_frame(self, websocket):
        async def send_pings():
            while True:
                try:
                    await websocket.ping()
                    await asyncio.sleep(20)
                except websockets.ConnectionClosed:
                    break

        ping_task = asyncio.create_task(send_pings())

        with self.mp_holistic.Holistic(min_detection_confidence=0.5,
                                       min_tracking_confidence=0.5) as holistic:
            while self.cap.isOpened():
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().error("Could not read frame.")
                    break

                current_time = time.time()
                image = frame.copy()

                if self.detecting_gesture:
                    if self.gesture_start_time is None:
                        self.gesture_start_time = current_time
                    elif current_time - self.gesture_start_time > self.gesture_duration:
                        self.detecting_gesture = False
                        self.rest_start_time = current_time
                        self.gesture_start_time = None
                    else:
                        image, results = mediapipe_detection(frame, holistic)
                        draw_styled_landmarks(image, results)

                        depth_frame = self.pipeline.wait_for_frames().get_depth_frame()
                        keypoints = extract_keypoints(results, depth_frame, frame.shape)

                        if results.left_hand_landmarks:
                            left_wrist = results.left_hand_landmarks.landmark[
                                self.mp_holistic.HandLandmark.WRIST]
                            x, y = int(left_wrist.x * frame.shape[1]), int(
                                left_wrist.y * frame.shape[0])
                            if self.in_range(x, y, frame.shape):
                                left_wrist_distance = get_depth_at_landmark(
                                    depth_frame, x, y)
                                self.get_logger().info(
                                    f"Left Wrist Distance: {left_wrist_distance} meters")

                        if results.right_hand_landmarks:
                            right_wrist = results.right_hand_landmarks.landmark[
                                self.mp_holistic.HandLandmark.WRIST]
                            x, y = int(right_wrist.x * frame.shape[1]), int(
                                right_wrist.y * frame.shape[0])
                            if self.in_range(x, y, frame.shape):
                                right_wrist_distance = get_depth_at_landmark(
                                    depth_frame, x, y)
                                self.get_logger().info(
                                    f"Right Wrist Distance: {right_wrist_distance} meters")

                        self.sequence.append(keypoints)
                        self.sequence = self.sequence[-30:]

                        if len(self.sequence) == 30:
                            res = self.model.predict(np.expand_dims(self.sequence, axis=0))[0]

                            if np.max(res) > self.threshold:
                                image = self.prob_viz(res, self.actions, image)
                                self.last_predicted_gesture = self.actions[np.argmax(res)]

                        remaining_time = self.gesture_duration - (
                                current_time - self.gesture_start_time)
                        cv2.putText(image, f'Gesture Time: {remaining_time:.2f}s',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 255, 255), 2, cv2.LINE_AA)

                else:
                    if self.rest_start_time is None:
                        self.rest_start_time = current_time
                    elif current_time - self.rest_start_time > self.rest_duration:
                        self.detecting_gesture = True
                        self.rest_start_time = None
                    else:
                        remaining_time = self.rest_duration - (
                                current_time - self.rest_start_time)
                        cv2.putText(image,
                                    f'Waiting for Response: {remaining_time:.2f}s',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 255, 255), 2, cv2.LINE_AA)
                        if self.last_predicted_gesture:
                            cv2.putText(image,
                                        f'Last Gesture: {self.last_predicted_gesture}',
                                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                        (255, 255, 255), 2, cv2.LINE_AA)

                _, buffer = cv2.imencode('.jpg', image)
                await websocket.send(buffer.tobytes())
                await asyncio.sleep(0.01)

        self.cap.release()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        ping_task.cancel()

    def prob_viz(self, res, actions, input_frame):
        output_frame = input_frame.copy()
        max_prob = np.argmax(res)
        frame_width = output_frame.shape[1]
        (text_width, text_height), _ = cv2.getTextSize(actions[max_prob],
                                                       cv2.FONT_HERSHEY_SIMPLEX, 1,
                                                       2)
        text_x = (frame_width - text_width) // 2
        text_y = 40
        cv2.putText(output_frame, actions[max_prob], (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return output_frame

    def in_range(self, x, y, shape):
        return 0 <= x < shape[1] and 0 <= y < shape[0]

    def process_frame(self):
        pass  # This is a placeholder to keep the node active

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
