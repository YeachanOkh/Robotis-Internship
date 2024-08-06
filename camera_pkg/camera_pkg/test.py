import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark
import pyrealsense2 as rs
import time
import asyncio

class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.publisher_ = self.create_publisher(String, 'gesture_done', 10)
        self.subscription = self.create_subscription(
            String,
            'gesture_done',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.actions = np.array(['Good Job', 'Hello', 'Fist Bump', 'High Five', 'Hungry', 'Thirsty',
                                 'Congratulations', 'Take Care', 'Handshake'])

        # Load the model
        self.model = load_model('action.h5')

        # Initialize MediaPipe holistic model
        self.mp_holistic = mp.solutions.holistic  # Holistic model
        self.mp_drawing = mp.solutions.drawing_utils  # Drawing utilities

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        self.sequence = []
        self.threshold = 0.8
        self.gesture_duration = 8  # seconds
        self.rest_duration = 6  # seconds
        self.gesture_start_time = None
        self.rest_start_time = None
        self.detecting_gesture = False
        self.last_predicted_gesture = None  # To store the last predicted gesture

        self.cap = cv2.VideoCapture(6)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.loop = asyncio.get_event_loop()
        self.websocket = self.loop.run_until_complete(self.connect_websocket())

    async def connect_websocket(self):
        return await websockets.connect("ws://rgs.bansheeuav.tech:8080")

    def prob_viz(self, res, actions, input_frame):
        output_frame = input_frame.copy()
        max_prob = np.argmax(res)

        # Get the width of the frame
        frame_width = output_frame.shape[1]

        # Get the width and height of the text
        (text_width, text_height), _ = cv2.getTextSize(actions[max_prob],
                                                       cv2.FONT_HERSHEY_SIMPLEX, 1,
                                                       2)

        # Calculate the position for the text to be centered
        text_x = (frame_width - text_width) // 2
        text_y = 40  # Slightly below the top of the frame

        cv2.putText(output_frame, actions[max_prob], (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return output_frame

    def in_range(self, x, y, shape):
        return 0 <= x < shape[1] and 0 <= y < shape[0]

    def timer_callback(self):
        current_time = time.time()
        if self.detecting_gesture:
            # Check if gesture duration is over
            if self.gesture_start_time is None:
                self.gesture_start_time = current_time
            elif current_time - self.gesture_start_time > self.gesture_duration:
                self.detecting_gesture = False
                self.rest_start_time = current_time
                self.gesture_start_time = None
            else:
                # Make detections
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().info("Error: Could not read frame.")
                    return
                image, results = mediapipe_detection(frame, self.mp_holistic)
                draw_styled_landmarks(image, results)

                # Extract keypoints
                depth_frame = self.pipeline.wait_for_frames().get_depth_frame()
                keypoints = extract_keypoints(results, depth_frame, frame.shape)

                # Print wrist landmarks distances
                if results.left_hand_landmarks:
                    left_wrist = results.left_hand_landmarks.landmark[
                        self.mp_holistic.HandLandmark.WRIST]
                    x, y = int(left_wrist.x * frame.shape[1]), int(left_wrist.y * frame.shape[0])
                    if self.in_range(x, y, frame.shape):
                        left_wrist_distance = get_depth_at_landmark(depth_frame, x, y)
                        self.get_logger().info(f"Left Wrist Distance: {left_wrist_distance} meters")

                if results.right_hand_landmarks:
                    right_wrist = results.right_hand_landmarks.landmark[
                        self.mp_holistic.HandLandmark.WRIST]
                    x, y = int(right_wrist.x * frame.shape[1]), int(right_wrist.y * frame.shape[0])
                    if self.in_range(x, y, frame.shape):
                        right_wrist_distance = get_depth_at_landmark(depth_frame, x, y)
                        self.get_logger().info(f"Right Wrist Distance: {right_wrist_distance} meters")

                self.sequence.append(keypoints)
                self.sequence = self.sequence[-30:]

                if len(self.sequence) == 30:
                    res = self.model.predict(np.expand_dims(self.sequence, axis=0))[0]

                    if np.max(res) > self.threshold:
                        image = self.prob_viz(res, self.actions, image)
                        self.last_predicted_gesture = self.actions[np.argmax(res)]
                        self.publisher_.publish(String(data=self.last_predicted_gesture))

                # Show gesture countdown timer
                remaining_time = self.gesture_duration - (current_time - self.gesture_start_time)
                cv2.putText(image, f'Gesture Time: {remaining_time:.2f}s',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            # Check if rest duration is over
            if self.rest_start_time is None:
                self.rest_start_time = current_time
            elif current_time - self.rest_start_time > self.rest_duration:
                self.detecting_gesture = True
                self.rest_start_time = None
            else:
                # Show rest countdown timer and last predicted gesture
                remaining_time = self.rest_duration - (current_time - self.rest_start_time)
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().info("Error: Could not read frame.")
                    return
                image = frame.copy()
                cv2.putText(image, f'Waiting for Response: {remaining_time:.2f}s',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                if self.last_predicted_gesture:
                    cv2.putText(image, f'Last Gesture: {self.last_predicted_gesture}',
                                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('OpenCV Feed', image)
        _, buffer = cv2.imencode('.jpg', image)
        self.loop.run_until_complete(self.websocket.send(buffer.tobytes()))
        if cv2.waitKey(10) & 0xFF == ord('q'):
            self.timer.cancel()
            self.pipeline.stop()
            self.cap.release()
            cv2.destroyAllWindows()

    def listener_callback(self, msg):
        self.get_logger().info('Received gesture: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    gesture_recognition_node = GestureRecognitionNode()
    rclpy.spin(gesture_recognition_node)
    gesture_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
