import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from camera_pkg.mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark
import pyrealsense2 as rs
import time
from websockets.sync.client import connect
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import asyncio
import websockets

# Load the model
model = load_model('/home/ubuntu/robotis-internship/src/Robotis-Internship/camera_pkg/camera_pkg/action.h5')

# Initialize message
publishermsg = String()
subscribermsg = ""

class camera(Node):
    def __init__(self):
        super().__init__("camera_working")


        self.subscription = self.create_subscription(String, 'gesture_done', self.listener_callback, 10)
        
        self.subscription

        self.publisher_ = self.create_publisher(String, 'camera_gesture', 10)

        asyncio.run(conn())
    
    def listener_callback(self, msg):
        self.subscribermsg = msg
        self.get_logger().info(f'Received gesture command from camera node: {msg.data}')

    def publish_feedback(self, feedback):
        msg = String()
        msg.data = feedback
        self.publisher_.publish(msg)
        self.get_logger().info(f'Gesture finished: {msg.data}')
            

async def send_frame(websocket):
    async def send_pings():
        while True:
            try:
                await websocket.ping()
                await asyncio.sleep(20)  # Adjust the interval as needed
            except websockets.ConnectionClosed:
                break

    ping_task = asyncio.create_task(send_pings())



    actions = np.array(
        ['Good Job', 'Hello', 'Fist Bump', 'High Five', 'Hungry', 'Thirsty',
         'Congratulations', 'Take Care', 'Handshake'])

    # Initialize MediaPipe holistic model
    mp_holistic = mp.solutions.holistic  # Holistic model
    mp_drawing = mp.solutions.drawing_utils  # Drawing utilities

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)


    def prob_viz(res, actions, input_frame):
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


    sequence = []
    threshold = 0.8

    cap = cv2.VideoCapture(6)

    # Gesture timing variables
    gesture_duration = 8  # seconds
    rest_duration = 6  # seconds
    gesture_start_time = None
    rest_start_time = None
    detecting_gesture = False
    last_predicted_gesture = ""  # To store the last predicted gesture


    def in_range(x, y, shape):
        return 0 <= x < shape[1] and 0 <= y < shape[0]


    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open video device.")
    else:
        with mp_holistic.Holistic(min_detection_confidence=0.5,
                                  min_tracking_confidence=0.5) as holistic:
            while cap.isOpened():
                ret, frame = cap.read()

                # Check if the frame was read correctly
                if not ret or frame is None:
                    print("Error: Could not read frame.")
                    break

                current_time = time.time()
                image = frame.copy()  # Initialize the image

                if detecting_gesture:
                    # Check if gesture duration is over
                    if gesture_start_time is None:
                        gesture_start_time = current_time
                    elif current_time - gesture_start_time > gesture_duration:
                        detecting_gesture = False
                        rest_start_time = current_time
                        gesture_start_time = None
                    else:
                        # Make detections
                        image, results = mediapipe_detection(frame, holistic)

                        # Draw landmarks
                        draw_styled_landmarks(image, results)

                        # Extract keypoints
                        depth_frame = pipeline.wait_for_frames().get_depth_frame()
                        keypoints = extract_keypoints(results, depth_frame,
                                                      frame.shape)

                        # Print wrist landmarks distances
                        if results.left_hand_landmarks:
                            left_wrist = results.left_hand_landmarks.landmark[
                                mp_holistic.HandLandmark.WRIST]
                            x, y = int(left_wrist.x * frame.shape[1]), int(
                                left_wrist.y * frame.shape[0])
                            if in_range(x, y, frame.shape):
                                left_wrist_distance = get_depth_at_landmark(
                                    depth_frame, x, y)
                                print(
                                    f"Left Wrist Distance: {left_wrist_distance} meters")

                        if results.right_hand_landmarks:
                            right_wrist = results.right_hand_landmarks.landmark[
                                mp_holistic.HandLandmark.WRIST]
                            x, y = int(right_wrist.x * frame.shape[1]), int(
                                right_wrist.y * frame.shape[0])
                            if in_range(x, y, frame.shape):
                                right_wrist_distance = get_depth_at_landmark(
                                    depth_frame, x, y)
                                print(
                                    f"Right Wrist Distance: {right_wrist_distance} meters")

                        sequence.append(keypoints)
                        sequence = sequence[-30:]

                        if len(sequence) == 30:
                            res = model.predict(np.expand_dims(sequence, axis=0))[
                                0]

                            if np.max(res) > threshold:
                                image = prob_viz(res, actions, image)
                                last_predicted_gesture = actions[np.argmax(
                                    res)]  # Store the last predicted gesture

                        # Show gesture countdown timer
                        remaining_time = gesture_duration - (
                                    current_time - gesture_start_time)
                        cv2.putText(image, f'Gesture Time: {remaining_time:.2f}s',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 255, 255), 2, cv2.LINE_AA)

                else:
                    # Check if rest duration is over
                    if rest_start_time is None:
                        rest_start_time = current_time
                    elif current_time - rest_start_time > rest_duration:
                        detecting_gesture = True
                        rest_start_time = None
                    else:
                        # Show rest countdown timer and last predicted gesture
                        remaining_time = rest_duration - (
                                    current_time - rest_start_time)
                        cv2.putText(image,
                                    f'Waiting for Response: {remaining_time:.2f}s',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 255, 255), 2, cv2.LINE_AA)
                        if last_predicted_gesture:
                            cv2.putText(image,
                                        f'Last Gesture: {last_predicted_gesture}',
                                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                        (255, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow('OpenCV Feed', image)
                #added code for webscoketing
                _, buffer = cv2.imencode('.jpg', image)
                publishermsg.data = last_predicted_gesture
                websocket.send(buffer.tobytes())
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
                #REMOVE OR CHANGE IF AND ONLY IF YOU NEED MORE FRAMES
                time.sleep(0.01)

                

        cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()
    ping_task.cancel()

async def conn():
    with connect("ws://rgs.bansheeuav.tech:8080") as websocket:
        await send_frame(websocket)

def main(args=None):
    rclpy.init(args=args)
    node = camera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()