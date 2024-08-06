

import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark
import pyrealsense2 as rs
import time
import threading
import asyncio
import websockets

actions = np.array(['Good Job', 'Hello', 'Fist Bump', 'High Five', 'Hungry', 'Thirsty',
                    'Congratulations', 'Take Care', 'Handshake'])

# Load the model
model = load_model('action.h5')

# Initialize MediaPipe holistic model
mp_holistic = mp.solutions.holistic  # Holistic model
mp_drawing = mp.solutions.drawing_utils  # Drawing utilities

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipeline.start(config)

sequence = []
threshold = 0.8

cap = cv2.VideoCapture(6)

# Gesture timing variables
gesture_duration = 4  # seconds
rest_duration = 4  # secondss
gesture_start_time = None
rest_start_time = None
detecting_gesture = False
last_predicted_gesture = None  # To store the last predicted gesture

frame = None
processed_frame = None
lock = threading.Lock()


def in_range(x, y, shape):
    return 0 <= x < shape[1] and 0 <= y < shape[0]


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


def capture_frames():
    global frame
    while cap.isOpened():
        ret, captured_frame = cap.read()
        if not ret or captured_frame is None:
            print("Error: Could not read frame.")
            break
        with lock:
            frame = captured_frame.copy()


async def send_frame(websocket):
    while True:
        with lock:
            if processed_frame is None:
                continue
            current_frame = processed_frame.copy()
        _, buffer = cv2.imencode('.jpg', current_frame)
        await websocket.send(buffer.tobytes())
        await asyncio.sleep(0.03)  # Adjust sleep time for desired frame rate


async def websocket_handler():
    async with websockets.connect("ws://rgs.bansheeuav.tech:8080") as websocket:
        await send_frame(websocket)


def process_frames():
    global gesture_start_time, rest_start_time, detecting_gesture, last_predicted_gesture, sequence, processed_frame
    with mp_holistic.Holistic(min_detection_confidence=0.5,
                              min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            with lock:
                if frame is None:
                    continue
                process_frame = frame.copy()

            current_time = time.time()
            image = process_frame.copy()  # Initialize the image

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
                    image, results = mediapipe_detection(process_frame, holistic)

                    # Draw landmarks
                    draw_styled_landmarks(image, results)

                    # Extract keypoints
                    depth_frame = pipeline.wait_for_frames().get_depth_frame()
                    keypoints = extract_keypoints(results, depth_frame,
                                                  process_frame.shape)

                    # Print wrist landmarks distances
                    if results.left_hand_landmarks:
                        left_wrist = results.left_hand_landmarks.landmark[
                            mp_holistic.HandLandmark.WRIST]
                        x, y = int(left_wrist.x * process_frame.shape[1]), int(
                            left_wrist.y * process_frame.shape[0])
                        if in_range(x, y, process_frame.shape):
                            left_wrist_distance = get_depth_at_landmark(
                                depth_frame, x, y)
                            print(
                                f"Left Wrist Distance: {left_wrist_distance} meters")

                    if results.right_hand_landmarks:
                        right_wrist = results.right_hand_landmarks.landmark[
                            mp_holistic.HandLandmark.WRIST]
                        x, y = int(right_wrist.x * process_frame.shape[1]), int(
                            right_wrist.y * process_frame.shape[0])
                        if in_range(x, y, process_frame.shape):
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

            with lock:
                processed_frame = image.copy()

            cv2.imshow('OpenCV Feed', image)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()


# Start threads for capture and processing
capture_thread = threading.Thread(target=capture_frames)
processing_thread = threading.Thread(target=process_frames)

capture_thread.start()
processing_thread.start()

# Run the WebSocket handler
asyncio.run(websocket_handler())
