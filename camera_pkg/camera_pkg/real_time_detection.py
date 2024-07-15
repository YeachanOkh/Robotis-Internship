import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark
from data_preparation import actions
import pyrealsense2 as rs

# Load the model
model = load_model('action.h5')

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
    (text_width, text_height), _ = cv2.getTextSize(actions[max_prob], cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
    
    # Calculate the position for the text to be centered
    text_x = (frame_width - text_width) // 2
    text_y = 40  # Slightly below the top of the frame
    
    cv2.putText(output_frame, actions[max_prob], (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    return output_frame

sequence = []
threshold = 0.8

cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device.")
else:
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            ret, frame = cap.read()
            
            # Check if the frame was read correctly
            if not ret or frame is None:
                print("Error: Could not read frame.")
                break

            # Make detections
            image, results = mediapipe_detection(frame, holistic)

            # Draw landmarks
            draw_styled_landmarks(image, results)

            # Extract keypoints
            depth_frame = pipeline.wait_for_frames().get_depth_frame()
            keypoints = extract_keypoints(results, depth_frame, frame.shape)
            
            # Print wrist landmarks distances
            if results.left_hand_landmarks:
                left_wrist = results.left_hand_landmarks.landmark[mp_holistic.HandLandmark.WRIST]
                left_wrist_distance = get_depth_at_landmark(depth_frame, int(left_wrist.x * frame.shape[1]), int(left_wrist.y * frame.shape[0]))
                print(f"Left Wrist Distance: {left_wrist_distance} meters")
            
            if results.right_hand_landmarks:
                right_wrist = results.right_hand_landmarks.landmark[mp_holistic.HandLandmark.WRIST]
                right_wrist_distance = get_depth_at_landmark(depth_frame, int(right_wrist.x * frame.shape[1]), int(right_wrist.y * frame.shape[0]))
                print(f"Right Wrist Distance: {right_wrist_distance} meters")

            sequence.append(keypoints)
            sequence = sequence[-30:]

            if len(sequence) == 30:
                res = model.predict(np.expand_dims(sequence, axis=0))[0]

                if np.max(res) > threshold:
                    image = prob_viz(res, actions, image)

            cv2.imshow('OpenCV Feed', image)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    cap.release()
pipeline.stop()
cv2.destroyAllWindows()