import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
from mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints
from data_preparation import actions

# Load the model
model = load_model('action.h5')

# Initialize MediaPipe holistic model
mp_holistic = mp.solutions.holistic  # Holistic model
mp_drawing = mp.solutions.drawing_utils  # Drawing utilities

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
            keypoints = extract_keypoints(results)
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
cv2.destroyAllWindows()
