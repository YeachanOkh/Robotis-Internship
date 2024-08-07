import cv2
import os
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from mediapipe_utils import mediapipe_detection, draw_styled_landmarks, extract_keypoints, get_depth_at_landmark

mp_holistic = mp.solutions.holistic  # Holistic model
mp_drawing = mp.solutions.drawing_utils  # Drawing utilities

DATA_PATH = os.path.join('MP_Data')  # Path for exported data, numpy arrays
actions = np.array(['Good Job','Hello', 'Fist Bump','High Five', 'Hungry', 'Thirsty', 'Congratulations','Take Care', 'Handshake'])# 'Good Job','Hello', 'Fist Bump','High Five', 'Hungry', 'Thirsty', 'Congratulations','Take Care', 'Handshake' # Actions that we try to detect
no_sequences = 40  # Thirty videos worth of data
sequence_length = 30  # Videos are going to be 30 frames in length

for action in actions:
    for sequence in range(no_sequences):
        try:
            os.makedirs(os.path.join(DATA_PATH, action, str(sequence)))
        except:
            pass

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

def collect_data():
    cap = cv2.VideoCapture(6)
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        for action in actions:
            for sequence in range(no_sequences):
                for frame_num in range(sequence_length):
                    ret, frame = cap.read()
                    depth_frame = pipeline.wait_for_frames().get_depth_frame()

                    image, results = mediapipe_detection(frame, holistic)
                    draw_styled_landmarks(image, results)

                    if frame_num == 0:
                        cv2.putText(image, 'STARTING COLLECTION', (120, 200),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4, cv2.LINE_AA)
                        cv2.putText(image, 'Collecting frames for {} Video Number {}'.format(action, sequence), (15, 12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.imshow('OpenCV Feed', image)
                        cv2.waitKey(500)
                    else:
                        cv2.putText(image, 'Collecting frames for {} Video Number {}'.format(action, sequence), (15, 12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.imshow('OpenCV Feed', image)

                    keypoints = extract_keypoints(results, depth_frame, frame.shape)
                    npy_path = os.path.join(DATA_PATH, action, str(sequence), str(frame_num))
                    np.save(npy_path, keypoints)

                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        break

    cap.release()
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    collect_data()