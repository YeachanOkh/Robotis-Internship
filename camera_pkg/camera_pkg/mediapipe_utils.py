import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs

mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils

def mediapipe_detection(image, model):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = model.process(image)
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image, results

def draw_styled_landmarks(image, results):
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
    if results.left_hand_landmarks:
        mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    if results.right_hand_landmarks:
        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

def get_depth_at_landmark(depth_frame, x, y):
    width, height = depth_frame.get_width(), depth_frame.get_height()
    if 0 <= x < width and 0 <= y < height:
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth = depth_frame.get_distance(x, y)
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
        return depth_point[2]
    else:
        return None

def extract_keypoints(results, depth_frame, frame_shape):
    pose = np.array([[res.x, res.y, res.z, res.visibility] for res in results.pose_landmarks.landmark]).flatten() if results.pose_landmarks else np.zeros(33*4)

    lh = np.array([[res.x, res.y, 0.0] for res in results.left_hand_landmarks.landmark]).flatten() if results.left_hand_landmarks else np.zeros(21*3)
    if results.left_hand_landmarks:
        wrist_landmark = results.left_hand_landmarks.landmark[mp_holistic.HandLandmark.WRIST]
        x, y = int(wrist_landmark.x * frame_shape[1]), int(wrist_landmark.y * frame_shape[0])
        lh_z = get_depth_at_landmark(depth_frame, x, y)
        if lh_z is not None:
            print(f"Left wrist depth: {lh_z} meters")  # Print depth value for left wrist
            lh[2] = lh_z  # Replace the z value of the wrist landmark

    rh = np.array([[res.x, res.y, 0.0] for res in results.right_hand_landmarks.landmark]).flatten() if results.right_hand_landmarks else np.zeros(21*3)
    if results.right_hand_landmarks:
        wrist_landmark = results.right_hand_landmarks.landmark[mp_holistic.HandLandmark.WRIST]
        x, y = int(wrist_landmark.x * frame_shape[1]), int(wrist_landmark.y * frame_shape[0])
        rh_z = get_depth_at_landmark(depth_frame, x, y)
        if rh_z is not None:
            print(f"Right wrist depth: {rh_z} meters")  # Print depth value for right wrist
            rh[2] = rh_z  # Replace the z value of the wrist landmark

    return np.concatenate([pose, lh, rh])
