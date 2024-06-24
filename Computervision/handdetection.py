import cv2
import numpy
import mediapipe as mp
import pyrealsense2 as rs
import time

# Initialize MediaPipe hands and drawing modules
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize hands module with some parameters
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

try:
    prev_time = 0

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the BGR image to RGB
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Process the image and find hands
        results = hands.process(image_rgb)

        # Draw hand landmarks and extract the coordinates of landmark 0
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks
                mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Extract coordinates of landmark 0
                landmark_0 = hand_landmarks.landmark[0]
                h, w, _ = color_image.shape
                cx, cy = int(landmark_0.x * w), int(landmark_0.y * h)

                # Ensure coordinates are within the valid range
                if 0 <= cx < w and 0 <= cy < h:
                    # Measure distance using the depth frame and convert to millimeters
                    distance = depth_frame.get_distance(cx, cy) * 1000  # Convert meters to millimeters

                    # Draw a circle at landmark 0 and display the distance
                    cv2.circle(color_image, (cx, cy), 10, (255, 0, 0), -1)
                    cv2.putText(color_image, f'{distance:.0f} mm', (cx, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Print coordinates and distance
                    print(f"Landmark 0 coordinates: (x: {cx}, y: {cy}), Distance: {distance:.0f} mm")

        # Calculate FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        # Display FPS on the frame
        cv2.putText(color_image, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display the output
        cv2.imshow('Hand Tracking', color_image)

        # Break the loop on pressing 'q'
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

finally:
    # Release resources
    pipeline.stop()
    cv2.destroyAllWindows()
    hands.close()
