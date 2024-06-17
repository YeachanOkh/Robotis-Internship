import cv2
import mediapipe as mp
import time
import numpy as np
import pyrealsense2 as rs

class handDetector():
    def __init__(self, mode=False, maxHands=1, modelComplexity=1, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplexity = modelComplexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplexity, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def rsToCv(self, pipeline, align):
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return [depth_image, color_image, depth_frame]

    def findHands(self, imgRGB, draw=True):
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(imgRGB, handLms, self.mpHands.HAND_CONNECTIONS)
        return imgRGB
    
    def findPosition(self, imgRGB, handNo=0, draw=False):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                self.h = imgRGB.shape[0]
                self.w = imgRGB.shape[1]
                self.cx, self.cy = int(lm.x*self.w), int(lm.y*self.h)
                lmList.append([id, self.cx, self.cy])
                if draw:
                    cv2.circle(imgRGB, (self.cx, self.cy), 15, (255, 0, 255), cv2.FILLED)
                # Also print the distance from the depth sensing camera at these certain points
        return lmList
    
    def findDistance (self, lmList, depth_frame, color_image, show=False):
        distance = 0
        if len(lmList) != 0:
            if 0 <= self.cx < self.w & 0 <= self.cy < self.h:
                print("Got here")
                distance = 0
                distance = depth_frame.get_distance(self.cx, self.cy) * 1000
                cv2.circle(color_image, (self.cx, self.cy), 10, (255, 0, 0), -1)
                cv2.putText(color_image, f'{distance:.0f} mm', (self.cx, self.cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # print(f"Landmark 0 coordinates: (x: {self.cx}, y: {self.cy}), Distance: {distance:.0f} mm")


                        

def main():
    pTime = 0
    cTime = 0
    detector = handDetector()

    # Initializing RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    while True:
        depthImg = detector.rsToCv(pipeline, align)[0]
        colorImg = detector.rsToCv(pipeline, align)[1]
        depthFrame = detector.rsToCv(pipeline, align)[2]

        imgHands = detector.findHands(colorImg)
        lmList = detector.findPosition(imgHands)

        if lmList != []:
            print(lmList[0])

        distance = detector.findDistance(lmList, depthFrame, colorImg)

        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime

        cv2.putText(imgHands, str(int(fps)), (5,55), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 0), 2)
        cv2.imshow("Capture Device", colorImg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
