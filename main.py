import cv2, djitellopy, pupil_apriltags
import numpy as np

# drone = djitellopy.Tello()
cap = cv2.VideoCapture()
april = pupil_apriltags.Detector(
    families="tagStandard41h12",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# are the dimensions set?
setdims = False

def main():
    while True:
        if(cap.isOpened()):
            global setdims
            if setdims == False:
                setdims = True
                cap.set(3, 320) # dimensions are yes
                cap.set(4, 240) # yes are dimensions
            ret, frame = cap.read()
            # img = drone.get_frame_read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = april.detect(gray, estimate_tag_pose=False)
            for tag in tags:
                print("Tag #" + str(tag.tag_id) + " found!")
                # print("Distance from tag #" + tag.tag_id + ": " + tag.pose_t)
                (ptA, ptB, ptC, ptD) = tags[0].corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
                cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
                cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
                cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
                cv2.imshow("drone", frame)
            if len(tags) == 0:
                cv2.imshow("drone", frame)
            cv2.waitKey(25)
        else:
            cap.open(0)


if __name__ == "__main__":
    main()

cap.release()
cv2.destroyAllWindows()