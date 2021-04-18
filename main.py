import cv2, djitellopy, pupil_apriltags, json, math, threading
import numpy as np

drone = djitellopy.Tello()
# cap = cv2.VideoCapture()
april = pupil_apriltags.Detector(
    families="tagStandard41h12",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

TAG_SIZE = 0.2

drone.connect()
drone.streamon()

print("Drone battery: " + str(drone.get_battery()))

print("Reading camera calibration...")
try:
    f = open("calibration/calib.json","r")
except FileNotFoundError:
    print("Error, the calibration file doesn't exist!\nPlease calibrate it using the calibration/calibrate.py file.\nNote: You need to get images first. There is a utility called capture.py in the same directory.")
    exit(1)


print("File read, checking JSON...")
try:
    data = json.load(f)
except json.JSONDecodeError:
    print("JSON is invalid!")
    exit(1)

print("JSON Okay!")
f.close()

def main():
    while True:
            frame = drone.get_frame_read().frame
            gray = None
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            except cv2.error:
                print("[!] No frame!") 
            tags = april.detect(gray, estimate_tag_pose=True, camera_params=[data['fx'], data['fy'], data['cx'], data['cy']], tag_size=TAG_SIZE)
            handled = False
            for tag in tags:
                distance = tag.pose_t[0] + tag.pose_t[1] + tag.pose_t[2]
                print("Distance from tag #" + str(tag.tag_id) + ": " + str(distance) + " meters!")
                look_at(tag.pose_t)
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
                frame = cv2.putText(frame, str(round(distance[0], 2)), (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (150,0,0), 2, cv2.LINE_AA)
                key = cv2.waitKey(10) & 0xFF
                handled = True
                if handleKey(key, pnt=tag.pose_t):
                    print("Closing!")
                    break
            cv2.imshow("drone", frame)
            # if len(tags) == 0:
            #     cv2.imshow("drone", frame)
            if not handled:
                key = cv2.waitKey(10) & 0xFF
                if handleKey(key):
                    print("Closing!")
                    break
        # else:
            # cap.open(0)

def look_at(point):
    x = point[0] # X in 3d space
    y = point[2] # Z in 3d space
    # we don't care about the y axis yet

    rotx = math.atan2(x, y) * 180/math.pi

    print(rotx)

    return rotx

def handleKey(key, pnt=None):
    if key == ord("q"):
        return True
    if key == ord("t"):
        def takeoff():
            drone.takeoff()
        threading.Thread(target=takeoff).start()
    if key == ord("\\"):
        def land():
            drone.land()
        threading.Thread(target=land).start()
    if key == ord("y"):
        # look at
        if pnt is None:
            print("NO TAG!")
            return
        angle = look_at(pnt)
        if(angle > 0):
            # cw
            def cw(angle, drone):
                drone.rotate_clockwise(int(angle))
            threading.Thread(target=cw, args=(angle, drone)).start()
        else:
            # ccw
            def ccw(angle, drone):
                drone.rotate_counter_clockwise(int(-angle))
            threading.Thread(target=ccw, args=(angle, drone)).start()



if __name__ == "__main__":
    main()

cv2.destroyAllWindows()