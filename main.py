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

# AprilTag size in meters
TAG_SIZE = 0.2
# Frames to go bye until mission is deemed "failure" and lands
FAILURE_THRESHOLD = 10
# distance to travel before adjusting
ADJ_DISTANCE = 300

# random placeholder value until actually set
tag_distance = 30000
distance_phase = 1
failure_count = 0
travelling = False

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
            global failure_count
            global travelling
            global distance_phase
            frame = drone.get_frame_read().frame
            gray = None
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            except cv2.error:
                print("[!] No frame!") 
            tags = april.detect(gray, estimate_tag_pose=True, camera_params=[data['fx'], data['fy'], data['cx'], data['cy']], tag_size=TAG_SIZE)
            handled = False
            if travelling and len(tags) == 0:
                failure_count += 1
            else:
                failure_count = 0
            if failure_count >= 10:
                print("FAILURE!!!")
                failure_count = 0
                #stop
                drone.send_rc_control(0,0,0,0)
                #land
                drone.land()
                break
            for tag in tags:
                # in cm
                pnt = tag.pose_t
                distance = int(math.sqrt(math.pow(pnt[0],2) + math.pow(pnt[2],2)) * 100)
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
                frame = cv2.putText(frame, str(round(distance, 2)), (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (150,0,0), 2, cv2.LINE_AA)
                key = cv2.waitKey(10) & 0xFF
                handled = True
                if handleKey(key, pnt=tag.pose_t):
                    print("Closing!")
                    break
                if ADJ_DISTANCE * distance_phase > distance and travelling:
                    print("Adjusting")
                    #stop drone
                    drone.send_rc_control(0,0,0,0)
                    # look at pose
                    detect_rotate(pnt=tag.pose_t)
                    # start
                    drone.send_rc_control(0,50,0,0)
                    handle_height(tag.pose_t)
                    distance_phase += 1
                if distance < 350 and travelling:
                    print("Stopping, arrived!")
                    #stop drone
                    drone.send_rc_control(0,0,0,0)
                    travelling = False
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

def handle_height(pnt):
    distance = pnt[1]
    if abs(distance) < 0.1: # 10 cm
        return
    else:
        if distance > 0:
            drone.move_up(distance)
        else:
            drone.move_down(-distance)

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
        if pnt is None:
            print("NO TAG!")
            return
        threading.Thread(target=travel, args=(pnt, )).start()
    if key == ord("x"):
        # STOP!!!
        drone.send_rc_control(0,0,0,0)

def detect_rotate(pnt):
    # look at
    if pnt is None:
        print("NO TAG!")
        return
    angle = look_at(pnt)
    if angle < 5:
        return
    if(angle > 0):
        # cw
        drone.rotate_clockwise(int(angle))
    else:
        # ccw
        drone.rotate_counter_clockwise(int(-angle))

def travel(pnt): 
    detect_rotate(pnt)
    global travelling
    if not travelling:
        travelling = True
        drone.send_rc_control(0,50,0,0)


if __name__ == "__main__":
    main()

cv2.destroyAllWindows()