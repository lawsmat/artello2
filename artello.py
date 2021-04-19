# Artello Library

import cv2, djitellopy, pupil_apriltags, json, math, threading
import numpy as np

class Artello:
    def __init__(drone=None, filter=None):
        if not drone:
            raise Exception("Drone not provided, please initialize with a DJITellopy2 Tello object!")
            return
        self.travelling = False
        self.drone = drone
    
    def search():
        # TODO: Spin until finds code

    def look_at(point):
        x = point[0] # X in 3d space
        y = point[2] # Z in 3d space
        # we don't care about the y axis yet

        rotx = math.atan2(x, y) * 180/math.pi

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

    def rotate_to_look_at(pnt):
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
        if not self.travelling:
            self.travelling = True
            drone.send_rc_control(0,50,0,0)