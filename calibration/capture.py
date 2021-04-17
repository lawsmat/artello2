import cv2

cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop) 

index = 0

while(True):
    ret, frame = cap.read() # return a single frame in variable `frame`
    cv2.imshow('Press Y',frame) #display the captured image
    if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y'
        index = index + 1 
        cv2.imwrite('calib-'+ str(index) +'.png',frame)
        if(index == 11):
            cv2.destroyAllWindows()
            break

cap.release()