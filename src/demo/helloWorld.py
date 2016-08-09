import cv2

cam = cv2.VideoCapture(0)
running = True

while running:
    running, frame = cam.read()
    if running:
        # draw rectangle
        cv2.rectangle(frame,(100, 100), (200, 200), (255,0,0), 2)
        # write text
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame,'Press Escape to exit...',(10,50), font, 1,(255,255,255),2)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0XFF == 27: #Escape key pressed
            running = False
    else:
        print '[Error] reading video feed.'

cam.release()
cv2.destroyAllWindows()
