# --------------------------------------------------------
# Camera sample code for Tegra X2/X1
#
# This program could capture and display video from
# IP CAM, USB webcam, or the Tegra onboard camera.
# Refer to the following blog post for how to set up
# and run the code:
#   https://jkjung-avt.github.io/tx2-camera-with-python/
#
# Written by JK Jung <jkjung13@gmail.com>
# --------------------------------------------------------


import sys
import argparse
import cv2
import time
from pyzbar.pyzbar import decode
from pydarknet import Detector, Image

WINDOW_NAME = 'CameraDemo'


def open_window(width, height):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(WINDOW_NAME, 0, 0)
    cv2.setWindowTitle(WINDOW_NAME, 'Camera Demo for Jetson TX2/TX1')


def read_cam(cap):
    show_help = True
    full_scrn = False
    help_text = '"Esc" to Quit, "H" for Help, "F" to Toggle Fullscreen'
    font = cv2.FONT_HERSHEY_PLAIN
    usedarknet = False
    
    print('expected FPS: ', cap.get(cv2.CAP_PROP_FPS))
    #pydarknet.set_cuda_device(0)
    net = Detector(bytes("yoloBasic/cfg/yolov3.cfg", encoding="utf-8"), bytes("yoloBasic/weights/yolov3.weights", encoding="utf-8"), 0,
                       bytes("yoloBasic/cfg/coco.data", encoding="utf-8"))
                       
    currtime = time.time()
    counter = 0
    while True:
        counter += 1
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            # Check to see if the user has closed the window
            # If yes, terminate the program
            break
        if(time.time() - currtime > 1):
            print("actual FPS: ", counter)
            counter = 0
            currtime = time.time()
        
        while (True):
            ticks = time.time()
            cap.grab()
            #print((time.time()-ticks)*cap.get(cv2.CAP_PROP_FPS))
            if((time.time() - ticks) * cap.get(cv2.CAP_PROP_FPS) > 0.5): 
                break
                
        _, img = cap.read() # grab the next image frame from camera
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        text = "Not Blurry"
        bluriness = cv2.Laplacian(gray, cv2.CV_64F).var() 
        if(bluriness< 100):
            text = "Blurry"
        else:
            if(usedarknet):
                start_time = time.time()
                dark_frame = Image(img)
                results = net.detect(dark_frame)
                del dark_frame

                end_time = time.time()
                print("Elapsed Time:",end_time-start_time)

                for cat, score, bounds in results:
                    x, y, w, h = bounds
                    cv2.rectangle(img, (int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(255,0,0))
                    cv2.putText(img, str(cat.decode("utf-8")), (int(x), int(y)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 0))

## barcode detection
        for barcode in decode(img):
            rect = barcode.rect
            cv2.rectangle(img, (rect.left, rect.top),(rect.left + rect.width,rect.top + rect.height ),(255,0,0))


        cv2.putText(img, "{}: {:.2f}".format(text, bluriness), (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        if show_help:
            cv2.putText(img, help_text, (11, 20), font,
                        1.0, (32, 32, 32), 4, cv2.LINE_AA)
            cv2.putText(img, help_text, (10, 20), font,
                        1.0, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(10)
        if key == 27: # ESC key: quit program
            break
        elif key == ord('H') or key == ord('h'): # toggle help message
            show_help = not show_help
        elif key == ord('F') or key == ord('f'): # toggle fullscreen
            full_scrn = not full_scrn
            if full_scrn:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_NORMAL)

        
def readCamera(dev):
    cap = cv2.VideoCapture(dev)
    cap.set(3,1920)     #horizontal pixels
    cap.set(4,1080)     #vertical pixels
    cap.set(5, 5)      #FPS
    if not cap.isOpened():
        sys.exit('Failed to open camera!')
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(w, ", ", h)
    open_window(int(w), int(h))
    read_cam(cap)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
