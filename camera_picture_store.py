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

WINDOW_NAME = 'CameraDemo'
img_width = 0
img_height = 0

def open_window(width, height):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(WINDOW_NAME, 0, 0)
    cv2.setWindowTitle(WINDOW_NAME, 'Albatros camera view')


def read_cam(cap):
    show_help = True
    full_scrn = False
    help_text = '"Esc" to Quit, "H" for Help, "S" to store image'
    font = cv2.FONT_HERSHEY_PLAIN
    
    print('expected FPS: ', cap.get(cv2.CAP_PROP_FPS))
                       
    currtime = time.time()
    counter = 0
    img_counter = 1
    while True:
        counter += 1
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            # Check to see if the user has closed the window
            # If yes, terminate the program
            break
        if(time.time() - currtime > 1):
            print("FPS: ", counter)
            counter = 0
            currtime = time.time()

        ret, img_org = cap.read() # grab the next image frame from camera
        
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        img = cv2.resize(img_org, (480,270))
        if show_help:
            cv2.putText(img, help_text, (11, 20), font,
                        1.0, (32, 32, 32), 4, cv2.LINE_AA)
            cv2.putText(img, help_text, (10, 20), font,
                        1.0, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(1)
        if key == 27: # ESC key: quit program
            break
        if key == ord('H') or key == ord('h'): # toggle help message
            show_help = not show_help
        if key == ord('S') or key == ord('s'): # save
            img_path = 'imgs/'+ str(img_counter) + '.png'
            print("saving file: " + img_path)
            cv2.imwrite(img_path,img_org)
            img_counter += 1

        
def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    gst_str = ("v4l2src device=/dev/video{} ! "
               "video/x-raw, width=(int){}, height=(int){}, format=(string)RGB ! "
               "videoconvert ! appsink").format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def readCamera(dev):
    cap = cv2.VideoCapture(dev)
    fourcc_cap = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc_cap)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)     #horizontal pixels
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)     #vertical pixels
    cap.set(cv2.CAP_PROP_FPS, 30)      #FPS
    if not cap.isOpened():
        sys.exit('Failed to open camera!')
    img_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    img_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print("start reading at FPS ", cv2.CAP_PROP_FPS)
    print(img_width, ", ", img_height)
    open_window(int(img_width/4), int(img_height/4))
    try:
        read_cam(cap)
    except:
        cap.release()
        cv2.destroyAllWindows() 
        exit

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    readCamera("/dev/video0")
