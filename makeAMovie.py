
import sys
import argparse
import cv2
import time

WINDOW_NAME = "MOVIE"
 
def open_window(width, height):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(WINDOW_NAME, 0, 0)
    cv2.setWindowTitle(WINDOW_NAME, 'Camera Demo for Jetson TX2/TX1')


def open_cam_usb(dev, width, height):
    # We want to set width and height here, otherwise we could just do:
    #     return cv2.VideoCapture(dev)
    gst_str = ("v4l2src device=/dev/video{} ! "
               "video/x-raw, width=(int){}, height=(int){}, format=(string)RGB ! "
               "videoconvert ! appsink").format(dev, width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


def _gst_str():
        return 
   
capture_width = 3280
capture_height = 2464
frame_width = 3280
frame_height = 2464
fps = 21
parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name", help="movie name", required=True)
parser.add_argument("-v", "--video", help="video camera input", default=0, type=int)
parser.add_argument("-w", "--width", help="camera width", default=3840, type=int)
parser.add_argument("-he", "--height", help="camera height", default=1920, type=int)
args = parser.parse_args()
videoName=args.video
movieName=args.name
movieName += '.avi'
# Check if camera opened successfully
cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                capture_width, capture_height, fps, frame_width, frame_height), cv2.CAP_GSTREAMER)
if (cap.isOpened() == False):     
    print("Unable to read camera feed")
    exit
    
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(frame_height, ", ", frame_width)
open_window(min(int(frame_width/2),1920), min(int(frame_height/2), 1080))

out = cv2.VideoWriter(movieName,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
 
print("start reading at FPS ", cv2.CAP_PROP_FPS)
delay = 0
try:
  while(True):
    # while (True):
    #     ticks = time.time()
    #     cap.grab()
    #     #print((time.time()-ticks)*cap.get(cv2.CAP_PROP_FPS))
    #     if((time.time() - ticks) * cap.get(cv2.CAP_PROP_FPS) > 0.5): 
    #     this crashes the program if opencv doesnt get the right framerate
    #         break
    ret, frame = cap.read()
    
    if ret == True: 
      delay += 1
      # Write the frame into the file
      out.write(frame)
      cv2.imshow(WINDOW_NAME,frame)
      # if(delay == 10000):
      #     break

      # Display the resulting frame    

      
# Press Q on keyboard to stop recording
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
    # Break the loop
    else:
      break 
except:
  # When everything done, release the video capture and video write objects
  cap.release()
  out.release()
  print("safely closing files")
  # Closes all the frames
  cv2.destroyAllWindows() 
  exit

cap.release()
out.release()

# Closes all the frames
cv2.destroyAllWindows() 
