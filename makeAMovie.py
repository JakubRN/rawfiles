
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

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name", help="movie name", required=True)
parser.add_argument("-v", "--video", help="video camera input", default='dev/video0')
args = parser.parse_args()
videoName=args.video
movieName=args.name
movieName += '.avi'
# Check if camera opened successfully
cap = cv2.VideoCapture(videoName)
if (cap.isOpened() == False): 
    print("Failed, trying to open on default port")
    cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)     #horizontal pixels
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)     #vertical pixels
if (cap.isOpened() == False):     
    print("Unable to read camera feed")
    exit

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
# w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# print(w, ", ", h)
# open_window(int(w), int(h))

out = cv2.VideoWriter(movieName,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
 
print("start reading")
delay = 0
try:
  while(True):
    ret, frame = cap.read()
    
    if ret == True: 
      delay += 1
      # Write the frame into the file 'output.avi'
      out.write(frame)
      if(delay == 10000):
          break
      # Display the resulting frame    
      cv2.imshow('frame',frame)
      
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
