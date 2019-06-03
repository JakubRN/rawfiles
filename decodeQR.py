
import sys
import argparse
import cv2
import time
from pyzbar.pyzbar import decode
WINDOW_NAME = "MOVIE"
 
def open_window(width, height):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(WINDOW_NAME, 0, 0)
    cv2.setWindowTitle(WINDOW_NAME, 'Camera Demo for Jetson TX2/TX1')

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--video", help="video camera input", default='dev/video0')
parser.add_argument("-w", "--width", help="camera width", default=3840, type=int)
parser.add_argument("-he", "--height", help="camera height", default=1920, type=int)
args = parser.parse_args()
videoName=args.video
# Check if camera opened successfully
cap = cv2.VideoCapture(videoName)
if (cap.isOpened() == False): 
    print("Failed, trying to open on default port")
    cap = cv2.VideoCapture(cv2.CAP_DSHOW)
    #cap = cv2.VideoCapture(0)
if (cap.isOpened() == False):     
    print("Unable to read camera feed")
    exit
cap.set(cv2.CAP_PROP_FRAME_WIDTH,args.width)     #horizontal pixels
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)     #vertical pixels

cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
cap.set(cv2.CAP_PROP_FPS,30)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(frame_height, ", ", frame_width)
open_window(min(int(frame_width/2),1920), min(int(frame_height/2), 1080))


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
      for barcode in decode(frame):
        rect = barcode.rect
        cv2.rectangle(frame, (rect.left, rect.top),(rect.left + rect.width,rect.top + rect.height ),(255,0,0))
      delay += 1
      # Write the frame into the file
      #out.write(frame)
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
  print("safely closing files")
  # Closes all the frames
  cv2.destroyAllWindows() 
  exit

cap.release()

# Closes all the frames
cv2.destroyAllWindows() 
