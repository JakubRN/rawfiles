
import sys
import argparse
import cv2
import time

cap = cv2.VideoCapture(0)
 

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name", help="movie name")
args = parser.parse_args()
movieName=args.name
movieName += '.avi'
print(movieName)
# Check if camera opened successfully
if (cap.isOpened() == False): 
    print("Unable to read camera feed")
    exit

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
 
while(True):
  ret, frame = cap.read()
 
  if ret == True: 
     
    # Write the frame into the file 'output.avi'
    out.write(frame)
 
    # Display the resulting frame    
    cv2.imshow('frame',frame)
 
    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else:
    break 
 
# When everything done, release the video capture and video write objects
cap.release()
out.release()
 
# Closes all the frames
cv2.destroyAllWindows() 