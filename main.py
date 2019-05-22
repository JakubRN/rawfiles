import readtr1w
import threading
import cameraReader

tr1wDataThread = threading.Thread(target = readtr1w.readTransponder, args = ('/dev/ttyUSB0',))
tr1wDataThread.start()


cameraThread = threading.Thread(target = cameraReader.readCamera, args = ('/dev/video0',))
cameraThread.start()

print(readtr1w.airplanes)
