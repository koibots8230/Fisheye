import cv2 as cv
import numpy as np
from Constants import *
import tracemalloc

tracemalloc.start()

capture = cv.VideoCapture(0)

if ENABLE_VIDEO_SAVING:
    fourcc = cv.VideoWriter_fourcc("M", "J", "P", "G")
    out = cv.VideoWriter('output.avi',fourcc, 15, PROCESSING_SIZE, isColor = False)
 
if (capture.isOpened()== False): 
  print("Error opening video stream or file")
 
while(capture.isOpened()):
  ret, frame = capture.read()
  if ret == True:
    
    grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    resize = cv.resize(grey, PROCESSING_SIZE)
    
    threshold = cv.adaptiveThreshold(
        resize,
        255,
        cv.ADAPTIVE_THRESH_MEAN_C,
        cv.THRESH_BINARY,
        THRESHOLD_BLOCK_SIZE,
        2)
    
    sharpenKernel = np.array([
        [0, -1, 0],
        [-1, 5, -1],
        [0, -1, 0]])

    sharpened = cv.filter2D(threshold, ddepth = -1, kernel = sharpenKernel)

    contours, hiarchy = cv.findContours(sharpened, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    filtered = cv.drawContours(resize, contours, -1, (0, 255, 0), 1)
    
    cv.imshow('Frame', resize)

    if ENABLE_VIDEO_SAVING:
        out.write(filtered)
 
    if cv.waitKey(25) & 0xFF == ord('q'):
      break
  else: 
    break

print(tracemalloc.get_traced_memory())

print(contours)

capture.release()

tracemalloc.stop()

if ENABLE_VIDEO_SAVING:
    out.release()
 
cv.destroyAllWindows()