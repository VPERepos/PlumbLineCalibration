import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from wand.image import Image

#n = np.array([(54.5, 17.041667, 31.993),
#                  (54.5, 17.083333, 31.911),
#                  (54.458333, 17.041667, 31.945),
#                  (54.458333, 17.083333, 31.866)])

#griddata(n[:,0:2], n[:,2], [(54.4786674627, 17.0470721369)], method='linear')

img = cv2.imread("PlumbLineRasterInitial.jpg")
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
rvec = np.zeros(3, dtype=float)
tvec = np.zeros(3, dtype=float)
cameraMatrix = np.zeros((3,3), dtype=float)
distortionCoefficients = np.zeros(5, dtype=float)

cameraMatrix[0][0]= 0.0
cameraMatrix[0][1]= 0.0
cameraMatrix[0][2]= 3358.0

cameraMatrix[1][0]= 0.0
cameraMatrix[1][1]= 0.0
cameraMatrix[1][2]= 2350.0

cameraMatrix[2][0]= 0.0
cameraMatrix[2][1]= 0.0
cameraMatrix[2][2]= 1.0

distortionCoefficients[0] = -0.48
distortionCoefficients[1] = 0.025
distortionCoefficients[2] = 0.01
distortionCoefficients[3] = 0.01
distortionCoefficients[4] = 0.05

with Image(filename='PlumbLineRasterInitial.jpg') as img:
    print(img.size)
    img.virtual_pixel = 'transparent'
    img.distort('barrel', (0.2, 0.0, 0.0, 1.0))
    img.save(filename='PlumbLineRasterDistorted.png')
    