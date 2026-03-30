# #!/usr/bin/env python
import cv2
import numpy as np
import math
point3s = np.array(([5.8, 1.22,0], [5.8, 0.0, 0], [5.8, -1.66, 0], [11.66, 1.16, 0], [11.60, 0.0, 0],
                   [11.60, -1.64, 0]), dtype=np.double)
point2s = np.array(([186,322], [327,327], [525, 332], [261,286], [327,287], [
                   424,291],), dtype=np.double)

k = np.array([[769.50412258, 0.000, 270.45676359], [
    0.000, 772.92576942, 219.68535813], [0.000, 0.000, 1.000]])
dist=np.array(([ 0.15657607 ,-0.41546746 ,-0.00207998 ,-0.0119601   ,0.47152757]),dtype=np.double)
found, r, t = cv2.solvePnP(point3s, point2s, k, dist)


R, j = cv2.Rodrigues(r)


d3 = np.array([[3.0, 0, 0]])
d2, _ = cv2.projectPoints(d3, r, t, k, dist)
print(r)
print(R)
print(t)
print(d2)

