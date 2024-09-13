import cv2
import numpy as np
import dlib
import networkx as nx
import matplotlib.pyplot as plt
import math


# function for finding distance betwean 2 nodes in graph
def calc_distance(x1, y1, x2, y2):
    x_dist = (x2 - x1)
    y_dist = (y2 - y1)
    return math.sqrt(x_dist * x_dist + y_dist * y_dist)



# Load the detector

detector = dlib.get_frontal_face_detector()



# Load the predictor
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")



# read the image
img = cv2.imread("photo/mine.png")



# Convert image into grayscale
gray = cv2.cvtColor(src=img, code=cv2.COLOR_BGR2GRAY)



# Use detector to find landmarks
faces = detector(gray)


for face in faces:
    keyPoints = predictor(image=gray, box=face)
    # Loop through all the points
    for kp in range(0, 68):
        x = keyPoints.part(kp).x
        y = keyPoints.part(kp).y
        # Draw a circle
        cv2.circle(img=img, center=(x, y), radius=3, color=	(0, 400, 455), thickness=-1)



# show the image
cv2.imshow(winname="Face", mat=img)


# Delay between every fram
cv2.waitKey(delay=0)


# Close all windows
cv2.destroyAllWindows()

# creating array for x and y 's
arrx = []
arry = []
# appending x & y into array
for n in range(0, 68):
    x = keyPoints.part(n).x
    y = keyPoints.part(n).y
    y = y*(-1)
    arrx.append(x)
    arry.append(y)


# positions of nodes
pos = {0: (arrx[0], arry[0]), 1: (arrx[1], arry[1]), 2: (arrx[2], arry[2]), 3: (arrx[3], arry[3]), 4: (arrx[4], arry[4]), 5: (arrx[5], arry[5]), 6: (arrx[6], arry[6]), 7: (arrx[7], arry[7]), 8: (arrx[8], arry[8]), 9: (arrx[9], arry[9]), 10: (arrx[10], arry[10]), 11: (arrx[11], arry[11]), 12: (arrx[12], arry[12]), 13: (arrx[13], arry[13]), 14: (arrx[14], arry[14]), 15: (arrx[15], arry[15]), 16: (arrx[16], arry[16]), 17: (arrx[17], arry[17]), 18: (arrx[18], arry[18]), 19: (arrx[19], arry[19]), 20: (arrx[20], arry[20]), 21: (arrx[21], arry[21]),
       22: (arrx[22], arry[22]), 23: (arrx[23], arry[23]), 24: (arrx[24], arry[24]), 25: (arrx[25], arry[25]), 26: (arrx[26], arry[26]), 27: (arrx[27], arry[27]), 28: (arrx[28], arry[28]), 29: (arrx[29], arry[29]), 30: (arrx[30], arry[30]), 31: (arrx[31], arry[31]), 32: (arrx[32], arry[32]),
       33: (arrx[33], arry[33]), 34: (arrx[34], arry[34]), 35: (arrx[35], arry[35]), 36: (arrx[36], arry[36]), 37: (arrx[37], arry[37]), 38: (arrx[38], arry[38]), 39: (arrx[39], arry[39]), 40: (arrx[40], arry[40]),
       41: (arrx[41], arry[41]), 42: (arrx[42], arry[42]), 43: (arrx[43], arry[43]), 44: (arrx[44], arry[44]), 45: (arrx[45], arry[45]), 46: (arrx[46], arry[46]), 47: (arrx[47], arry[47]), 48: (arrx[48], arry[48]),
       49: (arrx[49], arry[49]), 50: (arrx[50], arry[50]), 51: (arrx[51], arry[51]), 52: (arrx[52], arry[52]), 53: (arrx[53], arry[53]), 54: (arrx[54], arry[54]), 55: (arrx[55], arry[55]), 56: (arrx[56], arry[56]),
       57: (arrx[57], arry[57]), 58: (arrx[58], arry[58]), 59: (arrx[59], arry[59]), 60: (arrx[60], arry[60]), 61: (arrx[61], arry[61]), 62: (arrx[62], arry[62]), 63: (arrx[63], arry[63]), 64: (arrx[64], arry[64]),
       65: (arrx[65], arry[65]), 66: (arrx[66], arry[66]), 67: (arrx[67], arry[67]) }


# making graph
X = nx.Graph()
nx.draw_networkx_nodes(X, pos, node_size=10, nodelist=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
                                                        39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
                                                        51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
                                                        63, 64, 65, 66, 67 ])

# adding nodes to graph
X.add_nodes_from(pos.keys())



# making edges
for nodeA in range(0, 68):
    for nodeB in range(0, 68):
        if nodeA != nodeB:
            if calc_distance(arrx[nodeA], arry[nodeA], arrx[nodeB], arry[nodeB]) < 51:
                X.add_edges_from([(nodeA, nodeB)])



nx.draw(X, pos , node_color= '#e64105')
plt.show()
# print(X.number_of_edges())
