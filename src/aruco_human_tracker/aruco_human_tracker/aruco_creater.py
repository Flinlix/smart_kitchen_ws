import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

marker_id = 4
px_size = 600  # pixels

img = cv2.aruco.drawMarker(dictionary, marker_id, px_size)
cv2.imwrite(f"aruco_{marker_id}.png", img)

print(f"Saved aruco_{marker_id}.png")