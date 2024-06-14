import cv2

cap = cv2.VideoCapture(0) 
ret,frame = cap.read()
cv2.imwrite('/data/0.jpg',frame)
image = cv2.imread('/data/0.jpg')

cv2.imshow("hi",image)
cv2.waitKey(0)