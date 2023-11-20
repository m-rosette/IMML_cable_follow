#!/usr/bin/env python2

#ROS imports
import rospy
from cv_bridge import CvBridge, CvBridgeError
from trial_control.srv import PictureTrigger, PictureTriggerResponse
from sensor_msgs.msg import Image

#image processing imports
import cv2
import numpy as np
from skimage.morphology import skeletonize, medial_axis, thin, binary_closing



class TakePicture:
    def __init__(self):
        #service
        self.picture_service = rospy.Service('picture_trigger', PictureTrigger, self.trigger_callback)
        #publishers
        self.mask_pub = rospy.Publisher("mask_image_topic", Image, queue_size=1)
        self.skeleton_pub = rospy.Publisher("skeleton_image_topic", Image, queue_size=1)
        self.original_pub = rospy.Publisher("original_image_topic", Image, queue_size=1)
        self.bridge = CvBridge()

    def trigger_callback(self, msg):
        #capture frame and save it under trial number
        cap = cv2.VideoCapture(2) 
        ret,frame = cap.read()
        cv2.imwrite('images/' + str(msg.trial_num) + '.jpg',frame)
        cap.release()

        #get mask and skeleton estimate
        skeleton, mask = self.process(frame)
        try:
            self.skeleton_image_message = self.bridge.cv2_to_imgmsg(skeleton, "passthrough")
            self.mask_image_message = self.bridge.cv2_to_imgmsg(mask, "passthrough")
            self.original_image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        #publish images
        self.skeleton_pub.publish(self.skeleton_image_message)
        self.mask_pub.publish(self.mask_image_message)
        self.original_pub.publish(self.original_image_message)
        return PictureTriggerResponse()
    
    def process(self, im):
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('img', 800, 600)
        #convert color to gray
        imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        #blur image
        blur = cv2.GaussianBlur(imgray, (25, 25), sigmaX=22)
        ret, thresh = cv2.threshold(blur, 170, 255, 0)
        #find contours
        j, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #get largest contour
        max_area = 0
        best_cnt = None
        for counter in contours:
            area = cv2.contourArea(counter)
            if area > 50:
                if area > max_area:
                    max_area = area
                    best_cnt = counter

        #create mask of image and draw the largest contour
        mask = np.zeros((imgray.shape), np.uint8)
        cv2.drawContours(mask, [best_cnt], 0, 255, -1)
        cv2.drawContours(mask, [best_cnt], 0, 0, 2)
        
        #dilate to get smoother contour
        kernel = np.ones((10, 10), np.uint8)
        img_dilation = cv2.dilate(mask, kernel, iterations=10)

        #skeletonize contour to get wire out
        img_dilation[img_dilation == 255] = 1
        skeleton = binary_closing(skeletonize(img_dilation*1)).astype("uint8")


        #return images
        skeleton[skeleton==1] = 255
        return skeleton, mask


        

if __name__ == '__main__':
    cable_follow = TakePicture()
    rospy.init_node('cable_follow', anonymous=True)
    rospy.spin()