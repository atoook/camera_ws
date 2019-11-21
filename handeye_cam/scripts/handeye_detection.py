#!/usr/bin/env python
## coding: UTF-8

import rospy
import cv2
import glob
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

####use custormized msg
from std_msgs.msg import Int32MultiArray

#------GUI class to create trackbar class
class GUI():
    def __init__(self,img_name,type_name,param_name_list,max_param,MIN,MAX):
        self.img_name = str(img_name)
        self.type_name = str(type_name)
        self.argnum = len(param_name_list)
        self.param_name_list = param_name_list
        self.max_param = max_param
        self.min = MIN
        self.max = MAX
    
    def get_param_as_tuple(self):
        return tuple(self.min),tuple(self.max)
    
    def changeColor(self,val):
        for i in range(self.argnum):
            self.min[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_min', self.img_name))
            self.max[i] = int(cv2.getTrackbarPos(self.type_name + self.param_name_list[i] + '_max', self.img_name))
        self.save_track_parameter()

    def create_trackbar(self):
        background = np.zeros((10,300,3), np.uint8)
        cv2.namedWindow(self.img_name)
        #create trackbar
        for i in range(self.argnum):
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_min', self.img_name, self.min[i], self.max_param[i], self.changeColor)
            cv2.createTrackbar(self.type_name + self.param_name_list[i] + '_max', self.img_name, self.max[i], self.max_param[i], self.changeColor)
        cv2.imshow(self.img_name,background)
    
    def save_track_parameter(self):
        np.save('./' + self.img_name + self.type_name + 'min.npy',self.min)
        np.save('./' + self.img_name + self.type_name + 'max.npy',self.max)

class ColorExtract(object):
    def __init__(self,image_width,image_height):
        self._msg_pub = rospy.Publisher('/usb_cam/handeye_msg', Int32MultiArray, queue_size=10)
        # self._red_pub = rospy.Publisher('/red_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()
        self._msg = Int32MultiArray()
        self._image_width = image_width
        self._image_height = image_height
        img_name = 'Handeye_setting'
        type_name1 = 'mask1--'
        type_name2 = 'mask2--'
        if(len(glob.glob('./'+img_name+ '*.npy')) == 4):
            self.Lmask_MIN = np.load('./' + img_name + type_name1 + 'min.npy')
            self.Lmask_MAX = np.load('./' + img_name + type_name1 + 'max.npy')
            self.Umask_MIN = np.load('./' + img_name + type_name2 + 'min.npy')
            self.Umask_MAX = np.load('./' + img_name + type_name2 + 'max.npy')
        else:
            self.Lmask_MIN = np.array([0,101,144])
            self.Lmask_MAX = np.array([7,255,240])
            self.Umask_MIN = np.array([170,160,104])
            self.Umask_MAX = np.array([180,255,206])
        #make instance to create trackbar
        self.trackbar_mask1 = GUI(img_name,type_name1,['H','S','V'],np.array([180,255,255]),self.Lmask_MIN,self.Lmask_MAX)
        self.trackbar_mask2 = GUI(img_name,type_name2,['H','S','V'],np.array([180,255,255]),self.Umask_MIN,self.Umask_MAX)
        self.trackbar_mask1.create_trackbar()
        self.trackbar_mask2.create_trackbar()

    def get_colored_area(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image1 = cv2.inRange(hsv_image, self.Lmask_MIN, self.Lmask_MAX)
        mask_image2 = cv2.inRange(hsv_image, self.Umask_MIN, self.Umask_MAX)
        ## Merge the mask and extract the red regions
        redmask = cv2.bitwise_or(mask_image1, mask_image2 )
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6,6))
        redmask = cv2.erode(redmask, kernel, iterations = 1)
        redmask = cv2.dilate(redmask, kernel, iterations = 1)
        filtered_extracted_imag = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
        area = cv2.countNonZero(redmask)

        return (area, filtered_extracted_imag)

    def callback(self, data):
        try:
            ##get mask param from track bar
            self.Lmask_MIN, self.Lmask_MAX = self.trackbar_mask1.get_param_as_tuple()
            self.Umask_MIN, self.Umask_MAX = self.trackbar_mask2.get_param_as_tuple()

            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e
        
        #get the result val and image obj
        red_area, red_image = self.get_colored_area(cv_image)
    
        box_index = np.where(red_image != 0)

        if len(box_index[0]) >= 10 : # check red exists

            left = np.amin(box_index[1])
            right = np.amax(box_index[1])
            top = np.amin(box_index[0])
            bottom = np.amax(box_index[0])

            i_centor = int(left + right) / 2
            j_centor = int(top + bottom) / 2

            #publish msg
            self._msg.data = [i_centor,j_centor,red_area]
            self._msg_pub.publish(self._msg)   
            print(self._msg.data)

            cv2.putText(red_image, str(red_area), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
            cv2.putText(red_image, '(' + str(i_centor - self._image_width/2)+','+str(j_centor - self._image_height/2) + ')', (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
            cv2.circle(red_image, (i_centor,j_centor),10,(0,0,255),-1)

        ##show the image of handeye
        cv2.namedWindow('handeye_extracted', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('handeye_extracted', red_image)
        rospy.loginfo('red=%d' % (red_area))

        cv2.waitKey(1)
   
if __name__ == '__main__':
    #define the initial HSV param for mask1&2
    rospy.loginfo('handeye_detection is started.....')
    #make instance to create trackbar
    rospy.init_node('color_extract')
    #############
    image_width  = int(rospy.get_param('~image_width', '640'))
    image_height = int(rospy.get_param('~image_height','480'))

    color = ColorExtract(image_width,image_height)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        pass