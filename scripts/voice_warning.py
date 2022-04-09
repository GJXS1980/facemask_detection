#!/usr/bin/env python
# -*- coding:utf-8 -*-

import time
import cv2
import rospy
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Image, RegionOfInterest

from playsound import playsound


class NoFacemask:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.init_node("no_facemask", anonymous=True)
        self.voice = rospy.get_param("~failed_file_path", "/params/voice/NoMask.wav") # 没有佩戴口罩
        self.voice1 = rospy.get_param("~file_path_a", "/params/voice/nor_temp.wav") # 体温正常
        self.voice2 = rospy.get_param("~file_path_b", "/params/voice/abnor_temp.wav")# 体温过高
        self.voice3 = rospy.get_param("~file_path_c", "/params/voice/Mask.wav") # 佩戴口罩
        self.class_id, self.mask_class_id = 10, 0
        self.temp_state = 0
        self.count = 0
        rospy.Subscriber("/Temperature_measurement_topic", Float32, self.temp_data)	# 体温
        rospy.Subscriber("NoMask", Int32, self.nomask_flag, queue_size=1) # data:1
        rospy.Subscriber("Mask", Int32, self.mask_flag, queue_size=1)	# data:1
        rospy.Subscriber("DetectDone", Float32, self.voice_warning, queue_size=1)
        self.maskdata_pub = rospy.Publisher('mask_data', Int32, queue_size=1) 
        rospy.spin()


    def temp_data(self, data):
        if data.data > 37 or data.data < 35:
            self.temp_state = 1
        elif data.data > 35 and data.data < 37 :
             self.temp_state = 0      
        self.count +=1

    def nomask_flag(self, data):
        self.class_id = data.data

    def mask_flag(self, data):
        self.mask_class_id = data.data

    def voice_warning(self, data):
        # 没有佩戴口罩
        if self.class_id == 1:    
            if self.count%3 == 0 : 
                rospy.sleep(1)
                # 体温过高
                if self.temp_state == 1:
                    playsound(self.voice2)
                # 体温正常
                elif self.temp_state == 0 :
                    playsound(self.voice1)
                self.count = 0 
               
            self.maskdata_pub.publish(0)
            # 没有佩戴口罩提醒
            playsound(self.voice)
            self.class_id = 10
        else:
            self.class_id = 10
            #playsound(self.voice)
            if data.data > 0.7:
                self.maskdata_pub.publish(1)
                if self.count%3 == 0 : 
                    rospy.sleep(1)
                    if self.temp_state == 1:
                        playsound(self.voice2)
                    elif self.temp_state == 0 :
                        playsound(self.voice1)
                    self.count = 0 
            else:
                 self.maskdata_pub.publish(2)

            #	佩戴口罩
            if self.mask_class_id:
                 playsound(self.voice3)  
                 self.mask_class_id = 0 

        self.temp_state = 2

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        NoFacemask()
	#rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detect test finished.")





