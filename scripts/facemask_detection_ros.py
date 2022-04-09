#!/usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import time
import math as m
import numpy as np

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from object_detect.msg import object_result_msg
from std_msgs.msg import Float32, Int32
from std_msgs.msg import Int32MultiArray
from PIL import Image as Im

import argparse
from utils.anchor_generator import generate_anchors
from utils.anchor_decode import decode_bbox
from utils.nms import single_class_non_max_suppression
from load_model.pytorch_loader import load_pytorch_model, pytorch_inference
from playsound import playsound


class Facemask:
    def __init__(self):
    	self.start_stamp = time.time()
        #rospy.on_shutdown(self.cleanup);
        self.model = load_pytorch_model('/home/gjxs/catkin_ws/src/facemask_detection/models/model360.pth')
    	rospy.init_node("facemask_detection", anonymous=True)
        self.voice = rospy.get_param("~failed_file_path", "/params/voice/failed.mp3")

    	# rate = rospy.Rate(10) # 发布频率为10hz
    	self.feature_map_sizes = [[45, 45], [23, 23], [12, 12], [6, 6], [4, 4]]
    	self.anchor_sizes = [[0.04, 0.056], [0.08, 0.11], [0.16, 0.22], [0.32, 0.45], [0.64, 0.72]]
    	self.anchor_ratios = [[1, 0.62, 0.42]] * 5
    	self.anchors = generate_anchors(self.feature_map_sizes, self.anchor_sizes, self.anchor_ratios)
    	self.anchors_exp = np.expand_dims(self.anchors, axis=0)
    	self.id2class = {0: 'Mask', 1: 'NoMask'}
    	self.color = (255, 255, 255)
        self.class_id = 10
        self.bridge = CvBridge()

        self.pub = rospy.Publisher('DetectDone', Float32, queue_size=10)   
        self.nomask_pub = rospy.Publisher('NoMask', Int32, queue_size=10)   
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
    	
    	while not rospy.is_shutdown():
    		self.main_fun()
    		rospy.spin()
    	self.shutdown()

    def main_fun(self):
        rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # start_stamp = time.time()
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            self.img = np.array(self.cv_image, dtype=np.uint8)

        except CvBridgeError, e:
            print e

        # 将图像从RGB转成灰度图
         # = frame

        img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        # read_frame_stamp = time.time()

        self.inference(img, show_result=True, target_shape=(360, 360))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))

    def inference(self, image,
              	conf_thresh=0.5,
              	iou_thresh=0.4,
              	target_shape=(160, 160),
              	draw_result=True,
              	show_result=True
              	):
        start_stamp = time.time()
    	output_info = []
    	height, width, _ = image.shape
    	image_resized = cv2.resize(image, target_shape)
    	image_np = image_resized / 255.0  # 归一化到0~1
    	image_exp = np.expand_dims(image_np, axis=0)

    	image_transposed = image_exp.transpose((0, 3, 1, 2))

    	y_bboxes_output, y_cls_output = pytorch_inference(self.model, image_transposed)
    	# remove the batch dimension, for batch is always 1 for inference.
    	y_bboxes = decode_bbox(self.anchors_exp, y_bboxes_output)[0]
    	y_cls = y_cls_output[0]
    	# To speed up, do single class NMS, not multiple classes NMS.
    	bbox_max_scores = np.max(y_cls, axis=1)
    	bbox_max_score_classes = np.argmax(y_cls, axis=1)

    	# keep_idx is the alive bounding box after nms.
    	keep_idxs = single_class_non_max_suppression(y_bboxes,
                                                 	bbox_max_scores,
                                                 	conf_thresh=conf_thresh,
                                                 	iou_thresh=iou_thresh,
                                                 	)
    	for idx in keep_idxs:
        	conf = float(bbox_max_scores[idx])
        	self.class_id = bbox_max_score_classes[idx]
        	bbox = y_bboxes[idx]
        	# clip the coordinate, avoid the value exceed the image boundary.
        	xmin = max(0, int(bbox[0] * width))
        	ymin = max(0, int(bbox[1] * height))
        	xmax = min(int(bbox[2] * width), width)
        	ymax = min(int(bbox[3] * height), height)

        	if draw_result:
        		if self.class_id == 0:
        			self.color = (0, 255, 0)
            	elif self.class_id == 1:
                	self.color = (255, 0, 0)
                else:
                    pass
            	cv2.rectangle(self.img, (xmin, ymin), (xmax, ymax), self.color, 2)
            	self.img = cv2.putText(self.img, "%s: %.2f" % (self.id2class[self.class_id], conf), (xmin + 2, ymin - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color)
        	output_info.append([self.class_id, conf, xmin, ymin, xmax, ymax])

    	if show_result:
    	# 	# cv2.imshow('image', self.img[:, :, ::-1])
     #        #   发布图像话题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
            if self.class_id == 1:
                self.nomask_pub.publish(self.class_id)
                # playsound(self.voice)
                self.class_id = 10
            else:
                pass
            
        self.read_frame_stamp = time.time()
        self.pub.publish(self.read_frame_stamp - self.start_stamp)
     #    	Im.fromarray(image).show()
         #    #   当识别到没有戴口罩的时候，发布一个flag
         #    if self.class_id == 1:
         #        

    	return output_info




    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        Facemask()
	#rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detect test finished.")





