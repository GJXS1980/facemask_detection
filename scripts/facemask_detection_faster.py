#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import cv2
import time
import math as m
import numpy as np

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
# from object_detect.msg import object_result_msg
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
		self.model = load_pytorch_model('/opt/ros/melodic/share/facemask_detection/models/model360.pth')
		rospy.init_node("facemask_detection", anonymous=True)
		self.feature_map_sizes = [[45, 45], [23, 23], [12, 12], [6, 6], [4, 4]]
		self.anchor_sizes = [[0.04, 0.056], [0.08, 0.11], [0.16, 0.22], [0.32, 0.45], [0.64, 0.72]]
		self.anchor_ratios = [[1, 0.62, 0.42]] * 5
		self.anchors = generate_anchors(self.feature_map_sizes, self.anchor_sizes, self.anchor_ratios)
		self.anchors_exp = np.expand_dims(self.anchors, axis=0)
		self.id2class = {0: 'Mask', 1: 'NoMask'}
		self.color = (255, 255, 255)
		self.class_id = 10
		self.bridge = CvBridge()
		self.temp = 0.0
		self.pub = rospy.Publisher('DetectDone', Float32, queue_size=10)   
		self.nomask_pub = rospy.Publisher('NoMask', Int32, queue_size=10) 
		self.mask_pub = rospy.Publisher('Mask', Int32, queue_size=10)   
		#self.maskdata_pub = rospy.Publisher('mask_data', Int32, queue_size=1) 
		self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
		self.opencv_cam =int(rospy.get_param("~opencv_cam", 0))
		self.cap = cv2.VideoCapture(self.opencv_cam)
		self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		self.total_frames = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
		if not self.cap.isOpened():
			raise ValueError("Video open failed.")
			return
		status = True
		idx = 0
		while not rospy.is_shutdown() and status:
			start_stamp = time.time()
			status, img_raw = self.cap.read()
			img_raw = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
			read_frame_stamp = time.time()
			if (status):
				self.inference(img_raw,
				conf_thresh = 0.5,
				iou_thresh=0.5,
				target_shape=(360, 360),
				draw_result=True,
				show_result=True)
				#cv2.imshow('image', img_raw[:, :, ::-1])
				#cv2.waitKey(1)
				inference_stamp = time.time()
				write_frame_stamp = time.time()
				idx += 1
				rospy.Subscriber("Temperature", Float32, self.temp_data, queue_size=1)
		self.shutdown()

	def temp_data(self, data):
		self.temp = data.data
		
	def inference(self, image,
								conf_thresh=0.5,
              					iou_thresh=0.4,
              					target_shape=(160, 160),
              					draw_result=True,
              					show_result=True
              				):
		conf = 0
		output_info = []
		height, width, _ = image.shape
		image_resized = cv2.resize(image, target_shape)
		image_np = image_resized / 255.0  # 归一化到0~1
		image_exp = np.expand_dims(image_np, axis=0)
		image_transposed = image_exp.transpose((0, 3, 1, 2))
		y_bboxes_output, y_cls_output = pytorch_inference(self.model, image_transposed)
		y_bboxes = decode_bbox(self.anchors_exp, y_bboxes_output)[0]
		y_cls = y_cls_output[0]
		bbox_max_scores = np.max(y_cls, axis=1)
		bbox_max_score_classes = np.argmax(y_cls, axis=1)
		keep_idxs = single_class_non_max_suppression(y_bboxes, bbox_max_scores, conf_thresh=conf_thresh, iou_thresh=iou_thresh,)
		for idx in keep_idxs:
			conf = float(bbox_max_scores[idx])
			self.class_id = bbox_max_score_classes[idx]
			bbox = y_bboxes[idx]
			xmin = max(0, int(bbox[0] * width))
			ymin = max(0, int(bbox[1] * height))
			xmax = min(int(bbox[2] * width), width)
			ymax = min(int(bbox[3] * height), height)
			if draw_result:
				if self.class_id == 0:
					self.color = (0, 255, 0)
					self.mask_pub.publish(1)
				elif self.class_id == 1:
					self.color = (255, 0, 0)
				else:
					pass
				cv2.rectangle(image, (xmin, ymin), (xmax, ymax), self.color, 2)
				image = cv2.putText(image, "%s: %.2f"  % (self.id2class[self.class_id], conf), (xmin + 2, ymin - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color)
				output_info.append([self.class_id, conf, xmin, ymin, xmax, ymax])

		if show_result:
			if self.class_id == 1 and conf >= 0.99:
				self.nomask_pub.publish(self.class_id)
				self.class_id = 10
			else:
				self.class_id = 10
				self.nomask_pub.publish(self.class_id)
		self.read_frame_stamp = time.time()
		#self.pub.publish(self.read_frame_stamp - self.start_stamp)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))
		self.pub.publish(conf)
		return output_info

	def cleanup(self):
		print("Shutting down vision node.")
		cv2.destroyAllWindows()

if __name__ == "__main__":
	try:
		Facemask()
	except rospy.ROSInterruptException:
		rospy.loginfo("object_detect test finished.")
