#!/usr/bin/env python
from __future__ import print_function

import rospy
import os
import cv2
import cv_bridge

from cloudvis.srv import Get, GetRequest
from cloudvis.msg import Property

from qut_msgs.srv import GetDetections, GetDetectionsRequest

path = os.path.dirname(os.path.abspath(__file__))

img = cv2.imread(os.path.join(path, 'dog.jpg'))

bridge = cv_bridge.CvBridge()

rospy.init_node('cloudvis_test')

cloudvis = rospy.ServiceProxy('cloudvis', Get)

req = GetRequest()
req.service_name = 'yolo'

image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")

req.properties.append(Property(name='image', image=image_message))
req.properties.append(Property(name='render', data='true'))

response = cloudvis(req)

for prop in response.result:
  if prop.image.data:
    image = bridge.imgmsg_to_cv2(prop.image, desired_encoding="passthrough")
    cv2.imshow('Image', image)
    cv2.waitKey(0)
  else:
    print('{}: {}'.format(prop.name, prop.data))

cloudvis = rospy.ServiceProxy('cloudvis/yolo', GetDetections)

req = GetDetectionsRequest()
req.observation = Observation(rgb_image=image_message)

response = cloudvis(req)

for obj in response.result.detections:
  image = bridge.imgmsg_to_cv2(obj.cropped_rgb, desired_encoding="passthrough")

  cv2.imshow(obj.class_label, image)
  cv2.waitKey(0)


