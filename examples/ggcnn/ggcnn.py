#!/usr/bin/env python
from __future__ import print_function

import rospy
import os
import sys
import cv2
import numpy as np
import cv_bridge
import base64

from cloudvis.srv import Get, GetRequest
from cloudvis.msg import Property

path = os.path.dirname(os.path.abspath(__file__))

rospy.init_node('cloudvis_test')

bridge = cv_bridge.CvBridge()

cloudvis = rospy.ServiceProxy('cloudvis', Get)

req = GetRequest()
req.service_name = 'ggcnn'

img = cv2.imread(os.path.join(path, 'pcd0102d.tiff'), -1)
image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")

req.properties.append(Property(name='image', image=image_message))
req.properties.append(Property(name='render', data='true'))

response = cloudvis(req)

for prop in response.result:
  if prop.image.data:
    result = bridge.imgmsg_to_cv2(prop.image, desired_encoding="passthrough")
    cv2.imshow('Image', img)
    cv2.imshow('Grasp Points', result)
    cv2.waitKey(0)
  else:
    print('{}: {}'.format(prop.name, prop.data))