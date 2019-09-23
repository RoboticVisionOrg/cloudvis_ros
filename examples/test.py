#!/usr/bin/env python
from __future__ import print_function

import rospy
import os
import cv2
import cv_bridge

from cloudvis.srv import Get, GetRequest
from cloudvis.msg import Property

path = os.path.dirname(os.path.abspath(__file__))

img = cv2.imread(os.path.join(path, 'dog.jpg'))

bridge = cv_bridge.CvBridge()

rospy.init_node('cloudvis_test')

cloudvis = rospy.ServiceProxy('cloudvis', Get)

req = GetRequest()
req.service_name = 'skinCancerDetector'

image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")

req.properties.append(Property(name='input_image', image=image_message))
req.properties.append(Property(name='render', data='true'))

response = cloudvis(req)

for prop in response.result:
  if prop.image.data:
    image = bridge.imgmsg_to_cv2(prop.image, desired_encoding="passthrough")
    cv2.imshow('Image', image)
    cv2.waitKey(0)
  else:
    print('{}: {}'.format(prop.name, prop.data))