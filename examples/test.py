import rospy
import os
import cv2
import cv_bridge
import numpy as np

from cloudvis.srv import GraspPoint, GraspPointRequest
from qut_msgs.msg import Object

rospy.init_node('cloudvis_test')

bridge = cv_bridge.CvBridge()

path = os.path.dirname(os.path.abspath(__file__))

img = cv2.imread(os.path.join(path, 'ggcnn', 'pcd0102d.tiff'), -1)
image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")

obj = Object()

req = GraspPointRequest()
req.depth_image = image_message
req.object = obj

cloudvis = rospy.ServiceProxy('cloudvis/ggcnn', GraspPoint)
cloudvis.wait_for_service()

resp = cloudvis(req)
print(resp)