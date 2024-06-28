#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
#import sys
import rospy
from std_msgs.msg import String

import torch
import torch.nn.parallel
import torch.nn.functional as F
import numpy as np
import cv2
from LPN import LPN
from fall_net import Fall_Net
from pose_utils import Cropmyimage
from pose_utils import Drawkeypoints
import plot_sen
from time import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global cam_image
def callback(data):
    try:
        global cam_image
        cam_image = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height, data.width, -1))
        #print(cam_image.shape)

    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('detector', anonymous=True)
    pub = rospy.Publisher('det_result', Image, queue_size=10)
    rospy.Subscriber('cam_image', Image, callback)
    rate = rospy.Rate(50) # 10hz
    # model 
    pose_net = LPN(nJoints=17)
    pose_net.load_state_dict(torch.load('/home/ubuntu/catkin_ws/src/ROS_Fall_Det/pose_net_pred100_python2.pth', map_location=torch.device('cpu')))
    fall_net = Fall_Net(64, 48, 17, device=torch.device('cpu'))
    fall_net.load_state_dict(torch.load('/home/ubuntu/catkin_ws/src/ROS_Fall_Det/fall_net_pred5_python2.pth', map_location=torch.device('cpu')))
    pose_net.eval()
    fall_net.eval()
    print('Load successfully!')

    bridge = CvBridge()
    global cam_image
    cam_image = np.array([])
    fall_count = []
    while not rospy.is_shutdown():
        rate.sleep()
        if not cam_image.any():
            print('waiting!')
            continue
        start = time()
        
        # image initialize
        input = cam_image
        bbox = [0, 0, input.shape[1], input.shape[0]]
        input_image, details = Cropmyimage(input, bbox)
        input_image = np.array([input_image.numpy()])
        input_image = torch.from_numpy(input_image)
        pose_out = pose_net(input_image)
        fall_out, pose_cor = fall_net(pose_out)

        # Print shapes for debugging
        # print("pose_out shape:", pose_out.shape)
        # print("pose_cor shape:", pose_cor.shape)

        # Ensure pose_cor has a batch dimension
        if len(pose_cor.shape) == 2:
            pose_cor = pose_cor.unsqueeze(0)

        neck = (pose_cor[:, 5:6, :] + pose_cor[:, 6:7, :]) / 2
        pose_cor = torch.cat((pose_cor, neck), dim=1)
        pose_cor = pose_cor * 4 + 2.
        scale = torch.Tensor([[256, 192]])
        pose_cor = pose_cor / scale
        scale = torch.Tensor([[details[3]-details[1], details[2]-details[0]]])
        pose_cor = pose_cor * scale
        scale = torch.Tensor([[details[1], details[0]]])
        pose_cor = pose_cor + scale
        pose_cor = torch.flip(pose_cor, dims=[2])
        ones = torch.ones(1, 18, 1)
        pose_cor = torch.cat((pose_cor, ones), dim=2).cpu().detach().numpy()
        det_result = plot_sen.plot_poses(input, pose_cor)

        fall_out = F.softmax(fall_out, dim=1)  # Apply softmax along the correct dimension
        # print("fall_out after softmax:", fall_out)

        fall_out_max = torch.max(fall_out, dim=1)  # Max along the correct dimension
        # print("fall_out_max:", fall_out_max)

        # Debugging prints
        # print("fall_out_max.indices shape:", fall_out_max.indices.shape)
        # print("fall_out_max.indices:", fall_out_max.indices)

        # Ensure scalar value is appended
        fall_count.append(fall_out_max.indices.item() if fall_out_max.indices.numel() == 1 else fall_out_max.indices[0].item())
        fall_dis = sum(fall_count[len(fall_count)-30 : len(fall_count)])
        end = time()
        run_time = end-start
        if fall_dis > 24:
            print('Normal!', 1. / run_time)
        else:
            print('Down!', 1. / run_time)
        det_result = bridge.cv2_to_imgmsg(det_result, encoding="passthrough")
        pub.publish(det_result)
