#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  3 22:13:47 2019

@author: vik748
# sudo apt install ros-kinetic-jsk-common-msgs
"""
from __future__ import print_function
import numpy as np
import cv2
import glob
import re
import argparse
import logging
from ros import rosbag
import roslib
import rospy
import yaml
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
import sys
import time
#import ImageFile
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
progressbar.streams.wrap_stderr()

bglog = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(levelname)8s  %(message)s')
mpl_logger = logging.getLogger('matplotlib') 
mpl_logger.setLevel(logging.WARNING) 


def CreateMonoBag(imgs,bagname):
    '''Creates a bag file with camera images'''
    ts = rospy.rostime.Time.from_sec(time.time())
    interval = 1.0/freq
    bag = rosbag.Bag(bagname, 'w')
    seq = 0
    if display:
        im = cv2.imread(imgs[0], cv2.IMREAD_GRAYSCALE)            
        fig, ax= plt.subplots(dpi=200)
        fig_image = plt.imshow(im,cmap='gray')
        plt.axis("off")
        
    try:
        for im_name in progressbar.progressbar(imgs):
            bglog.debug("Adding %s" % im_name)
            im = cv2.imread(im_name, cv2.IMREAD_GRAYSCALE)
            
            if use_clahe:
                im = clahe.apply(im)
            
            if display:
                fig_image.set_data(im)
                plt.draw()
                plt.pause(0.0001)

            Stamp = ts
            ts = ts + rospy.rostime.Duration.from_sec(interval)
            try:
                Img = bridge.cv2_to_imgmsg(im)
            except CvBridgeError as e:
                bglog.error(e)
                            
            Img.header.stamp = Stamp
            Img.header.seq = seq
            
            seq += 1
            bag.write(out_topic, Img, Stamp)
    finally:
        bag.close()       

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='This scripts finds Zenike \
                                     features for all images in a folder and adds \
                                     them to a bagfile')
    parser.add_argument('-c', '--config', help='location of config file in yaml format',
                        default='../config/kitti_config.yaml') #go_pro_icebergs_config.yaml
    args = parser.parse_args()
             
    config_dict = yaml.safe_load(open(args.config))
    USE_MASKS = config_dict['use_masks']
    image_ext = config_dict['image_ext']
    use_clahe = config_dict['use_clahe']
    
    if use_clahe:
        clahe = cv2.createCLAHE(**config_dict['CLAHE_settings'])
    
    if sys.platform == 'darwin':
        image_folder = config_dict['osx_image_folder']
        if USE_MASKS: masks_folder = config_dict['osx_masks_folder']

        window_xadj = 0
        window_yadj = 45
    else:
        image_folder = config_dict['linux_image_folder']
        if USE_MASKS: masks_folder = config_dict['linux_masks_folder']
        window_xadj = 65
        window_yadj = 430

    images_full = sorted([f for f in glob.glob(image_folder+'/*') 
                     if re.match('^.*\.'+image_ext+'$', f, flags=re.IGNORECASE)])
        
    assert images_full is not None, "ERROR: No images read"
    freq = config_dict['frequency']
    display = config_dict['display_images']
    out_topic = config_dict['image_topic']
    
    bridge = CvBridge()
        
    CreateMonoBag(images_full, config_dict['bag_name'])
    
