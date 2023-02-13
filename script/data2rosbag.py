#!/usr/bin/python
import argparse
from recording_pb2 import VideoCaptureData
import os.path as osp
import os
import cv2
import csv
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
# from cv_bridge import CvBridge
from pyquaternion import Quaternion
import numpy as np
import shutil
import yaml
from utils import OpenCVDumper
import time
from cvbridge import cv2_to_imgmsg, imgmsg_to_cv2


#bridge = CvBridge()
NSECS_IN_SEC=int(1e9)

def convert_to_bag(proto, video_path, result_path, save_image=True, subsample=1, compress_img=False, compress_bag=False, resize = [], raw_imu =False):
    #Init rosbag
    # bz2 is better compression but lz4 is 3 times faster
    resolution = None
    img_topic = "/cam0/image_raw/compressed" if compress_img else "/cam0/image_raw"
    print('img_topic=', img_topic)

    try:
        bag = rosbag.Bag(result_path, 'w', compression='lz4' if compress_bag else 'none')

        if save_image:
            # Open video stream
            try:
                cap = cv2.VideoCapture(video_path)
                
                start = None 
                # Generate images from video and frame data
                for i,frame_data in enumerate(proto.video_meta):
                    ret, frame = cap.read()

                    '''
                    if i == 0:
                        continue 
                    if i == 1:
                        start = frame_data.time_ns

                    # if not i==frame_data.frame_number: print('skipping frame {}, missing data'.format(i))
                    
                    k = i - 1 
                    '''
                    k = i 
                    if (k % subsample) == 0: # and i != 0 and i > 30:
                        rosimg, timestamp, resolution = img_to_rosimg(frame,
                                                                    frame_data.time_ns,
                                                                    compress=compress_img,
                                                                    resize = resize)

                        # back = imgmsg_to_cv2(rosimg)
                        #cv2.imshow('window', back)
                        #cv2.waitKey(1000)
                        # print(type(back))
                        bag.write(img_topic, rosimg, timestamp)

            finally:
                cap.release()

        # Now IMU
        flag = True 
        item = None
        for imu_frame in proto.imu:
            if not raw_imu:
                gyro_drift = getattr(imu_frame, 'gyro_drift', np.zeros(3))
                accel_bias = getattr(imu_frame, 'accel_bias', np.zeros(3))
            else:
                gyro_drift = accel_bias = np.zeros(3)
            
            '''
            ns = imu_frame.time_ns
            if ns < start:
                item = [imu_frame.time_ns, imu_frame.gyro, gyro_drift, imu_frame.accel, accel_bias] 
                print(f'skip imu {ns}, first frame={start}, diff={start - ns}')
                continue 
            
            if flag:
                rosimu, timestamp = imu_to_rosimu(item[0], item[1], item[2], item[3], item[4])
                bag.write("/imu0", rosimu, timestamp)
                flag = False 
                print(f'second imu {ns}, first frame={start}, diff={start - ns}')
            '''

            rosimu, timestamp = imu_to_rosimu(imu_frame.time_ns, imu_frame.gyro, gyro_drift, imu_frame.accel, accel_bias)
            bag.write("/imu0", rosimu, timestamp)

    finally:
        bag.close()

    return resolution



def img_to_rosimg(img, timestamp_nsecs, compress = True, resize = []):

    timestamp = rospy.Time(secs=timestamp_nsecs//NSECS_IN_SEC,
                           nsecs=timestamp_nsecs%NSECS_IN_SEC)

    gray_img  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if resize:
        gray_img = cv2.resize(gray_img, tuple(resize), cv2.INTER_AREA)
        assert gray_img.shape[0] == resize[1]

    if compress:
        rosimage = bridge.cv2_to_compressed_imgmsg(gray_img, dst_format='png')
    else:
        # rosimage = bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
        rosimage = cv2_to_imgmsg(gray_img, encoding="mono8")
    rosimage.header.stamp = timestamp

    return rosimage, timestamp, (gray_img.shape[1], gray_img.shape[0])

def imu_to_rosimu(timestamp_nsecs, omega, omega_drift, alpha, alpha_bias):
    timestamp = rospy.Time(secs=timestamp_nsecs//NSECS_IN_SEC,
                           nsecs=timestamp_nsecs%NSECS_IN_SEC)

    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = omega[0] - omega_drift[0]
    rosimu.angular_velocity.y = omega[1] - omega_drift[1]
    rosimu.angular_velocity.z = omega[2] - omega_drift[2]
    rosimu.linear_acceleration.x = alpha[0] - alpha_bias[0]
    rosimu.linear_acceleration.y = alpha[1] - alpha_bias[1]
    rosimu.linear_acceleration.z = alpha[2] - alpha_bias[2]

    return rosimu, timestamp

def adjust_calibration(input_yaml_path, output_yaml_path, resolution):
    with open(input_yaml_path,'r') as f:
        calib = yaml.safe_load(f)

    cam0 = calib['cam0']
    if cam0['resolution'][0] != resolution[0]:
        sx = float(resolution[0])/cam0['resolution'][0]
        cam0['intrinsics'][0] *= sx
        cam0['intrinsics'][2] *= sx
        cam0['resolution'][0] = resolution[0]

    if cam0['resolution'][1] != resolution[1]:
        sy = float(resolution[1])/cam0['resolution'][1]
        cam0['intrinsics'][1] *= sy
        cam0['intrinsics'][3] *= sy
        cam0['resolution'][1] = resolution[1]

    with open(output_yaml_path,'w') as f:
        yaml.dump(calib, f, Dumper=OpenCVDumper)

