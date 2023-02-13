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
import numpy as np
import shutil
import time


# bridge = CvBridge()
NSECS_IN_SEC=int(1e9)

def convert_to_data(proto, video_path, result_dir, raw_imu =False):
    #Init rosbag
    # bz2 is better compression but lz4 is 3 times faster
    resolution = None
    if os.path.exists(result_dir):
        shutil.rmtree(result_dir)

    os.makedirs(result_dir)

    fs = open(osp.join(result_dir, 'timestamp.txt'), 'w', encoding='utf-8')
    imdir = osp.join(result_dir, 'img')
    os.makedirs(imdir)

    try:
        
        try:
            cap = cv2.VideoCapture(video_path)

            # Generate images from video and frame data
            for i,frame_data in enumerate(proto.video_meta):
                ret, frame = cap.read()
                if not i==frame_data.frame_number:
                    print(f'skipping frame {i}, missing data, i={i}, frame_number={frame_data.frame_number}')
                    
                subsample = 30
                #if (i % subsample) == 0 and i != 0 :
                if True:
                    name = str(frame_data.time_ns)
                    #frame  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #.T
                    #cv2.rotate(frame, frame, 90) #cv2.ROTATE_90_CLOCKWISE);
                    
                    if True:
                        cv2.imwrite(osp.join(imdir, name+'.png'), frame)
                        fs.write(name + '\n')
    
        finally:
            cap.release()
        
        fs.close()
        # Now IMU
        # #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        
        fs = open(osp.join(result_dir, 'imu.csv'), 'w', encoding='utf-8')
        fs.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
        for imu_frame in proto.imu:
            if not raw_imu:
                gyro_drift = getattr(imu_frame, 'gyro_drift', np.zeros(3))
                accel_bias = getattr(imu_frame, 'accel_bias', np.zeros(3))
            else:
                gyro_drift = accel_bias = np.zeros(3)

            rosimu, timestamp = imu_to_rosimu(imu_frame.time_ns, imu_frame.gyro, gyro_drift, imu_frame.accel, accel_bias)
            

            fs.write(f'{imu_frame.time_ns},{rosimu.angular_velocity.x},{rosimu.angular_velocity.y},{rosimu.angular_velocity.z},{rosimu.linear_acceleration.x},{rosimu.linear_acceleration.y},{rosimu.linear_acceleration.z}\n')

    finally:
        pass 

    fs.close()


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



if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Convert video and proto to rosbag')
    parser.add_argument('--data_dir', type=str, help='Path to folder with video_recording.mp4 and video_meta.pb3 or root-folder containing multiple datasets')
    parser.add_argument('--raw-imu', action='store_true', help='Do not compensate for bias')

    args = parser.parse_args()

    for root, dirnames, filenames in os.walk(args.data_dir):
        if not 'video_meta.pb3' in filenames:
            continue

        # Read proto
        proto_path = osp.join(root, 'video_meta.pb3')
        with open(proto_path,'rb') as f:
            proto = VideoCaptureData.FromString(f.read())

        video_path = osp.join(root, 'video_recording.mp4')
        des = os.path.join(root, 'data')

        convert_to_data(proto,
                             video_path,
                             des,
                             raw_imu = args.raw_imu)

