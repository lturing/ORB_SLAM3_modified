#!/usr/bin/python
import argparse
from recording_pb2 import VideoCaptureData
import os.path as osp
import os
import cv2
import csv
import yaml
from pyquaternion import Quaternion
import numpy as np
import shutil
from data2rosbag import convert_to_bag
import numpy as np


def create_camera_yaml(proto, camera_yaml_path):
    c = proto.camera_meta
    est_focal_length = proto.video_meta[0].est_focal_length_pix
    print(c.lens_pose_rotation)
    
    '''
    q = Quaternion(c.lens_pose_rotation[3], *c.lens_pose_rotation[:3])
    print('intrinsics: ', c.intrinsic_params)
    P = q.transformation_matrix
    print("Translation")
    print(c.lens_pose_translation)
    P[:3,3] = -np.matmul(q.rotation_matrix,c.lens_pose_translation)
    print("P")
    print(P)
    '''

    P = [ 0.99997874,  0.00527204,  0.00383698,  0.0357282,
      0.00529593, -0.99996649, -0.00624306, -0.00774089,
      0.00380394,  0.00626325, -0.99997315, -0.00668908,
      0.,          0.,          0.,          1.        ]

    intrinsics = [ 596.43479098,  596.30401751,  299.97118587,  376.29549719]
    # [0.0842841997124349, -0.16181139298878658, -0.0038366505561627027, 0.009092039977852805
    # [k1 k2 r1 r2]

    radial_dist = [-0.0038366505561627027, 0.009092039977852805]
    #[0.0] * 2

    tangential_dist = [0.0842841997124349, -0.16181139298878658]
    # [0.0] * 2

    distortion_coeffs = [0.0842841997124349, -0.16181139298878658, -0.0038366505561627027, 0.009092039977852805]
    # intrinsics = 

    camera_dict = {
        'camera_model': 'pinhole',
        'intrinsics': intrinsics,
        'distortion_model': 'radtan',
        'distortion_coeffs': distortion_coeffs, #radial_dist + tangential_dist,
        'T_cam_imu': P, #.tolist(),
        'timeshift_cam_imu': 0,
        'rostopic': '/cam0/image_raw',
        'resolution': [c.resolution.width, c.resolution.height],
        'cam_overlaps': []
    }

    with open(camera_yaml_path, 'w') as f:
        yaml.safe_dump({'cam0':camera_dict}, f, default_flow_style=False)

def create_imu_yaml_1(proto, imu_yaml_path):
    #Noise density values from pixel 3 IMU datasheet
    imu_dict = {
        'accelerometer_noise_density': 9.8*180e-6,   #Noise density (continuous-time)
        'accelerometer_random_walk':   9.8*180e-6,   #Bias random walk
        'gyroscope_noise_density':     0.007*np.pi/180.0,   #Noise density (continuous-time)
        'gyroscope_random_walk':       0.007*np.pi/180.0,   #Bias random walk
        'rostopic':                    '/imu0',      #the IMU ROS topic
        'update_rate':                 proto.imu_meta.sample_frequency   #Hz (for discretization of the values above)
    }

    with open(imu_yaml_path, 'w') as f:
        yaml.safe_dump(imu_dict, f, default_flow_style=False)


def create_imu_yaml_1(proto, imu_yaml_path):
    #Noise density values from pixel 3 IMU datasheet
    imu_dict = { 
        'accelerometer_noise_density': 16.0e-3, #9.8*180e-6,   #Noise density (continuous-time)
        'accelerometer_random_walk':   5.5e-5, # 9.8*180e-6,   #Bias random walk
        'gyroscope_noise_density':     24.0e-4	, # 0.007*np.pi/180.0,   #Noise density (continuous-time)
        'gyroscope_random_walk':       2.0e-5	, # 0.007*np.pi/180.0,   #Bias random walk
        'rostopic':                    '/imu0',      #the IMU ROS topic
        'update_rate':                 proto.imu_meta.sample_frequency   #Hz (for discretization of the values above)
    }   

    with open(imu_yaml_path, 'w') as f:
        yaml.safe_dump(imu_dict, f, default_flow_style=False)



def create_imu_yaml_22(proto, imu_yaml_path):
    # from imu_utils
    #Noise density values from pixel 3 IMU datasheet
    imu_dict = { 
        'accelerometer_noise_density': 3.0512634784943819e-02, #Noise density (continuous-time)
        'accelerometer_random_walk':   9.5379674088788373e-04, #Bias random walk
        'gyroscope_noise_density':     2.0677945491652749e-03, #Noise density (continuous-time)
        'gyroscope_random_walk':       3.4130103547126040e-05, #Bias random walk
        'rostopic':                    '/imu0',      #the IMU ROS topic
        'update_rate':                 proto.imu_meta.sample_frequency   #Hz (for discretization of the values above)
    }   

    with open(imu_yaml_path, 'w') as f:
        yaml.safe_dump(imu_dict, f, default_flow_style=False)

def create_imu_yaml_2(proto, imu_yaml_path):
    # from allan_variance_ros
    #Noise density values from pixel 3 IMU datasheet
    factor = 15.0
    '''
        #Accelerometer
        accelerometer_noise_density: 0.0024114438724648136
        accelerometer_random_walk: 0.0002443341374270453

        #Gyroscope
        gyroscope_noise_density: 0.00012414996824684825
        gyroscope_random_walk: 2.4141054948567875e-06

    '''
    imu_dict = { 
        'accelerometer_noise_density': 0.0024114438724648136 * factor, #Noise density (continuous-time)
        'accelerometer_random_walk':   0.0002443341374270453 * factor, #Bias random walk
        'gyroscope_noise_density':     0.00012414996824684825 * factor, #Noise density (continuous-time)
        'gyroscope_random_walk':       2.4141054948567875e-06 * factor, #Bias random walk
        'rostopic':                    '/imu0',      #the IMU ROS topic
        'update_rate':                 proto.imu_meta.sample_frequency   #Hz (for discretization of the values above)
    }   

    with open(imu_yaml_path, 'w') as f:
        yaml.safe_dump(imu_dict, f, default_flow_style=False)



def create_imu_yaml(proto, imu_yaml_path):
    # from allan_variance_ros
    #Noise density values from pixel 3 IMU datasheet
    factor = 3
    '''
        #Accelerometer
        accelerometer_noise_density: 0.0023413009369821977 
        accelerometer_random_walk: 0.00019010025012693242 

        #Gyroscope
        gyroscope_noise_density: 0.00012728232878921668 
        gyroscope_random_walk: 2.441367294141376e-06 
    '''
    imu_dict = {
        'accelerometer_noise_density': 0.0023413009369821977 * factor, #Noise density (continuous-time)
        'accelerometer_random_walk':   0.00019010025012693242 * factor, #Bias random walk
        'gyroscope_noise_density':     0.00012728232878921668 * factor, #Noise density (continuous-time)
        'gyroscope_random_walk':       2.441367294141376e-06 * factor, #Bias random walk
        'rostopic':                    '/imu0',      #the IMU ROS topic
        'update_rate':                 proto.imu_meta.sample_frequency,   #Hz (for discretization of the values above)
        'factor11':                    factor,
    }

    with open(imu_yaml_path, 'w') as f:
        yaml.safe_dump(imu_dict, f, default_flow_style=False)


def create_target_yaml(target_path):
    target = {
        'target_type': 'aprilgrid', #gridtype
        'tagCols': 6,               #number of apriltags
        'tagRows': 4,               #number of apriltags
        'tagSize': 0.035,           #size of apriltag, edge to edge [m]
        'tagSpacing': 0.2          #ratio of space between tags to tagSize
                             #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]
    }
    with open(target_path, 'w') as f:
        yaml.safe_dump(target, f, default_flow_style=False)


def create_target_yaml_1(target_path):
    target = { 
        'target_type': 'checkerboard', #gridtype
        'targetCols': 6,               #number of internal chessboard corners
        'targetRows': 8,               #number of internal chessboard corners
        'rowSpacingMeters': 0.025,     #size of one chessboard square [m]
        'colSpacingMeters': 0.025      #size of one chessboard square [m]
    }   
    with open(target_path, 'w') as f:
        yaml.safe_dump(target, f, default_flow_style=False)




if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Prepare video and proto for Kalibr')
    parser.add_argument('data_dir', type=str, help='Path to folder with video_recording.mp4 and video_meta.pb3')
    parser.add_argument('--result-dir', type=str, help='Path to result folder, default same as proto', default = None)
    parser.add_argument('--subsample', type=int, help='Take every n-th video frame', default = 1)
    parser.add_argument('--save_image', type=bool, help="whether to save image.", default=True)


    args = parser.parse_args()
    result_dir = args.result_dir if args.result_dir else osp.join(args.data_dir, 'kalibr')
    os.makedirs(result_dir, exist_ok=True)

    # Read proto
    proto_path = osp.join(args.data_dir, 'video_meta.pb3')
    with open(proto_path, 'rb') as f:
        proto = VideoCaptureData.FromString(f.read())

    video_path = osp.join(args.data_dir, 'video_recording.mp4')
    bag_path = osp.join(result_dir, 'kalibr.bag')
    convert_to_bag(proto, video_path, bag_path, subsample=args.subsample, save_image=args.save_image)
    
    camera_yaml_path = osp.join(result_dir, 'kalibr-camchain.yaml')
    #create_camera_yaml(proto, camera_yaml_path)
    imu_yaml_path = osp.join(result_dir, 'imu.yaml')
    create_imu_yaml(proto, imu_yaml_path)
    target_yaml_path = osp.join(result_dir, 'target.yaml')
    create_target_yaml(target_yaml_path)
