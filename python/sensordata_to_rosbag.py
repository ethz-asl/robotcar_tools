# !/usr/bin/python
#
# Convert the sensor data files in the given directory to a single rosbag.
#
# To call:
#
#   python sensordata_to_rosbag.py 2012-01-08/ 2012-01-08.bag
#

import argparse
import rosbag, rospy
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import NavSatStatus, NavSatFix, PointCloud2, PointField, Image, Imu, MagneticField
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from scipy import misc
import sys
import numpy as np
import struct
import IPython
import os
import time
from time_helpers import *
from read_images import *
from read_rtk_gps import *
from read_vo import *
from transformation_helpers import *

def write_rtk_gps(timestamp_ns, T_UTM_S, utm_zone, lat_deg, lon_deg, alt_m, status, bag):
    ros_timestamp = nanoseconds_to_ros_timestamp(timestamp_ns)

    ros_status = NavSatStatus()

    if status == 'INS_SOLUTION_GOOD':
        ros_status.status = NavSatStatus.STATUS_FIX
    else:
        ros_status.status = NavSatStatus.STATUS_FIX

    ros_status.service = NavSatStatus.SERVICE_GPS

    fix = NavSatFix()
    fix.status = ros_status
    fix.latitude = lat_deg
    fix.longitude = lon_deg
    fix.altitude = alt_m
    fix.header.frame_id = "WGS"
    fix.header.stamp = ros_timestamp

    rospose = Odometry()
    rospose.child_frame_id = "S"
    rospose.header.frame_id = "UTM"
    rospose.header.stamp = ros_timestamp
    rospose.pose.pose.position.x = T_UTM_S.getPosition()[0]
    rospose.pose.pose.position.y = T_UTM_S.getPosition()[1]
    rospose.pose.pose.position.z = T_UTM_S.getPosition()[2]
    rospose.pose.pose.orientation.x = T_UTM_S.getRotation().x()
    rospose.pose.pose.orientation.y = T_UTM_S.getRotation().y()
    rospose.pose.pose.orientation.z = T_UTM_S.getRotation().z()
    rospose.pose.pose.orientation.w = T_UTM_S.getRotation().w()
    rospose.twist.twist.linear.x = 0.0
    rospose.twist.twist.linear.y = 0.0
    rospose.twist.twist.linear.z = 0.0
    rospose.twist.twist.angular.x = 0.0
    rospose.twist.twist.angular.y = 0.0
    rospose.twist.twist.angular.z = 0.0

    bag.write('/gps_rtk', rospose, t=ros_timestamp)
    bag.write('/gps_rtk_fix', fix, t=ros_timestamp)

def write_odometry(ts_ns, T_XB30_XB3k, bag):
    ros_timestamp = nanoseconds_to_ros_timestamp(ts_ns)

    T_B0_Bk = get_T_B_XB3() * T_XB30_XB3k * get_T_B_XB3().inverse()

    rospose = Odometry()
    rospose.child_frame_id = "B"
    rospose.header.stamp = ros_timestamp
    rospose.pose.pose.position.x = T_B0_Bk.getPosition()[0]
    rospose.pose.pose.position.y = T_B0_Bk.getPosition()[1]
    rospose.pose.pose.position.z = T_B0_Bk.getPosition()[2]
    rospose.pose.pose.orientation.x = T_B0_Bk.getRotation().x()
    rospose.pose.pose.orientation.y = T_B0_Bk.getRotation().y()
    rospose.pose.pose.orientation.z = T_B0_Bk.getRotation().z()
    rospose.pose.pose.orientation.w = T_B0_Bk.getRotation().w()
    rospose.twist.twist.linear.x = 0.0
    rospose.twist.twist.linear.y = 0.0
    rospose.twist.twist.linear.z = 0.0
    rospose.twist.twist.angular.x = 0.0
    rospose.twist.twist.angular.y = 0.0
    rospose.twist.twist.angular.z = 0.0

    bag.write('/visual_odometry', rospose, t=ros_timestamp)

def write_img(ts_ns, img_array, camera, bag):
  ros_timestamp = nanoseconds_to_ros_timestamp(ts_ns)

  rosimage = Image()
  rosimage.data = img_array.tostring()
  rosimage.step = img_array.shape[1] #only with mono8! (step = width * byteperpixel * numChannels)
  rosimage.encoding = "mono8"
  rosimage.height = img_array.shape[0]
  rosimage.width = img_array.shape[1]
  rosimage.header.stamp = ros_timestamp

  bag.write('/' + camera + '/image_raw', rosimage, t=ros_timestamp)

def processDataset(root_dir, models_dir, dataset_name, out_dir, num_to_process, full_size_images=True, front_only=False):
    assert(len(dataset_name) > 0)

    dataset_dir = os.path.join(root_dir, dataset_name)

    assert(len(models_dir) > 0)

    print "Processing dataset ", dataset_name, ", num to process: ", num_to_process
    
    if full_size_images:
      bag_filepath = os.path.join(out_dir, dataset_name + '.bag')
    else:
      bag_filepath = os.path.join(out_dir, dataset_name + '_ds2.bag')

    print 'Opening bag ', bag_filepath, ' for writing.'
    bag = rosbag.Bag(bag_filepath, 'w')

    print "Writing odometry..."
    for t_ns, T_XB30_XB3k in iterate_vo(dataset_dir):
      write_odometry(t_ns, T_XB30_XB3k, bag)
    print "Done"

    print "Writing RTK GPS..."
    for timestamp_ns, T_UTM_S, utm_zone, lat_deg, lon_deg, alt_m, status in iterate_rtk_gps(dataset_dir):
      write_rtk_gps(timestamp_ns, T_UTM_S, utm_zone, lat_deg, lon_deg, alt_m, status, bag)
    print "Done"

    i = 0
    print "Writing images, front..."
    for ts_ns, img in iterate_images(dataset_dir, 'stereo/left', models_dir, full_size=full_size_images):
      write_img(ts_ns, img, 'mono_front', bag)
      i += 1
      if i > num_to_process:
        break
    print "Done"

    if not front_only:
      print "Writing images, left..."
      i = 0
      for ts_ns, img in iterate_images(dataset_dir, 'mono_left', models_dir):
        write_img(ts_ns, img, 'mono_left', bag)
        i += 1
        if i > num_to_process:
          break
      print "Done"

      print "Writing images, rear..."
      i = 0
      for ts_ns, img in iterate_images(dataset_dir, 'mono_rear', models_dir):
        write_img(ts_ns, img, 'mono_rear', bag)
        i += 1
        if i > num_to_process:
          break
      print "Done"

      print "Writing images, right..."
      i = 0
      for ts_ns, img in iterate_images(dataset_dir, 'mono_right', models_dir):
        write_img(ts_ns, img, 'mono_right', bag)
        i += 1
        if i > num_to_process:
          break
      print "Done"

    bag.close()

def main():
    parser = argparse.ArgumentParser(description='''UP-Drive VI-Map Evaluation''')
    parser.add_argument('root_dir')
    parser.add_argument('--dataset', default='')
    parser.add_argument('--out_dir', default='')
    parser.add_argument('--num_to_process', default=long(1e10))
    parser.add_argument('--front_only', default=False, action="store_true")
    
    parsed_args = parser.parse_args()

    root_dir = parsed_args.root_dir
    assert(os.path.exists(root_dir))

    models_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, 'models')
    assert(os.path.exists(models_dir))  

    out_dir = parsed_args.out_dir
    if len(out_dir) == 0:
      out_dir = os.path.join(root_dir, 'bags')  

    if not os.path.exists(out_dir):
      os.makedirs(out_dir)

    num_to_process = parsed_args.num_to_process

    front_only = parsed_args.front_only
    
    dataset_name = parsed_args.dataset
    if len(dataset_name) == 0:
      subfolders = os.listdir(root_dir)
      for subfolder in subfolders:
        if subfolder.count('-') == 5:
          processDataset(root_dir, models_dir, subfolder, out_dir, num_to_process, full_size_images=True, front_only=front_only)
          processDataset(root_dir, models_dir, subfolder, out_dir, num_to_process, full_size_images=False, front_only=front_only)

    else:
      processDataset(root_dir, models_dir, dataset_name, out_dir, num_to_process, full_size_images=True, front_only=front_only)
      processDataset(root_dir, models_dir, dataset_name, out_dir, num_to_process, full_size_images=False, front_only=front_only)


    return 0

if __name__ == '__main__':
    main()
