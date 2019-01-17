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

    T_B0_Bk = get_T_B_XB3() * T_XB30_XB3k * get_T_B_XB3.inverse()

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

def processDataset(root_dir, models_dir, dataset_name, out_dir):
    assert(len(dataset_name) > 0)

    dataset_dir = os.path.join(root_dir, dataset_name)

    assert(len(models_dir) > 0)

    print "Processing dataset ", dataset_name
    
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

    print "Writing images, front..."
    for ts_ns, img in iterate_images(dataset_dir, 'stereo/left', models_dir):
      write_img(ts_ns, img, 'mono_front', bag)
    print "Done"

    print "Writing images, left..."
    for ts_ns, img in iterate_images(dataset_dir, 'mono_left', models_dir):
      write_img(ts_ns, img, 'mono_left', bag)
    print "Done"

    print "Writing images, rear..."
    for ts_ns, img in iterate_images(dataset_dir, 'mono_rear', models_dir):
      write_img(ts_ns, img, 'mono_rear', bag)
    print "Done"

    print "Writing images, right..."
    for ts_ns, img in iterate_images(dataset_dir, 'mono_right', models_dir):
      write_img(ts_ns, img, 'mono_right', bag)
    print "Done"

    bag.close()

def main():
    parser = argparse.ArgumentParser(description='''UP-Drive VI-Map Evaluation''')
    parser.add_argument('root_dir')
    parser.add_argument('--dataset', default='')
    parser.add_argument('--out_dir', default='')
    
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
    
    dataset_name = parsed_args.dataset
    if len(dataset_name) == 0:
      subfolders = os.listdir(root_dir)
      for subfolder in subfolders:
        if subfolder.count('-') == 5:
          processDataset(root_dir, models_dir, subfolder, out_dir)

    else:
      processDataset(root_dir, models_dir, dataset_name, out_dir)


    return 0

if __name__ == '__main__':
    main()
