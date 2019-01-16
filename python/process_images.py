################################################################################
#
# Copyright (c) 2017 University of Oxford
# Authors:
#  Geoff Pascoe (gmp@robots.ox.ac.uk)
#
# This work is licensed under the Creative Commons
# Attribution-NonCommercial-ShareAlike 4.0 International License.
# To view a copy of this license, visit
# http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to
# Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
#
################################################################################

import argparse
import os
import re
import cv2
import matplotlib.pyplot as plt
from datetime import datetime as dt
from image import load_image
from camera_model import CameraModel
import IPython
import scipy.misc

def process_images(root_dir, camera_description, models_dir):
  if 'stereo' in camera_description:
    timestamps_path = os.path.join(root_dir, 'stereo.timestamps')
  else:
    timestamps_path = os.path.join(root_dir, camera_description + '.timestamps')

  if not os.path.isfile(timestamps_path):
    raise IOError("Could not find timestamps file")

  images_dir = os.path.join(root_dir, camera_description)
  model = CameraModel(models_dir, images_dir)

  current_chunk = 0
  timestamps_file = open(timestamps_path)

  idx = 0
  for line in timestamps_file:
      tokens = line.split()
      timestamp_us = long(tokens[0])
      timestamp_ns = timestamp_us * long(1000)

      datetime = dt.utcfromtimestamp(int(tokens[0])/1000000)
      chunk = int(tokens[1])

      filename = os.path.join(images_dir, tokens[0] + '.png')
      if not os.path.isfile(filename):
          if chunk != current_chunk:
              print("Chunk " + str(chunk) + " not found")
              current_chunk = chunk
          continue

      current_chunk = chunk

      filename_processed = os.path.join(images_dir, tokens[0] + '_ds2_undistorted.png')
      if not os.path.exists(filename_processed):
        img = load_image(filename, model)

        im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        im_gray_downscaled = cv2.resize(im_gray, (im_gray.shape[1] / 2, im_gray.shape[0] / 2))

        cv2.imwrite(filename_processed, im_gray_downscaled)

      if idx % 100 == 0:
        print "At ", idx

      idx += 1


def processDataset(root_dir, models_dir, dataset_name, camera=''):
    assert(len(dataset_name) > 0)

    dataset_dir = os.path.join(root_dir, dataset_name)

    assert(len(models_dir) > 0)

    print "Processing dataset ", dataset_name
    
    if camera == '':
      print "Processing images, front..."
      process_images(dataset_dir, 'stereo/left', models_dir)

      print "Processing images, left..."
      process_images(dataset_dir, 'mono_left', models_dir)
      print "Done"

      print "Processing images, rear..."
      process_images(dataset_dir, 'mono_rear', models_dir)
      print "Done"

      print "Processing images, right..."
      process_images(dataset_dir, 'mono_right', models_dir)
      print "Done"  
    else:
      print "Processing images, ", camera, "..."
      process_images(dataset_dir, camera, models_dir)
      print "Done"  

def main():
    parser = argparse.ArgumentParser(description='''...''')
    parser.add_argument('root_dir')
    parser.add_argument('models_dir')
    parser.add_argument('--dataset', default='')
    parser.add_argument('--camera', default='')
    
    parsed_args = parser.parse_args()

    root_dir = parsed_args.root_dir
    assert(os.path.exists(root_dir))

    models_dir = parsed_args.models_dir
    assert(os.path.exists(models_dir))

    camera = parsed_args.camera
    
    dataset_name = parsed_args.dataset
    if len(dataset_name) == 0:
      subfolders = os.listdir(root_dir)
      for subfolder in subfolders:
        if subfolder.count('-') == 5:
          processDataset(root_dir, models_dir, subfolder, camera)

    else:
      processDataset(root_dir, models_dir, dataset_name, camera)

    return 0

if __name__ == '__main__':
    main()
