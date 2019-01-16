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

import os
import re
import cv2
import matplotlib.pyplot as plt
from datetime import datetime as dt
from image import load_image
from camera_model import CameraModel
import IPython
import scipy.misc

def iterate_images(root_dir, camera_description, models_dir):
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
      if os.path.exists(filename_processed):
        im_gray_downscaled = cv2.imread(filename_processed, cv2.IMREAD_GRAYSCALE)

      else:
        img = load_image(filename, model)

        im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        im_gray_downscaled = cv2.resize(im_gray, (im_gray.shape[1] / 2, im_gray.shape[0] / 2))
        cv2.imwrite(im_gray_downscaled, filename_processed)

      yield timestamp_ns, im_gray_downscaled
