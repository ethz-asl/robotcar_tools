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
import matplotlib.pyplot as plt
from datetime import datetime as dt
from image import load_image
from camera_model import CameraModel

def main():
  parser = argparse.ArgumentParser()

  parser.add_argument('timestamps_path', type=str, help='...')
  args = parser.parse_args()

  timestamps_path = args.timestamps_path

  if not os.path.isfile(timestamps_path):
    timestamps_path = os.path.join(args.dir, os.pardir, os.pardir, camera + '.timestamps')
    if not os.path.isfile(timestamps_path):
        raise IOError("Could not find timestamps file")


  timestamps_ns = []
  timestamps_file = open(timestamps_path)
  for line in timestamps_file:
      tokens = line.split()
      timestamp_us = long(tokens[0])
      timestamp_ns = timestamp_us * long(1000)

      chunk = int(tokens[1])

      filename = os.path.join(args.dir, tokens[0] + '.png')
      if not os.path.isfile(filename):
          if chunk != current_chunk:
              print("Chunk " + str(chunk) + " not found")
              current_chunk = chunk
          continue

      current_chunk = chunk

      img = load_image(filename, model)

if __name__ == '__main__':
    main()
