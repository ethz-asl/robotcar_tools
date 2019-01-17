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
import csv
import IPython
import numpy as np
from transformation_helpers import *
from transform import *

def iterate_rtk_gps(root):
  ins_path = os.path.join(root, "gps", "ins.csv")

  with open(ins_path) as ins_file:
      ins_reader = csv.reader(ins_file)
      headers = next(ins_file)

      for row in ins_reader:  
          timestamp_ns = long(row[0]) * long(1000)
          status = str(row[1])
          lat_deg = float(row[2])
          lon_deg = float(row[3])
          alt_m = float(row[4])
          north = float(row[5])
          east = float(row[6])
          down = float(row[7])
          utm_zone = str(row[8])
          
          roll_rad = float(row[12])
          pitch_rad = float(row[13])
          yaw_rad = float(row[14])

          T_UTM_S = get_T_ENU_NED() * mk.Transformation(build_se3_transform([north, east, down, roll_rad, pitch_rad, yaw_rad]))

          yield timestamp_ns, T_UTM_S, utm_zone, lat_deg, lon_deg, alt_m, status

  
