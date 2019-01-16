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


def iterate_vo(root):
  ins_path = os.path.join(root, "vo", "vo.csv")

  T_O_Ik = mk.Transformation()

  first = True
  with open(ins_path) as ins_file:
      ins_reader = csv.reader(ins_file)
      headers = next(ins_file)

      for row in ins_reader:
          timestamp_us_source = long(row[0])
          timestamp_ns_source = timestamp_us_source * long(1000)

          timestamp_us_dest = long(row[1])
          timestamp_ns_dest = timestamp_us_dest * long(1000)
  
          x = float(row[2])
          y = float(row[3])
          z = float(row[4])
          roll_rad = float(row[5])
          pitch_rad = float(row[6])
          yaw_rad = float(row[7])

          T_Ik_Ikp1 =  mk.Transformation(build_se3_transform([x, y, z, roll_rad, pitch_rad, yaw_rad]))
          
          if first:
            first = False
            yield timestamp_ns_dest, T_O_Ik
          
          T_O_Ik = T_O_Ik * T_Ik_Ikp1
          yield timestamp_ns_source, T_O_Ik
