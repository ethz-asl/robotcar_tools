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

extrinsics_folder = '/home/mbuerki/Downloads/robotcar-dataset-sdk-2.1/extrinsics'

def main():
  ins_filepath = os.path.join(extrinsics_folder, 'ins.txt')
  mono_left_filepath = os.path.join(extrinsics_folder, 'mono_left.txt')
  mono_rear_filepath = os.path.join(extrinsics_folder, 'mono_rear.txt')
  mono_right_filepath = os.path.join(extrinsics_folder, 'mono_right.txt')

  T_B_RTK = None
  with open(ins_filepath) as extrinsics_file:
      extrinsics = [float(x) for x in next(extrinsics_file).split(' ')]
      T_B_RTK = get_R_ENU_NED() * mk.Transformation(build_se3_transform(extrinsics))

  T_B_MF = get_R_ENU_NED() * get_T_B_I()

  T_B_ML = None
  with open(mono_left_filepath) as extrinsics_file:
      extrinsics = [float(x) for x in next(extrinsics_file).split(' ')]
      T_B_ML = get_R_ENU_NED() * mk.Transformation(build_se3_transform(extrinsics)).inverse() * get_T_B_I()

  T_B_MRe = None
  with open(mono_rear_filepath) as extrinsics_file:
      extrinsics = [float(x) for x in next(extrinsics_file).split(' ')]
      T_B_MRe = get_R_ENU_NED() * mk.Transformation(build_se3_transform(extrinsics)).inverse() * get_T_B_I()

  T_B_MRi = None
  with open(mono_right_filepath) as extrinsics_file:
      extrinsics = [float(x) for x in next(extrinsics_file).split(' ')]
      T_B_MRi = get_R_ENU_NED() * mk.Transformation(build_se3_transform(extrinsics)).inverse() * get_T_B_I()

  IPython.embed()
  

if __name__ == '__main__':
    main()
