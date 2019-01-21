#!/usr/bin/env python2

import argparse

import rosbag
import os
import shutil
import IPython

def processRosbag(input_bag_filepath, delete_input_bag):
    input_bag_path, input_bag_filename_with_ending = os.path.split(input_bag_filepath)
    input_bag_filename, ending = os.path.splitext(input_bag_filename_with_ending)

    output_bag_filepath = os.path.join(input_bag_path, input_bag_filename + '_ngh.bag')
    assert(input_bag_filepath != output_bag_filepath)

    if os.path.exists(output_bag_filepath):
      print "Skipping processing bag ", input_bag_filepath, " because the output bag ", output_bag_filepath, " already exists."
      return

    input_bag = rosbag.Bag(input_bag_filepath, 'r')

    print "Processing bag ", input_bag_filepath, "."

    types, topics = input_bag.get_type_and_topic_info()
    has_grasshoppers = False
    for topic in topics.iterkeys():
      if 'mono' in topic:
        has_grasshoppers = True
        break
    
    if not has_velodynes:
      print "Renaming bag ", input_bag_filepath, " which apparently does not have any grasshopper streams."
      shutil.move(input_bag_filepath, output_bag_filepath)
      return  

    output_bag = rosbag.Bag(output_bag_filepath, 'w')

    for topic, msg, t in input_bag.read_messages():
      if 'mono' in topic:
        continue

      output_bag.write(topic, msg, t)

    output_bag.close()

    if delete_input_bag:
      print "Deleting input bag ", input_bag_filepath, "."
      os.remove(input_bag_filepath)

def main():
    parser = argparse.ArgumentParser(
        description='Create a new rosbag by removing messages from a chosen '
                    'topic.')
    parser.add_argument('input_rosbag_or_folder')
    parser.add_argument('--delete_input_bag', default=False, action="store_true")

    parsed_args = parser.parse_args()

    input_rosbag_or_folder = parsed_args.input_rosbag_or_folder
    delete_input_bag = parsed_args.delete_input_bag

    if os.path.isfile(input_rosbag_or_folder):
      processRosbag(input_rosbag_or_folder, delete_input_bag=delete_input_bag)
    else:
      for root, dirs, files in os.walk(input_rosbag_or_folder):
        for filename_with_ending in files:
          filename, ending = os.path.splitext(filename_with_ending)
          if ending != ".bag":
            continue

          filepath = os.path.join(root, filename_with_ending)
          processRosbag(filepath, delete_input_bag=delete_input_bag)

if __name__ == '__main__':
    main()
