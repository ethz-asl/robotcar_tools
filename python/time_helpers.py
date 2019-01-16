import rospy

def nanoseconds_to_ros_timestamp(time_ns):
  timestamp_nanoseconds_str = str(time_ns)
  timestamp = rospy.Time(
    int(timestamp_nanoseconds_str[0:-9]), int(timestamp_nanoseconds_str[-9:]))
  return timestamp
