# This file is based on:
# - https://github.com/wg-perception/people/blob/noetic/people_msgs/msg/Person.msg
# - https://github.com/wg-perception/people/blob/noetic/people_msgs/msg/PositionMeasurement.msg
# - https://github.com/spencer-project/spencer_people_tracking/blob/master/messages/spencer_tracking_msgs/msg/DetectedPerson.msg

# standard ROS header file
std_msgs/Header header
# Unique id of the detection, monotonically increasing over time
uint64 detection_id
# The name of the detector that detected the person (i.e frontalface, profileface)
string name
# The name of the object
string object_id
# Pose of the person (position + orientation) with covariance
geometry_msgs/PoseWithCovariance pose
# Velocity of the person (6 DOF)
geometry_msgs/Pose velocity
# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the confidence in the detection
float64 reliability
# Additional fields, handy for tagging groups etc.
string[] tagnames
string[] tags
byte initialization
