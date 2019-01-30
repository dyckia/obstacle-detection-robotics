#!/usr/bin/env python  

"""
ROS Node which displays lines extracted from laser range scans.  The node
subscribes to 'lines_topic' and publishes the data needed to display these
lines to 'vis_lines_topic' and correspondingly coloured first and last scan points used to generate each line to 'vis_scanpoints_topic'.  RViz must be
configured appropriately to display the messages posted to these topics.
"""

import rospy
from math import *

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from a3_p2.msg import ExtractedLine
from a3_p2.msg import ExtractedLines

def display_callback(lines):
    """Callback for extracted lines, displayed by publishing to
    vis_lines_topic and vis_scanpoints_topic."""

    create_lines_marker(lines)
    #create_scanpoints_marker(lines)

def create_lines_marker(lines):
    marker = Marker()

    # Get the pairs of points that comprise the endpoints for all lines
    marker.points = generate_endpoints(lines)

    # Initialize basic fields of the marker
    marker.header.frame_id = lines.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    # The marker's pose is at (0,0,0) with no rotation.
    marker.pose.orientation.w = 1

    # Set line width
    marker.scale.x = 0.05

    n = len(lines.lines)
    if n == 1:
        # There is only one line.  Just set the color field.
        marker.color.r = 1.0
        marker.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker.colors.append(color)
            marker.colors.append(color)

    lines_publisher.publish(marker)

def create_scanpoints_marker(lines):
    marker = Marker()

    # Initialize basic fields of the marker
    marker.header.frame_id = lines.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # The marker's pose is at (0,0,0) with no rotation.
    marker.pose.orientation.w = 1

    # Set point width
    marker.scale.x = 0.10
    marker.scale.y = 0.10

    # Add the scan points
    marker.points = []
    n = len(lines.lines)
    for i in range(n):
        marker.points.append(lines.lines[i].firstScanPoint)
        marker.points.append(lines.lines[i].lastScanPoint)

    if n == 1:
        # There is only one line.  Just set the color field.
        marker.color.r = 1.0
        marker.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker.colors.append(color)
            marker.colors.append(color)

    scanpoints_publisher.publish(marker)

def generate_endpoints(lines):
    """
    Generate two endpoints for each given line.

    Returns a list of point pairs to represent each of the given lines.  
    The points are positioned at a fixed distance 'q' from the point on the
    line closest to the origin (where the orthogonal ray hits the line).
    """

    # Distance from the closest point on the line to the origin at which to
    # place the endpoints.
    q = 5

    # The list of points
    points = []

    n = len(lines.lines)
    for i in range(n):
        r = lines.lines[i].r
        alpha = lines.lines[i].alpha

        # Each point is a vector sum from the origin to the point on the line
        # closest to the origin + a vector along the line in either direction.
        point1 = Point(r*cos(alpha) + q*cos(alpha+pi/2), \
                       r*sin(alpha) + q*sin(alpha+pi/2), \
                       0)
        point2 = Point(r*cos(alpha) + q*cos(alpha-pi/2), \
                       r*sin(alpha) + q*sin(alpha-pi/2), \
                       0)
        points.append(point1)
        points.append(point2)

    return points

if __name__ == '__main__':
    global lines_publisher#, scanpoints_publisher
    global lines_topic, vis_lines_topic#, vis_scanpoints_topic

    # Initialize this node.
    rospy.init_node('display_lines')

    lines_topic = rospy.get_param('~lines_topic')
    vis_lines_topic = rospy.get_param('~vis_lines_topic')
#    vis_scanpoints_topic = rospy.get_param('~vis_scanpoints_topic')

    # Subscribe to 'lines_topic'
    rospy.Subscriber(lines_topic, ExtractedLines, display_callback)

    # We will publish to vis_lines_topic which will show the lines (as
    # line segments) in RViz.  We will also publish the first and last scan
    # points to topic vis_scanpoints_topic in the same colour so that it
    # is clear from what range of scan points the lines are generated.
    lines_publisher = rospy.Publisher(vis_lines_topic, Marker, queue_size=1)
    #scanpoints_publisher = rospy.Publisher(vis_scanpoints_topic, Marker, \
    #                                       queue_size=1)

    rospy.spin()
