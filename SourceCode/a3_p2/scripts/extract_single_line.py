#!/usr/bin/env python  

"""
ROS node which subscribes to the /scan topic and fits a single line to
this scan, publishing it on the /extracted_lines topic.
"""

import rospy
import math
from fit_line import fit_line

from sensor_msgs.msg import LaserScan
from a3_p2.msg import ExtractedLine
from a3_p2.msg import ExtractedLines

def scan_callback(scan):
    """
    Fit a single line to the given laser scan.

    The line is fitted using scan points that are within 'maximum_range'
    distance (this parameter should be found by the ROS parameter server).  An
    ExtractedLines message is constructed to hold this single line and this
    message is published on /extracted_lines.
    """

    # The fit_line method expects the scan itself, a pair of integers
    # describing the indices upon which it will operate, and the max. range.
    n = len(scan.ranges)
    line = fit_line(scan, 0, n-1, maximum_range)

    lines = ExtractedLines()
    lines.header.frame_id = '/base_laser_link'

    if line is not None:
        lines.lines.append(line)

    extracted_publisher.publish(lines)

if __name__ == '__main__':
    global maximum_range, extracted_publisher

    # Initialize this node.
    rospy.init_node('extract_single_line')

    maximum_range = rospy.get_param('maximum_range')

    # Subscribe to /base_scan
    rospy.Subscriber('/base_scan', LaserScan, scan_callback)

    extracted_publisher = rospy.Publisher('/extracted_lines', ExtractedLines, \
                                          queue_size=1)

    rospy.spin()
