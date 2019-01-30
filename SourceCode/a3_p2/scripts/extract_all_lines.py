#!/usr/bin/env python

import rospy
from math import cos
from fit_line import fit_line
from sensor_msgs.msg import LaserScan
from a3_p2.msg import ExtractedLine
from a3_p2.msg import ExtractedLines


def find_max_dist(scan, line, start_index, end_index):
    """
    Compute the maximum distance between the points and the fitted line.
    Return the maximum distance and the index of the data point.
    """
    max_dist = -1
    max_dist_index = -1
    r = line.r
    alpha = line.alpha
    for i in range(start_index, end_index+1):
        rho = scan.ranges[i]
        # if the range is 0 or larger than maximum range, discard the data point
        if rho == 0 or rho > max_range:
            continue
        theta = scan.angle_min + i * scan.angle_increment
        dist = rho * cos(theta - alpha) - r
        if dist > max_dist:
            max_dist = dist
            max_dist_index = i
    return max_dist, max_dist_index


def split(scan):
    """
    Function split() always checks the first item of line_list and see if it
    is splitable. If so then it will split the list and create two sub lists.
    If not it will append the line to lines and then delete it from the
    line_list.
    """
    n = len(scan.ranges)
    line_list = [[0, n-1]] # a list instance whose each item represents the [start_data_index, end_data_index] of a line

    lines = ExtractedLines()
    lines.header.frame_id = '/base_laser_link'

    # keep splitting while line_list is not empty
    while len(line_list) > 0:
        # find the fitted line of the first item in line_list
        # line is an ExtractedLine(r, alpha, firstScanPoint, lastScanPoint) instance
        line = fit_line(scan, line_list[0][0], line_list[0][1], max_range)
        # return the max_dist and its index
        max_dist, max_dist_index = find_max_dist(scan, line, line_list[0][0], line_list[0][1])
        # if max_dist is larger than or equal to the threshold, split the points
        if max_dist >= dist_threshold:
            # split only if the number of points in the splitted subset is larger than the threshold
            if (max_dist_index - line_list[0][0] + 1) >= min_points and \
               (line_list[0][1] - max_dist_index + 1) >= min_points:
                # insert [max_dist_index, end_index]
                line_list.insert(0, [max_dist_index, line_list[0][1]])
                # note that the original first item now becomes the second item
                line_list.insert(0, [line_list[1][0], max_dist_index])
                # delete the original first item in line_list, which now
                # becomes the third item
                line_list.pop(2)
            # if the number of points after splitting are two small, then
            # do not split and just append the line to lines
            else:
                lines.lines.append(line)
                # delete the first item in line_list
                line_list.pop(0)
        # if max_dist is less than the threshold, append the line to lines
        else:
            lines.lines.append(line)
            line_list.pop(0)
    return lines


def extract_lines(scan):
    lines = split(scan)
    extracted_pub.publish(lines)


if __name__ == '__main__':
    # declare some global variables
    global extracted_pub, max_range, dist_threshold, min_points
    max_range = rospy.get_param('maximum_range')
    dist_threshold = rospy.get_param('orthog_distance_threshold')
    min_points = rospy.get_param('min_points_per_line')

    # initialize the node
    rospy.init_node('extract_all_lines')

    # create a Publiser instance
    extracted_pub = rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=1)

    # subscribe to /base_scan
    rospy.Subscriber('/base_scan', LaserScan, extract_lines)

    # keep the node running
    rospy.spin()
