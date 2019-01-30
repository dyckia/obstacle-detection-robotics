
from a3_p2.msg import ExtractedLine
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from math import *
from angles import constrain_angle

def fit_line(scan, start_index, end_index, maximum_range):
    """ 
    Fit a line to the given LaserScan yielding an ExtractedLine.
    
    The method of weighted least squares is applied to yield a description of
    the line consisting of the tuple (r, alpha) where 'r' is the orthogonal
    distance of the line to the origin and 'alpha' is the angle that the ray
    perpendicular to the line makes with the origin.  Weighted least squares is
    applied as described in "Introduction to Autonomous Mobile Robots" by
    Siegwart, Nourbakhsh, and Scaramuzza.

    Arguments:
    start_index -- index of the first data point to be used in the line fitting
                   procedure.
    end_index -- index of the last data point to be used in the line fitting
                 procedure.
    maximum_range -- specifies the maximum range of data points which will be
                     incorporated into the fitted line.

    If the scan is empty or there are no points within 'maximum_range' then
    None is returned.
    """

    # First we must calculate alpha, using the formula presented in the
    # course notes (due to Arras, 1998).
    sumWeights = 0
    sumNumL = 0 # The summation term in the numerator on the left
    sumNumR = 0 # The summation term in the numerator on the right
    sumDenL = 0 # The summation term in the denominator on the left
    sumDenR = 0 # The summation term in the denominator on the rt.
    for i in range(start_index, end_index+1):

        rho = scan.ranges[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = scan.angle_min + i * scan.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sumWeights += weight

        factor1 = weight * rho * rho
        sumNumL += factor1 * sin(2 * theta)
        sumDenL += factor1 * cos(2 * theta)
        
        for j in range(start_index, end_index+1):
            rho_j = scan.ranges[j]
            if rho_j == 0 or rho_j > maximum_range:
                continue
            theta_j = scan.angle_min + j * scan.angle_increment
            weight_j = 1 / rho_j**2
            #weight_j = 1

            factor2 = weight * weight_j * rho * rho_j
            sumNumR += factor2 * cos(theta) * sin(theta_j)
            sumDenR += factor2 * cos(theta + theta_j)

    if sumWeights == 0:
        # There are either no scan points at all, or none within range.
        return None

    sumNumR *= 2.0 / sumWeights
    sumDenR /= sumWeights
    alpha = atan2(sumNumL - sumNumR, sumDenL - sumDenR) / 2.0 + pi/2

    # We now calculate r.
    sumNum = 0 # The summation term in the numerator
    for i in range(start_index, end_index+1):
        rho = scan.ranges[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = scan.angle_min + i * scan.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sumNum += weight * rho * cos(theta - alpha)

    r = sumNum / sumWeights

    # It is possible that the r value returned above is negative.  We formulate
    # r as a positive quantity, but the optimization process doesn't know about
    # this.  Having a negative r can create problems down the road (e.g. for
    # line-based wall following).  So we flip our parameters to make r positive.
    if r < 0:
        r *= -1
        alpha += pi

    # Make sure that alpha is in the range (-pi, pi].
    alpha = constrain_angle(alpha)

    # Determine the first and last points used to estimate this line's
    # parameters.  These two points do not define the line, but they are useful
    # for visualization to show the range of points involved.
    firstScanPoint = Point32()
    lastScanPoint = Point32()
    dist = scan.ranges[start_index]
    angle = scan.angle_min + start_index * scan.angle_increment
    if dist <= maximum_range:
        firstScanPoint.x = dist * cos(angle)
        firstScanPoint.y = dist * sin(angle)
    dist = scan.ranges[end_index]
    angle = scan.angle_min + end_index * scan.angle_increment
    if dist <= maximum_range:
        lastScanPoint.x = dist * cos(angle)
        lastScanPoint.y = dist * sin(angle)

    return ExtractedLine(r, alpha, firstScanPoint, lastScanPoint)
