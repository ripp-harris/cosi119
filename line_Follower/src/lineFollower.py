#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math

def image_callback(msg):
    image = bridge.imgmsg_to_cv2(msg) # Take image from camera

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Convert to hsv
    lowYel = numpy.array([ 40, 0, 0]) # Lower bound for yellow
    upYel = numpy.array([120, 255, 255]) # Upper bound of yellow
    mask = cv2.inRange(hsv, lowYel, upYel) # Create mask
    masked = cv2.bitwise_and(image, image, mask=mask) # Create masked

# Crop image and leave just a band
    h, w, d = image.shape
    topSearch = 3*h/4
    botSearch = topSearch + 20
    mask[0:topSearch, 0:w] = 0
    mask[botSearch:h, 0:w] = 0

# Creating centroid to follow and overlaying on image
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

    # P controller
        global err 
        err = cx - w/2
        t.linear.x = 0.3
        if (err > 0):
            t.angular.z = -.4
        elif (err <0):
            t.angular.z = .4
        cmd_vel.publish(t)

# Display images
    cv2.imshow("Image with Centroid", image)
    cv2.waitKey(3)

# Unused
def calc_pid():
    global currError, prevError, sumError, pid

    prevError = currError
    currError = err
    sumError = currError * dT

    pid = getPid(currError, prevError, sumError)

    return pid

# Unused
def getPid(curr, prev, sum):
    pComponent = curr
    dComponent = (curr - prev)/dT
    iComponent = sum

    return propConstant * pComponent + derConstant * dComponent + intConstant * iComponent




rospy.init_node('line_Follower_node')
imgSub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
bridge = cv_bridge.CvBridge()
cv2.namedWindow("window", 1)
t = Twist()

# Unused PID related variables
propConstant = 0.8 # Proportional constant
intConstant = 0.01 # Integral constant
derConstant = 0.2 # Derivative constant

dT = 1
currError = 0
prevError = 0
sumError = 0
pid = 0

# Unused but attempted to create PID

# rospy.sleep(1)
# while not rospy.is_shutdown():
#     pid = calc_pid()
#     t.linear.x = .18
#     t.angular.z = ((math.pi)/6) * pid
#     cmd_vel.publish(t)




rospy.spin()