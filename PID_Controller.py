#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# Declaring error as global to keep the previous values and
# sensor reading as global to use it across functions.

integralError = []
previousError = 0
sensorReading = 0

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # creates publisher class


# function to control velocity of robot
def forwards(speed, turn):
    global pub
    vel_x = Twist()  # initiates twist message
    vel_x.linear.x = speed  # sets linear speed of robot
    vel_x.angular.z = turn * 1.0  # sets angle to turn to pid function output (turns left?)
    pub.publish(vel_x)  # publishes velocity


def pid(sensor_value):
    global integralError  # brings e_i list within function scope
    global previousError
    # parameters to tune to get a smooth output
    kP = 1
    kD = 22
    kI = 0.2
    #Default behaviour if no wall is found with in 3mt of right sensors
    if (sensor_value > 3):
        print("Distance is high turning right")
        return -0.5
    # Calculating proportionalError
    proportionalError = derivativeError = pidValue = 0.0
    # Distance from the wall at which car should follow the wall
    requiredDistance = 0.7
    print("############################")
    print("Distance to Wall = ", sensor_value)
    print("############################")

    proportionalError = requiredDistance - sensor_value
    print("Proportional Error = ", proportionalError)
    # only storing last 10 integral error and taking the sum
    if len(integralError) > 9:
        integralError.pop(0)
    integralError.append(proportionalError)
    print("Intergral len", len(integralError))
    totalIe = sum(integralError)
    print("Integral Error = ", totalIe)
    derivativeError = proportionalError - previousError
    print("Derivative Error = ", derivativeError)
    previousError = proportionalError
    #Calculating the Pid_value, which is  the angular velocity of the robot
    pid_value = proportionalError * kP + totalIe * kI + derivativeError * kD
    print("PID = ", pid_value)
    return pid_value

# this function is called with sensor readings
def callback(msg):
    global sensorReading
    #only getting the minimum right front sensor reading
    sensorReading = min(msg.ranges[539:639])  # GET THE RANGES VALUE YOU WANT


def controller():
    global sensorReading
    rate = rospy.Rate(10)  # sleep in loop at rate 10hz
    base_speed = 0.3
    while not rospy.is_shutdown():
        # Getting the value to control angular velociry of the robot
        pid_value = pid(sensorReading)
        forwards(base_speed, pid_value)
        rate.sleep()  # pauses rate


if __name__ == '__main__':
    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        controller()
    except rospy.ROSInterruptException:
        pass
