#!/usr/bin/env python
'''This file has implementation for fuzzy logic obstacle avoidance and right Edge following behaviour
using Fuzzy logic'''
# importing ros libraries related to sensor and geometry.
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

global behaviour_option
''' MemebershipFunction Class: It is used to detrmine the membership value of a given sensor value
with different membership functions.'''
class MembershipFunctions:

    def __init__(self, mfPoints, label):
        # [A,B,C,D]
        self.points = mfPoints
        self.linguistic_label = label

    '''This function calculates the memebership values of different sensors
    Parameters:
    [input-value]: Sensor value whose membership is to be determined'''
    def calculateMembershipValue(self, input_value):
        if input_value < self.points[0] or input_value > self.points[3]:
            return 0.0
        elif input_value >= self.points[0] and input_value < self.points[1]:
            return (input_value - self.points[0]) / (self.points[1] - self.points[0])
        elif input_value >= self.points[1] and input_value <= self.points[2]:
            return 1.0
        elif input_value > self.points[2] and input_value < self.points[3]:
            return (self.points[3] - input_value) / (self.points[3] - self.points[2])

        return 0.0

'''This class stores different fuzzy rule needed to calculate the firing strength'''
class Rule:
    ''' Object of this class is initialized with antecedents and consequent list, the rule is
    created with the instantiation of the class.
    Parameters:
    [antecedents]:list of antecedents, with index-0 1st sensor antecedent and index-1 with 2nd sensor value
    [consequents]:list of consequents, with index-0 as speed and index-1 as steer
    '''
    def __init__(self, antecedents, consequents):
        self.antecedents = antecedents
        self.consequents = consequents

    '''This Function calculates the firing strength fo each rules with respect two sensor membership of both 
        the sensors.
        Parameters:
        [membership_values_dictionary]: List of dictionary of membership function name as key and it's respective
        membership value for both the sensors.
        Return Value: returns the minimum of membership value of sensors as firing strength for that particular rule.'''
    def calculateFiringStrength(self, membership_values_dictionary):
        list_MemValues = []
        for i in range(len(self.antecedents)):
            list_MemValues.append(membership_values_dictionary[i][self.antecedents[i]])
        # returns the minimum of the two values as the firing strength of the rule.
        return min(list_MemValues)

'''This class stores all  the rules needed for fuzzy logic and also gives the crisp output for 
    speed and velocity.
'''
class RuleBase:
    '''An object of RuleBase class is initiated with list of rules objects passed to it.
    Parameters: 
    [rules]: List of objects of class Rules'''
    def __init__(self, rules):
        self.rules = rules

    ''' This function gives the crisp output
        Parameters:
        [rule_firingStrength]: list of firingstrength of different rules
        [rule_speed]:           List of speed values w.r.t to firing strength
        [rule_steer]:          List of steer values w.r.t to firing strength
    '''
    def getCrispOutput(self, rule_firingStrength, rule_speed, rule_steer):
        speed = steer = firing_strength_sum = 0.0
        firing_strength_length = len(rule_firingStrength)
        ''' At the begining when the Robot is stationary, firing strength of 0 is 
        obtained, assigning a default velocity of 0.5 so that the robot starts moving, '''
        if firing_strength_length == 0:
            speed = 0.5
            steer = 0.0
            return speed, steer
        # if firing strenght is a list of one element, converting the list into float for calculation later.
        elif firing_strength_length == 1:
            firing_strength_sum = rule_firingStrength[0]
        else:
            # calculating sum of firing strength, could have also used sum(rule_firingStrength)
            for i in rule_firingStrength:
                firing_strength_sum += i

        print("firing_strength_sum:", firing_strength_sum)
        # Calculating product of firing strength and speed
        for firingStrength, velocity in zip(rule_firingStrength, rule_speed):
            speed += (firingStrength * velocity)
        print(speed)

        # Calculating product of firing strength and steer
        for firingStrength, turn in zip(rule_firingStrength, rule_steer):
            steer += (firingStrength * turn)

        print(steer)
        speed = speed / firing_strength_sum
        steer = steer / firing_strength_sum
        # returning crisp speed and turn values 
        return speed, steer

'''This is the Parent class which implements fuzzy logic.
    It calculates membership value different sensors based on the their sensor reading.
'''
class FuzzySet(object):
    def __init__(self):
        # Righ front sensor in  REF and top left sensor in OA
        self.sensor_1_dist = 0.0
        # Right back sensor in  REF and top right sensor in OA
        self.sensor_2_dist = 0.0
        self.front_sensor_dist = 0.0
        self.rule_base = []
        self.sensor_1_Membership = []
        self.sensor_2_Membership = []
        self.classspeedCentreSet = []
        self.classsteerCentreSet = []

        #output centre of set for speed
        self.speedCentreSet = {"Slow": 0.1, "Medium": 0.25, "Fast": 0.4}
        #output centre of set for steer
        self.steerCentreSet = {"Left": 0.8, "fwd": 0, "Right": -0.5}

        # Membership function for both obstacle avoidance and Right Edge Following.
        self.class_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "OA"),
                                 MembershipFunctions([1.0, 1.5, 2.0, 3], "REF")]
        self.rule_base = self.createRules()

    def setSensorValues(self, sensor_1, sensor_2):
        self.sensor_1_dist = sensor_1
        self.sensor_2_dist = sensor_2

    def getSensorValues(self):
        return self.sensor_1_dist, self.sensor_2_dist
    ''' This function create object rules and uses it to create Rulebase object.
        Return Value: obect of RuleBase class'''
    def createRules(self):
        r_base = []
        rule_1 = Rule(["OA"], ["Speed_OA", "Steer_OA"])
        rule_2 = Rule(["REF"], ["Speed_REF", "Steer_REF"])
        rule_base = RuleBase([rule_1, rule_2])
        return rule_base

    # Calculates membership value of obstacle avoidance and right edge following w.r.t to fron sensor.
    # Return Value: list of dictionary.
    def class_fuzzification(self):
        front_sensor_dict = {"OA": self.class_Membership[0].calculateMembershipValue(self.front_sensor_dist),
                             "REF": self.class_Membership[1].calculateMembershipValue(self.front_sensor_dist)}
        return [front_sensor_dict]

    # Calculates membership value of sensor 1 and sensor 2 w.r.t to respective membership functions.
    # This function is called both for obstacle avoidance and right edge following.
    # Return Value: list of dictionary.
    def fuzzification(self):
        sensor_1_dict = {"Near": self.sensor_1_Membership[0].calculateMembershipValue(self.sensor_1_dist),
                         "Medium": self.sensor_1_Membership[1].calculateMembershipValue(self.sensor_1_dist),
                         "Far": self.sensor_1_Membership[2].calculateMembershipValue(self.sensor_1_dist)}
        print("sensor_1_dict", sensor_1_dict)
        sensor_2_dict = {"Near": self.sensor_2_Membership[0].calculateMembershipValue(self.sensor_2_dist),
                         "Medium": self.sensor_2_Membership[1].calculateMembershipValue(self.sensor_2_dist),
                         "Far": self.sensor_2_Membership[2].calculateMembershipValue(self.sensor_2_dist)}
        print("sensor_2_dict", sensor_2_dict)

        return [sensor_1_dict, sensor_2_dict]

    # Calculates firing strength, gets the crisp output..
    # This function is called is only called when doing defuzzification between OA and REF.
    # Return Value: list of dictionary.
    def class_defuzzification (self, sensor_membership_values):
        rule_speed = []
        rule_steer = []
        rule_firingStrength = []
        print("fuzzification Values: ", sensor_membership_values)
        # Iterating over RuleBase object to get the firing strength of each rule stored in it,
        for rule in self.rule_base.rules:
            fs = rule.calculateFiringStrength(sensor_membership_values)
            # only store the firing strength of rule and its related consequents if the firing
            # strength of the rule is greater than 0.
            if fs > 0:
                rule_firingStrength.append(fs)
                print("speed set", rule.consequents[0])
                print("steer set", rule.consequents[1])
                print("fs set",rule_firingStrength)
                # storing and appending both the consequents of rules in two different lists        
                rule_speed.append(self.classspeedCentreSet[rule.consequents[0]])
                rule_steer.append(self.classsteerCentreSet[rule.consequents[1]])
        print("Firing Strength: ", rule_firingStrength)
        print("Rule Speed:", rule_speed)
        print("Rule Steer: ", rule_steer)
        # Gets crisp output for both steer and speed
        speed, steer = self.rule_base.getCrispOutput(rule_firingStrength, rule_speed, rule_steer)
        return speed, steer

    # This function is called for both OA and REF
    # Calculates firing strength, gets the crisp output..
    # This function is called is only called when doing defuzzification between OA and REF.
    # Return Value: list of dictionary.
    def defuzzification(self, membership_values_dictionary):
        rule_speed = []
        rule_steer = []
        rule_firingStrength = []
        print("fuzzification Values: ", membership_values_dictionary)
        # Iterating over RuleBase object to get the firing strength of each rule stored in it,
        for rule in self.rule_base.rules:
            fs = rule.calculateFiringStrength(membership_values_dictionary)
            # only store the firing strength of rule and its related consequents if the firing
            # strength of the rule is greater than 0.
            if fs > 0:
                rule_firingStrength.append(fs)
                print("speed set", rule.consequents[0])
                print("steer set", rule.consequents[1])
                print("fs set", rule_firingStrength)
                # storing and appending both the consequents of rules in two different lists        
                rule_speed.append(self.speedCentreSet[rule.consequents[0]])
                rule_steer.append(self.steerCentreSet[rule.consequents[1]])
        print("Firing Strength: ", rule_firingStrength)
        print("Rule Speed:", rule_speed)
        print("Rule Steer: ", rule_steer)
        # Gets crisp output for both steer and speed
        speed, steer = self.rule_base.getCrispOutput(rule_firingStrength, rule_speed, rule_steer)
        return speed, steer

    '''This is the child class derived from FuzzySet clsass for Right Edge Following
    It reuses all the common functions from the prent class'''
class FuzzySetRef(FuzzySet):
    def __init__(self):
        super(FuzzySetRef, self).__init__()
        #Memebership function for Right Edge following for both the sensors
        self.sensor_1_Membership = [MembershipFunctions([0.1, 0.1, 0.2, 0.5], "Near"),
                                    MembershipFunctions([0.2, 0.5, 0.5, 0.7], "Medium"),
                                    MembershipFunctions([0.5, 0.7, 0.9, 2.0], "Far")]
        self.sensor_2_Membership = [MembershipFunctions([0.1, 0.1, 0.2, 0.5], "Near"),
                                    MembershipFunctions([0.2, 0.5, 0.5, 0.7], "Medium"),
                                    MembershipFunctions([0.5, 0.7, 0.9, 2.0], "Far")]
        self.rule_base = self.createRules()
        # Rules for Right Edge following
    def createRules(self):
        rule_base = []
        rule_1 = Rule(["Near", "Near"], ["Slow", "Left"])
        rule_2 = Rule(["Near", "Medium"], ["Slow", "Left"])
        rule_3 = Rule(["Near", "Far"], ["Slow", "Left"])
        rule_4 = Rule(["Medium", "Near"], ["Slow", "Right"])
        rule_5 = Rule(["Medium", "Medium"], ["Medium", "fwd"])
        rule_6 = Rule(["Medium", "Far"], ["Medium", "Left"])
        rule_7 = Rule(["Far", "Near"], ["Slow", "Right"])
        rule_8 = Rule(["Far", "Medium"], ["Medium", "Right"])
        rule_9 = Rule(["Far", "Far"], ["Fast", "Right"])
        rule_base = RuleBase([rule_1, rule_2, rule_3, rule_4, rule_5, rule_6, rule_7, rule_8, rule_9])
        return rule_base

'''This is the child class derived from FuzzySet clsass for Right Edge Following
    It reuses all the common functions from the prent class'''
class FuzzySetOA(FuzzySet):
    def __init__(self):
        super(FuzzySetOA, self).__init__()
        #Memebership function for obstacle avoidance for both the sensors

        self.sensor_1_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "Near"),
                                    MembershipFunctions([1.0, 1.5, 1.5, 2.0], "Medium"),
                                    MembershipFunctions([1.5, 2.0, 2.5, 3.0], "Far")]
        self.sensor_2_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "Near"),
                                    MembershipFunctions([1.0, 1.5, 1.5, 2.0], "Medium"),
                                    MembershipFunctions([1.5, 2.0, 2.5, 3.0], "Far")]

        self.rule_base = self.createRules()

        # Rules for obstacle avoidance
    def createRules(self):
        rule_base = []
        rule_1 = Rule(["Near", "Near"], ["Slow", "Left"])
        rule_2 = Rule(["Near", "Medium"], ["Slow", "Left"])
        rule_3 = Rule(["Near", "Far"], ["Slow", "Left"])
        rule_4 = Rule(["Medium", "Near"], ["Slow", "Left"])
        rule_5 = Rule(["Medium", "Medium"], ["Slow", "fwd"])
        rule_6 = Rule(["Medium", "Far"], ["Medium", "fwd"])
        rule_7 = Rule(["Far", "Near"], ["Slow", "Left"])
        rule_8 = Rule(["Far", "Medium"], ["Medium", "fwd"])
        rule_9 = Rule(["Far", "Far"], ["Fast", "fwd"])
        rule_base = RuleBase([rule_1, rule_2, rule_3, rule_4, rule_5, rule_6, rule_7, rule_8, rule_9])

        return rule_base
# Declaring publisher to velocity and angular velocity to topic 'vmd_vel'
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # creates publisher class

# function to control velocity  and turn of the robo robot
def forwards(speed, turn):
    global pub
    print("Robot Speed: ", speed)
    print("Robot Turn: ", turn)
    vel_x = Twist()  # initiates twist message
    vel_x.linear.x = speed  # sets linear speed of robot
    vel_x.angular.z = turn  # sets angle to turn to pid function output (turns left?)
    pub.publish(vel_x)  # publishes velocity

#This function is called with new sensor values.
def callback(msg, fuzzy):
    # sensor ranges for REF
    frontRightSensor = min(msg.ranges[540:629])  # GET THE RANGES VALUE YOU WANT
    backRightSensor = min(msg.ranges[449:539])
    # Sensor ranges for OA
    topLeftSensor = min(msg.ranges[0:10])  # GET THE RANGES VALUE YOU WANT
    topRightSensor = min(msg.ranges[625:635])
    # Sensor value for control architecture
    min_top_dist = min(topLeftSensor, topRightSensor)

    # If the sensor value is 'inf' or greater than the memebership function's
    # range, then hard-code it a value so that it falls under it gets 'FAR' membership functios
    # membership
    if str(frontRightSensor) == 'inf' or frontRightSensor >= 2:
        frontRightSensor = 1.99
    if str(backRightSensor) == 'inf' or backRightSensor >= 2:
        backRightSensor = 1.99
    if str(topLeftSensor) == 'inf' or topLeftSensor >= 3:
        topLeftSensor = 2.99
    if str(topRightSensor) == 'inf' or topRightSensor >= 3:
        topRightSensor = 2.99
    if str(min_top_dist) == 'inf' or min_top_dist >= 3:
        min_top_dist = 2.99
    fuzzy[0].front_sensor_dist = min_top_dist
    fuzzy[1].setSensorValues(frontRightSensor, backRightSensor)
    fuzzy[2].setSensorValues(topLeftSensor, topRightSensor)

# This is the function which runs continuosly  and does the operations based on sensor values.
def controller(fuzzy):
    global behaviour_option
    rate = rospy.Rate(10)  # sleep in loop at rate 10hz
    while not rospy.is_shutdown():
        #Control Architecture
        if behaviour_option == 0:
            print("BOTH")

            print("Right Front Sensor Readings", fuzzy[1].sensor_1_dist)
            print("Right Back Sensor Readings", fuzzy[1].sensor_2_dist)

            print("Top Left Sensor Readings", fuzzy[2].sensor_1_dist)
            print("Top Right Sensor Readings", fuzzy[2].sensor_2_dist)

            # Getting membership for REF for both sensor
            membership_values_dictionary = fuzzy[1].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_ref, steer_ref = fuzzy[1].defuzzification(membership_values_dictionary)
            # Getting membership for AO for both sensor
            membership_values_dictionary = fuzzy[2].fuzzification()
            # Getting Crisp output for both speed and steer     
            speed_oa, steer_oa = fuzzy[2].defuzzification(membership_values_dictionary)
            # Creating dynamic dictionary with REF speed, steer and OA speed,steer
            fuzzy[0].classspeedCentreSet = {"Speed_OA": speed_oa, "Speed_REF": speed_ref}
            fuzzy[0].classsteerCentreSet = {"Steer_OA": steer_oa, "Steer_REF": steer_ref}
            # Getting membership value for OA and REF
            membership_values_dictionary = fuzzy[0].class_fuzzification()
             # Getting Crisp output for both speed and steer     
            speed, steer = fuzzy[0].class_defuzzification(membership_values_dictionary)
            print("****speed****:", speed)
            print("****steer****:", steer)
             # Passing speed and steer to drive the robot
            forwards(speed, steer)
        #REF    
        if behaviour_option == 1:
            print("REF")
            print("Right Front Sensor Readings", fuzzy[1].sensor_1_dist)
            print("Right Back Sensor Readings", fuzzy[1].sensor_2_dist)
            # Getting membership for REF for both sensor
            membership_values_dictionary = fuzzy[1].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_ref, steer_ref = fuzzy[1].defuzzification(membership_values_dictionary)
            print("****speed****:", speed_ref)
            print("****steer****:", steer_ref)
             # Passing speed and steer to drive the robot
            forwards(speed_ref, steer_ref)
        #Obstacle Avoidance
        if behaviour_option == 2:
            print("OA")
            print("Top Left Sensor Readings", fuzzy[2].sensor_1_dist)
            print("Top Right Sensor Readings", fuzzy[2].sensor_2_dist)
            # Getting membership for AO for both sensor
            membership_values_dictionary = fuzzy[2].fuzzification()
            # Getting Crisp output for both speed and steer     
            speed_oa, steer_oa = fuzzy[2].defuzzification(membership_values_dictionary)
            print("****speed****:", speed_oa)
            print("****steer****:", steer_oa)
             # Passing speed and steer to drive the robot
            forwards(speed_oa, steer_oa)
        rate.sleep()  # pauses rate


if __name__ == '__main__':
    global behaviour_option
    # Reading options passed through scrip to select the behaviour
    if str(sys.argv[1]) == 'Both':
        behaviour_option = 0
    if str(sys.argv[1]) == 'REF':
        behaviour_option = 1
    if str(sys.argv[1]) == 'OA':
        behaviour_option = 2


    # creating main class object
    fuzzySuper = FuzzySet()

    # Creating Fuzzy object for Righ Edge Following
    fuzzyRef = FuzzySetRef()

    # # Creating Fuzzy object for Obstacle avoidance3
    fuzzyOA = FuzzySetOA()
    # creating list of fuzzy objects
    fuzzy = [fuzzySuper, fuzzyRef, fuzzyOA]
    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback, fuzzy)
        controller(fuzzy)
    except rospy.ROSInterruptException:
        pass
