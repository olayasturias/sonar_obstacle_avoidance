#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Vision_based_Navigation_TTT.msg import OpticalFlow
from Vision_based_Navigation_TTT.msg import TauComputation
from sensor_msgs.msg import Image, LaserScan
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
import cv2

class SonarAvoidanceComputationClass:

    def __init__(self, sonar_topic = '/rexrov2/sss_sonar'):

        self.n_ROIs = 5
        self.Kf = 20
        self.Km = 5
        self.sonar_sub_name = sonar_topic
        self.velocity_command_topic_name = '/rexrov2/cmd_vel'
        self.ranges = np.empty(self.n_ROIs)

        # Sonar Subscriber
        self.sonar_sub = rospy.Subscriber(self.sonar_sub_name, LaserScan, self.callback_sonar)
        # Tau Computation message Publisher
        self.tau_values = rospy.Publisher("tau_values", TauComputation, queue_size=10)
        # Velocity publisher
        self.steering_signal = rospy.Publisher(self.velocity_command_topic_name, Twist, queue_size=10)

    # Callback for the sonar topic
    def callback_sonar(self, sonar_data):
        rospy.logdebug_once("Scan starts in angle {:.2f} , and ends in angle {:.2f}"
               .format(sonar_data.angle_min*(180/np.pi), sonar_data.angle_max*(180/np.pi)))
        measurement_range = sonar_data.angle_max*(180/np.pi)-sonar_data.angle_min*(180/np.pi)

        rospy.logdebug_once('Total sonar range: {:.2f} deg'.format(measurement_range))

        ROI_n_values = int(measurement_range/self.n_ROIs)

        rospy.logdebug_once('ROI angle: {:.2f} deg'.format(ROI_n_values))
        # Reshape array to have one row per ROI
        # and one colum per ROI values
        roi_ranges = np.reshape(sonar_data.ranges,(self.n_ROIs,ROI_n_values))
        # compute average and std for each ROI
        roi_avg_ranges = np.amin(roi_ranges,axis = 1)
        self.ranges = roi_avg_ranges
        roi_avg_std = np.std(roi_ranges, axis=1)
        rospy.logdebug('ROI average values %s',roi_avg_ranges)
        rospy.logdebug('ROI stds %s',roi_avg_std)

        

    def there_is_obstacle(self, range, min_dist_to_obstacle):        
        if range <= min_dist_to_obstacle:
            obstacle = True
        else:
            obstacle = False
        return obstacle

    def set_velocity_values(self, linear_x_vel):
        print(self.ranges)
        norm_ranges = self.ranges/np.amin(self.ranges)
        #norm_ranges = (self.ranges - np.min(self.ranges))/np.ptp(self.ranges)
        print(norm_ranges)
        
        if self.there_is_obstacle(self.ranges[2],7):
            rospy.logwarn('obstacle detected, time to turn!')
            steering_velocity = (1/np.amin(self.ranges))*self.Kf*(norm_ranges[4]-norm_ranges[0])+self.Km*(norm_ranges[3]-norm_ranges[1])
            rospy.logwarn('steering velocity %s', steering_velocity)
            # Turn
            msg = Twist()
            msg.linear.x = float(linear_x_vel*self.ranges[2]/7)
            msg.angular.z = float(steering_velocity)
            self.steering_signal.publish(msg)
        else:
            rospy.logwarn('Distance to front obstacle %s, move forward!', self.ranges[2])
            # Go straight
            msg = Twist()
            msg.linear.x = float(linear_x_vel)
            msg.angular.z = float(0)
            self.steering_signal.publish(msg)

        



def sonar_avoid():
    rospy.init_node("naive_avoidance", anonymous=False, log_level=rospy.DEBUG)
    avoid_boi = SonarAvoidanceComputationClass(sonar_topic = '/rexrov2/sss_sonar')

    rate = rospy.Rate(8) # 8hz
    while not rospy.is_shutdown():
        avoid_boi.set_velocity_values(0.9)
        rate.sleep()

if __name__ == '__main__':
    sonar_avoid()