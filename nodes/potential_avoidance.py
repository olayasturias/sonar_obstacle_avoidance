#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Vision_based_Navigation_TTT.msg import OpticalFlow
from Vision_based_Navigation_TTT.msg import TauComputation
from sensor_msgs.msg import Image, LaserScan
import numpy as np
from cv_bridge import CvBridgeError, CvBridge
import cv2

class SonarTauComputationClass:

    def __init__(self, sonar_topic = '/rexrov2/sss_sonar'):

        self.sum_potentials_x = 0
        self.sum_potentials_y = 0
        self.Kp = .9#0.05
        self.Kphi = 0.35
        self.sonar_sub_name = sonar_topic
        self.velocity_command_topic_name = '/rexrov2/cmd_vel'


        # Sonar Subscriber
        self.sonar_sub = rospy.Subscriber(self.sonar_sub_name, LaserScan, self.callback_sonar)
        # Velocity publisher
        self.steering_signal = rospy.Publisher(self.velocity_command_topic_name, Twist, queue_size=10)

    # Callback for the sonar topic
    def callback_sonar(self, sonar_data):
        rospy.logdebug_once("Scan starts in angle {:.2f} , and ends in angle {:.2f}"
               .format(sonar_data.angle_min*(180/np.pi), sonar_data.angle_max*(180/np.pi)))

        angles = np.linspace(sonar_data.angle_max, sonar_data.angle_min, num=len(sonar_data.ranges))
        potentials_x = np.multiply(1/np.asarray(sonar_data.ranges)**2, np.cos(angles))
        self.sum_potentials_x = np.sum(potentials_x)
        potentials_y = np.multiply(1/np.asarray(sonar_data.ranges)**2, np.sin(angles))
        print('ranges', sonar_data.ranges)
        print('angles',angles)
        print('PX',potentials_x)
        print('PY',potentials_y)
        self.sum_potentials_y = np.sum(potentials_y)

 
        

    def there_is_obstacle(self, range, min_dist_to_obstacle):        
        if range <= min_dist_to_obstacle:
            obstacle = True
        else:
            obstacle = False
        return obstacle

    def set_velocity_values(self, linear_x_vel):
    
        linear_x = linear_x_vel - self.Kp * self.sum_potentials_x
        print('self.sum_potentials_x',self.sum_potentials_x)
        print('linear_x',linear_x)
        # compute the angle between linear velocity and vy generated by potential obstacles
        angle = np.arctan(float(self.sum_potentials_y/linear_x))
        print('px',self.sum_potentials_x)
        print('py',self.sum_potentials_y)
        print('angle ', angle)
        print('angleKphi', self.Kphi*angle)
        # Turn
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = self.Kphi*angle
        if msg.linear.x < 0.05 and msg.angular.x <= 0.05:
            msg.angular.z = 0.5
        self.steering_signal.publish(msg)


        



def tau_computation():
    rospy.init_node("tau_computation", anonymous=False, log_level=rospy.DEBUG)
    avoid_boi = SonarTauComputationClass(sonar_topic = '/rexrov2/sss_sonar')

    rate = rospy.Rate(8) # 8hz
    while not rospy.is_shutdown():
        avoid_boi.set_velocity_values(0.9)
        rate.sleep()

if __name__ == '__main__':
    tau_computation()