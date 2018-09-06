from __future__ import division
import rospy
import numpy as np
import picamera.array
from pidrone_pkg.msg import State
from geometry_msgs.msg import TwistStamped


class AnalyzeFlow(picamera.array.PiMotionAnalysis):
    ''' A class used for real-time motion analysis of optical flow vectors to
    obtain the planar and yaw velocities of the drone.

    For more information, read the following:
    http://picamera.readthedocs.io/en/release-1.10/api_array.html#picamera.array.PiMotionAnalysis


    Publisher:
    /pidrone/picamera/twist

    Subscriber:
    /pidrone/state
    '''

    def setup(self, camera_wh):
        ''' Initialize the instance variables '''

        # TODO: calculate and initilize the instance variables used to scale
        #       the motion vectors


        # ROS setup:
        ############
        # Publisher:
        # TODO: create a ROS publisher to publish the velocities
            # message type: TwistStamped
            # topic: /pidrone/picamera/twist
        # Subscriber:
        # TODO: subscribe to /pidrone/state to extract altitude (z position) for
        #       scaling
            # message type: State
            # callback method: state_callback

    def analyse(self, a):
        ''' Analyze the frame, calculate the motion vectors, and publish the
        twist message. This is implicitly when  the camera is recording and
        AnalyzeFlow is set as the motion analyzer argument

        a : an array of the incoming motion data that is provided by the
            PiMotionAnalysis API
        '''
        # signed 1-byte values
        x = a['x']
        y = a['y']

        # TODO: calculate the velocities by summing and scaling the flow vectors

        # TODO: Create a TwistStamped message, fill in the values you've calculated,
        #       and publish this using the publisher you've created in setup

# TODO: Implement this method
    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State
        """
        pass
