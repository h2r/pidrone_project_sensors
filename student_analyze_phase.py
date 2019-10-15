#!/usr/bin/env python
import tf
import cv2
import rospy
import picamera
import numpy as np
from pidrone_pkg.msg import State
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseStamped


class AnalyzePhase(picamera.array.PiMotionAnalysis):
    """
    A class that uses OpenCV's estimateRigidTransform method to calculate
    the change in position of the drone.
    For more info, visit:
    https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#estimaterigidtransform

    Publishers:
    /pidrone/picamera/pose

    Subscribers:
    /pidrone/reset_transform
    /pidrone/position_control
    """

    def setup(self):       
        # initialize the Pose data
        self.pose_msg = PoseStamped()
        self.altitude = 0.0
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0

        # position hold is initialized as False
        self.position_control = False
        self.first_image = None
        self.previous_image = None
        
        # var indicating whether we are taking our first image
        self.first = True

        # ROS Setup
        ###########
        # Publishers:
        # TODO: create a ROS publisher for the estimated positions
            # message type: PoseStamped
            # topic: /pidrone/picamera/pose

        # TODO: create a publisher that publishes a boolean indicating whether the drone is lost or not
            # message type: Bool
            # topic: /pidrone/picamera/lost


        # Subscribers:
        # TODO: subscribe to /pidrone/reset_transform
            # message type: Empty
            # callback method: reset_callback

        # TODO: subscribe to /pidrone/position_control
            # message type: Bool
            # callback method: position_control_callback

        # TODO: subscribe to /pidrone/state
            # message type: State
            # callback method: state_callback

    # TODO: Implement this method
    def write(self, data):
        ''' A method that is called everytime an image is taken.'''
       
        # Run the following only if position control is enabled to conserve computation resources
            # reshape the image
            image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
            # If no first image is stored, store the image as the first image
                # update the image variables

            # else
                # try to estimate the transformation from the first image to the
                # current image to get a position estimate.
                transform_first = cv2.estimateRigidTransform(self.first_image, image, False)

                # if the first image was visible (the transformation was succesful and transform_first is not None) :
                    # calculate the x,y, and yaw from the transformation
                    # update first image data
                # else the first image was not visible (transform_first was None) :
                    # try to estimate the transformation from the previous image
                    transform_previous = cv2.estimateRigidTransform(self.previous_image, image, False)

                    # if the previous image was visible (transform_previous is not None)
                        # calculate the position by adding the displacement to the previous
                        # position of the drone.
                        # HINT: use self.x_position_from_state and self.y_position_from_state as the
                        # previous position
                        self.pose_msg.pose.position.x = self.x_position_from_state + ???
                        self.pose_msg.pose.position.y = self.y_position_from_state + ???
                        
                    # if the previous image wasn't visible (transform_previous is None)
                        # reset the pose
            
            # set the previous image equal to the current image
            self.previous_image = image

        # publish the pose message



    # TODO: Implement this method to extract useful information from the affine transformation
    def translation_and_yaw(self, transform):
        """ Takes an affine transform and returns the x and y translations of
        the camera and the yaw in meter coordinates

        transform : an affine transformation. Refer to the Sensors Theory assignment for more 
        information about this matrix. You should use your answer to the theory assignment to 
        help you here.
        """
        
        # TODO: extract the translation information from the transform variable. Divide the 
        # the x displacement by 320, which is the width of the camera resolution. Divide the
        # y displacement by 240, the height of the camera resolution.
        pixel_translation_x_y = ??? 
        
        real_translation_x_y = [0.0, 0.0]
        real_translation_x_y[0] = (pixel_translation_x_y[0] / 320.0) * self.altitude
        real_translation_x_y[1] = (translation_x_y[1] / 240.0) * self.altitude

        # TODO: use np.arctan2 and the transform variable to calculate the yaw
        yaw = ???
        
        return translation_x_y, yaw


    # ROS CALLBACK METHODS:
    #######################
    # TODO: Implement
    def reset_callback(self, msg):
        """ When this method is called, the position and orientation should be reset. """
        print "Resetting Phase"


    # TODO: Implement
    def position_control_callback(self, msg):
        ''' Store whether the position control is turned on in an instance variable '''
        pass

    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State, along with most recent
        x and y position estimate
        """
        self.altitude = msg.pose_with_covariance.pose.position.z
        self.x_position_from_state = msg.pose_with_covariance.pose.position.x
        self.y_position_from_state = msg.pose_with_covariance.pose.position.y
