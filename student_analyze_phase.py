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
        # TODO: Initialize any instance variables you wish to use

        # ROS Setup
        ###########
        # Publishers:
        # TODO: create a ROS publisher for the estimated positions
            # message type: PoseStamped
            # topic: /pidrone/picamera/pose

        # TODO: create a publisher for the lost variable
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

    def write(self, data):
        ''' A method that is called everytime an image is taken '''

        # Run the following only if position control is enabled to conserve computation resources
            # reshape the image
            ###image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
            # If no first image is stored
                # update the image variables

            # else
                # try to estimate the transformation from the first image to the
                # current image to get a position estimate.
                ###transform_first = cv2.estimateRigidTransform(self.first_image, image, False)

                # if the first image was visible (the transformation was succesful) :
                    # calculate the x,y, and yaw from the transformation
                    # update first image data
                # else the first image was not visible (the transformation was not succesful) :
                    # try to estimate the transformation from the previous image
                    ###transform_previous = cv2.estimateRigidTransform(self.previous_image, image, False)

                    # if the previous image was visible (the transformation was succesful)
                        # calculate the position by integrating
                    # if the previous image wasn't visible (the transformation was not succesful)
                        # reset the pose


        # publish the pose message



    # TODO: Implement this method to extract useful information from the affine transformation
    def translation_and_yaw(self, transform):
        """ Takes an affine transform and returns the x and y translations of
        the camera and the yaw

        transform : an affine transformation
        """
        translation_x_y = None

        yaw = None

        return translation_x_y, yaw


# ROS CALLBACK METHODS:
#######################
# TODO: Implement
    def reset_callback(self, msg):
        """ Reset the current position and orientation """
        print "Resetting Phase"


# TODO: Implement
    def position_control_callback(self, msg):
        ''' Store whether the pose is calculated and published '''
        pass

# TODO: Implement
    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State, along with most recent
        x and y position estimate
        """
        pass
