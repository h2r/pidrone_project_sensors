#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from raspicam_node.msg import MotionVectors
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty



class OpticalFlowNode(object):
    """
    Subscribe to the optical flow vectors and publish linear velocity as a Twist message.
    """
    def __init__(self, node_name):

        rospy.init_node(node_name)
        
        # flow variables
        camera_wh = (320, 240)        
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)

        self.altitude = 0.03 # initialize to a bit off the ground
        self.altitude_ts = rospy.Time.now()

        # subscribers
        self._sub_mv = rospy.Subscriber('/raspicam_node/motion_vectors', MotionVectors, self.motion_cb, queue_size=1)
        self._sub_alt = rospy.Subscriber('/pidrone/range', Range, self.altitude_cb, queue_size=1)
        self.setup()

    def setup(self):

        # ROS setup:
        ############
        # Publisher:
        # TODO: create a ROS publisher to publish the velocities
            # message type: TwistStamped
            # topic: /pidrone/picamera/twist
            # note: ensure that you pass in the argument queue_size=1 to the
            #       publisher to avoid lag
        # Subscriber:
        # TODO: subscribe to /pidrone/range to extract altitude (z position) for
        #       scaling
            # message type: Range
            # callback method: altitude_cb


        # TODO: subscribe to /raspicam_node/motion_vectors to extract the flow vectors for estimating velocity.
            # message type: MotionVectors
            # callback method: motion_cb




    def motion_cb(self, msg):
        ''' Average the motion vectors and publish the
        twist message that is the average of all the vectors.. 
        '''
        # signed 1-byte values
        x = msg.x
        y = msg.y

        # calculate the planar and yaw motions

        # TODO: calculate the optical flow velocities by summing the flow vectors
        opflow_x = ??? 
        opflow_y = ??? 

        
        x_motion = opflow_x * self.flow_coeff * self.altitude
        y_motion = opflow_y * self.flow_coeff * self.altitude

        
        # TODO: Create a TwistStamped message, fill in the values you've calculated,
        #       and publish this using the publisher you've created in setup



        
        duration_from_last_altitude = rospy.Time.now() - self.altitude_ts
        if duration_from_last_altitude.to_sec() > 10:
            rospy.logwarn("No altitude received for {:10.4f} seconds.".format(duration_from_last_altitude.to_sec()))

# TODO: Implement this method
    def altitude_cb(self, msg):
        """
        The altitude of the robot
        Args:
            msg:  the message publishing the altitude

        """
        self.altitude = ??
        self.altitude_ts = ??
    
def main():
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()

if __name__ == '__main__':
    main()
