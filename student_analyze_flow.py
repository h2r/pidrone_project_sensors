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

        # initilize the instance variables used to scale the motion vectors
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)

        self.altitude = 0.0

        # ROS setup:
        ############
        # Publisher:
        # TODO: create a ROS publisher to publish the velocities
            # message type: TwistStamped
            # topic: /pidrone/picamera/twist
            # note: ensure that you pass in the argument queue_size=1 to the
            #       publisher to avoid lag
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
        #       using the flow_coeff and the altitude

        # TODO: Create a TwistStamped message, fill in the values you've calculated,
        #       and publish this using the publisher you've created in setup

# TODO: Implement this method
    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State
        """
        pass


def main():
    import sys
    import os
    from cv_bridge import CvBridge

    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    print "Analyze flow started"

    try:
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with AnalyzeFlow(camera) as flow_analyzer:
                flow_analyzer.setup(camera.resolution)
                camera.start_recording("/dev/null", format='h264', motion_output=flow_analyzer)

                while not rospy.is_shutdown():
                    camera.wait_recording(1/100.0)
            camera.stop_recording()

        print "Shutdown Received"
        sys.exit()

    except Exception as e:
        print "Camera Error!"
        raise


if __name__ == '__main__':
    main()
