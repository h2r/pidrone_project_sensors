#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/pi/ws/src/pidrone_pkg/scripts')
import rospy
import signal
import Adafruit_ADS1x15
from std_msgs.msg import Empty
from sensor_msgs.msg import Range


class IR:
    """
    This is your IR class, which extracts distance readings from the infrared sensor on your drone! You will use
    these readings to estimate the drone's altitude, which is critical in many of the other motion and vision scripts.
    Be sure to read the docstrings carefully and fill in all parts indicated in the project instructions.
    """

    def __init__(self):
        """
        Here is where you can initialize any instance variables you might need!
        """
        self.adc = Adafruit_ADS1x15.ADS1115()
        self.GAIN = 1

    def get_voltage(self):
        """
        The IR sensor outputs an analog voltage signal that corresponds to distance, which is then converted into a
        digital signal by the ADC. This method returns the voltage after the ADC conversion.

        :return: A 12-bit integer voltage corresponding to the distance measured by the IR sensor
        """
        return self.adc.read_adc(0, self.GAIN)

    def calc_distance(self, voltage):
        """
        Here is where you will convert the voltage obtained from the IR sensor to an actual distance in meters. The
        distance is inversely proportional to the voltage and must be scaled and offset:

        d = m(1/V) + b

        You can find m and b empirically by lifting the drone and tracking the voltage and the corresponding height (as
        measured by a meter stick).

        :param voltage: The digital voltage output of the IR sensor/ADC
        :return: The distance corresponding to the input voltage
        """
        pass

    def publish_range(self, range):
        """
        Here is where you will want to publish your IR range reading. You will publish a ROS Range message, which has
        already been imported for you. Fill in the header.stamp, max_range, min_range, range, and header.frame_id
        fields of the message you publish. The header.frame_id field should be set to "base". For more details on the
        Range message, look at http://docs.ros.org/indigo/api/sensor_msgs/html/msg/Range.html

        :param range: The range/distance reading of the IR sensor
        """
        pass


def ctrl_c_handler(signal, frame):
    """
    This method quits the program if ctrl-c is pressed. You do not need to modify this method.
    """
    print "\nCaught ctrl-c! Stopping node."
    sys.exit(0)


def main():
    """
    This is your main method, where you will start the ROS node, create the publishers, and continuously update and
    publish the IR sensor data. Fill in all TODOs.
    """

    # ROS Setup
    ###########
    # TODO: initialize a ros node

    # TODO: instantiate your IR object

    # Publishers
    ############
    # TODO: instantiate a range publisher and a heartbeat publisher. the range publisher should publish a Range message
    # to the topic '/pidrone/infrared' while the heartbeat publisher should publish an Empty (another standard ROS type)
    # message to the topic '/pidrone/heartbeat/infrared'. each sensor has a heartbeat publisher to ensure that the mode
    # controller knows which sensors are publishing their data without actually accessing the data itself. both
    # publishers should have a queue size of 1.

    print 'Publishing IR'

    # Non-ROS Setup
    ###############
    # set the while loop frequency
    r = rospy.Rate(100)
    # set up the ctrl-c handler
    signal.signal(signal.SIGINT, ctrl_c_handler)

    while not rospy.is_shutdown():
        # TODO: calculate the range and publish the range and heartbeat messages

        r.sleep()


if __name__ == "__main__":
    main()
