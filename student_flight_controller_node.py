#!/usr/bin/env python
import traceback
import sys
import yaml

import rospy
import rospkg
import signal
import numpy as np

print("tf import")
import tf
print("done tf import")


import command_values as cmds
from sensor_msgs.msg import Imu
from h2rMultiWii import MultiWii
from serial import SerialException
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import Quaternion
from pidrone_pkg.msg import Battery, Mode, RC, State
from sensor_msgs.msg import Range
import os


class FlightController(object):
    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/range
    /pidrone/heartbeat/web_interface
    /pidrone/heartbeat/pid_controller
    /pidrone/state
    """

    def __init__(self):
        # Connect to the flight controller board
        print("getboard")
        self.board = self.getBoard()
        print("done")
        # stores the current and previous modes
        self.curr_mode = 'DISARMED'         #initialize as disarmed
        self.prev_mode = 'DISARMED'         #initialize as disarmed
        # store the command to send to the flight controller
        self.command = cmds.disarm_cmd      #initialize as disarmed
        self.last_command = cmds.disarm_cmd
        # store the mode publisher
        self.modepub = None
        # store the time for angular velocity calculations
        self.time = rospy.Time.now()

        # Initialize the Imu Message
        ############################
        # TODO: create an Imu message with some initial values.
        # In a different method, the message will be updated and
        # published (you will do this in another TODO).
        # Hint: investigate the Header message type, the Imu
        # message type, and rospy.Time.
        self.imu_message = ...
        self.imu_message.header = ...
        self.imu_message.header.frame_id = 'Body'
        self.imu_message.header.stamp = ...

        # Initialize the Battery Message
        ################################
        # TODO: create a Battery message with some initial values.
        # In a different method, the message will be updated and
        # published (you will do this in another TODO). Hint: read
        # the Battery message file in the pidrone_pkg msg folder.
        self.battery_message = ...
        self.battery_message.CHANGE_FIELD_HERE = None
        self.battery_message.CHANGE_FIELD_HERE = None

        # Adjust this based on how low the battery should discharge
        self.minimum_voltage = 4.5

        # Accelerometer parameters
        ##########################
        print("loading")
        rospack = rospkg.RosPack()
        path = rospack.get_path('pidrone_pkg')
        with open("%s/params/multiwii.yaml" % path) as f:
            means = yaml.load(f)
        print("done")
        self.accRawToMss = 9.8 / means["az"]
        self.accZeroX=means["ax"] * self.accRawToMss
        self.accZeroY=means["ay"] * self.accRawToMss
        self.accZeroZ=means["az"] * self.accRawToMss
        pass


    # ROS subscriber callback methods:
    ##################################
    def desired_mode_callback(self, msg):
        """ Set the current mode to the desired mode """
        self.prev_mode = self.curr_mode
        self.curr_mode = msg.mode
        self.update_command()

    def fly_commands_callback(self, msg):
        """ Store and send the flight commands if the current mode is FLYING """
        if self.curr_mode == 'FLYING':
            r = msg.roll
            p = msg.pitch
            y = msg.yaw
            t = msg.throttle
            self.command = [r,p,y,t]

    # Update methods:
    #################
    def update_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """

        # extract roll, pitch, heading
        self.board.getData(MultiWii.ATTITUDE)
        # extract lin_acc_x, lin_acc_y, lin_acc_z
        self.board.getData(MultiWii.RAW_IMU)

        # TODO: calculate roll, pitch, and yaw in radians.
        # Hint: roll is rotation about which axis? What about pitch?
        # Hint: what data is in self.board.attitude? What about
        # self.board.rawIMU? Is the relevant data in degrees or radians?
        # Hint: yaw is sometimes referred to as "heading".
        roll = ...
        pitch = ...
        heading = ...
        # Note that at pitch angles near 90 degrees, the roll angle reading can
        # fluctuate a lot
        
        # transform heading (similar to yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is CCW
        heading = (-heading) % (2 * np.pi)

        # TODO: calculate the previous roll, pitch, heading values.
        # Hint: start by printing self.imu_message.orientation and
        # type(self.imu_message.orientation) in order to understand
        # the data better.
        # Hint: recall that euler angles can be transformed to quaternions,
        # and vice-versa.
        # Hint: investigate tf.transformations.euler_from_quaternion.
        previous_quaternion = self.imu_message.orientation
        previous_roll, previous_pitch, previous_heading = ...

        # Although quaternion_from_euler takes a heading in range [0, 2pi),
        # euler_from_quaternion returns a heading in range [0, pi] or [0, -pi).
        # Thus need to convert the returned heading back into the range [0, 2pi).
        previous_heading = previous_heading % (2 * np.pi)

        # TODO: transform the current Euler angles into a quaternion
        # Hint: recall that Euler angles can be transformed to quaternions,
        # and vice-versa.
        # Hint: investigate tf.transformations.quaternion_from_euler.
        quaternion = ...

        # TODO: extract the raw linear accelerations from the flight controller.
        # Hint: what data is in self.board.attitude? What about
        # self.board.rawIMU? 
        raw_acc_x = ...
        raw_acc_y = ...
        raw_acc_z = ...

        # Turn the raw linear accelerations into real accelerations
        lin_acc_x = raw_acc_x * self.accRawToMss - self.accZeroX
        lin_acc_y = raw_acc_y * self.accRawToMss - self.accZeroY
        lin_acc_z = raw_acc_z * self.accRawToMss - self.accZeroZ

        # Rotate the IMU frame to align with our convention for the drone's body
        # frame. IMU: x is forward, y is left, z is up. We want: x is right,
        # y is forward, z is up.
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        # Account for gravity's affect on linear acceleration values when roll
        # and pitch are nonzero. When the drone is pitched at 90 degrees, for
        # example, the z acceleration reads out as -9.8 m/s^2. This makes sense,
        # as the IMU, when powered up / when the calibration script is called,
        # zeros the body-frame z-axis acceleration to 0, but when it's pitched
        # 90 degrees, the body-frame z-axis is perpendicular to the force of
        # gravity, so, as if the drone were in free-fall (which was roughly
        # confirmed experimentally), the IMU reads -9.8 m/s^2 along the z-axis.
        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g*np.sin(roll)*np.cos(pitch)
        lin_acc_y_drone_body = lin_acc_y_drone_body + g*np.cos(roll)*(-np.sin(pitch))
        lin_acc_z_drone_body = lin_acc_z_drone_body + g*(1 - np.cos(roll)*np.cos(pitch))

        # calculate the angular velocities of roll, pitch, and yaw in rad/s
        time = rospy.Time.now()
        dt = ...
        delta_roll = ...
        delta_pitch = ...
        delta_yaw = ...
        angvx = self.near_zero(delta_roll / dt)
        angvy = self.near_zero(delta_pitch / dt)
        angvz = self.near_zero(delta_yaw / dt)
        self.time = time

        # TODO: Update the imu_message header stamp.
        self.imu_message.header.stamp = ...
        
        # TODO: update the IMU message orientation
        # Hint: is the orientation a set of Euler angles or a quaternion?
        self.imu_message.orientation.x = ...
        self.imu_message.orientation.y = ...
        self.imu_message.orientation.z = ...
        self.imu_message.orientation.w = ...

        # TODO: update the IMU message angular velocities.
        self.imu_message.CHANGE_FIELD_HERE = angvx
        self.imu_message.CHANGE_FIELD_HERE = angvy
        self.imu_message.CHANGE_FIELD_HERE = angvz
        
        # TODO: update the IMU message linear accelerations.
        self.imu_message.linear_acceleration.x = ...
        self.imu_message.linear_acceleration.y = ...
        self.imu_message.linear_acceleration.z = ...


    def update_battery_message(self):
        """
        Compute the ROS battery message by reading data from the board.
        """
        # extract vbat, amperage
        self.board.getData(MultiWii.ANALOG)


        # TODO: Update Battery message:
        self.battery_message.vbat = ... * 0.10
        self.battery_message.amperage = ... 



    def update_command(self):
        ''' Set command values if the mode is ARMED or DISARMED '''
        if self.curr_mode == 'DISARMED':
            self.command = cmds.disarm_cmd
        elif self.curr_mode == 'ARMED':
            if self.prev_mode == 'DISARMED':
                self.command = cmds.arm_cmd
            elif self.prev_mode == 'ARMED':
                self.command = cmds.idle_cmd

    # Helper Methods:
    #################
    def getBoard(self):
        """ Connect to the flight controller board """
        # (if the flight cotroll usb is unplugged and plugged back in,
        #  it becomes .../USB1)
        try:
            board = MultiWii('/dev/ttyUSB0')
        except SerialException as e:
            print(("usb0 failed: " + str(e)))
            try:
                board = MultiWii('/dev/ttyUSB1')
            except SerialException:
                print('\nCannot connect to the flight controller board.')
                print('The USB is unplugged. Please check connection.')
                raise
                sys.exit()
        return board

    def send_cmd(self):
        """ Send commands to the flight controller board """
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, self.command)
        self.board.receiveDataPacket()
        if (self.command != self.last_command):
            print(('command sent:', self.command))
            self.last_command = self.command

    def near_zero(self, n):
        """ Set a number to zero if it is below a threshold value """
        return 0 if abs(n) < 0.0001 else n

    def ctrl_c_handler(self, signal, frame):
        """ Disarm the drone and quits the flight controller node """
        print("\nCaught ctrl-c! About to Disarm!")
        self.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        self.board.receiveDataPacket()
        rospy.sleep(1)
        self.modepub.publish('DISARMED')
        print("Successfully Disarmed")
        sys.exit()

    # Heartbeat Callbacks: These update the last time that data was received
    #                       from a node
    def heartbeat_web_interface_callback(self, msg):
        """Update web_interface heartbeat"""
        self.heartbeat_web_interface = rospy.Time.now()

    def heartbeat_pid_controller_callback(self, msg):
        """Update pid_controller heartbeat"""
        self.heartbeat_pid_controller = rospy.Time.now()

    def heartbeat_infrared_callback(self, msg):
        """Update ir sensor heartbeat"""
        self.heartbeat_infrared = rospy.Time.now()
        self.range = ... # TODO

    def heartbeat_state_estimator_callback(self, msg):
        """Update state_estimator heartbeat"""
        self.heartbeat_state_estimator = rospy.Time.now()

    def shouldIDisarm(self):
        """
        Disarm the drone if the battery values are too low or if there is a
        missing heartbeat
        """
        curr_time = rospy.Time.now()
        disarm = False
        if self.battery_message.vbat != None and self.battery_message.vbat < self.minimum_voltage:
            print('\nSafety Failure: low battery\n')
            disarm = False
        if curr_time - self.heartbeat_web_interface > rospy.Duration.from_sec(3):
            print('\nSafety Failure: web interface heartbeat\n')
            print('The web interface stopped responding. Check your browser')
            disarm = True
        if curr_time - self.heartbeat_pid_controller > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving flight commands.')
            print('Check the pid_controller node\n')
            disarm = True
        if curr_time - self.heartbeat_infrared > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving data from the IR sensor.')
            print('Check the infrared node\n')
            disarm = True

        if self.range > 0.5:
            print(('\nSafety Failure: too high: ' + str(self.range)))
            disarm = True            
        if curr_time - self.heartbeat_state_estimator > rospy.Duration.from_sec(1):
            print('\nSafety Failure: not receiving a state estimate.')
            print('Check the state_estimator node\n')
            disarm = True

        return disarm


def main():
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    print("init")
    rospy.init_node(node_name)
    print("done")
    # create the FlightController object
    fc = FlightController()
    curr_time = rospy.Time.now()
    fc.heartbeat_infrared = curr_time
    fc.range = None
    fc.heartbeat_web_interface= curr_time
    fc.heartbeat_pid_controller = curr_time
    fc.heartbeat_flight_controller = curr_time
    fc.heartbeat_state_estimator = curr_time

    # Publishers
    ###########
    imupub = rospy.Publisher('/pidrone/imu', Imu, queue_size=1, tcp_nodelay=False)
    batpub = rospy.Publisher('/pidrone/battery', Battery, queue_size=1, tcp_nodelay=False)
    fc.modepub = rospy.Publisher('/pidrone/mode', Mode, queue_size=1, tcp_nodelay=False)
    print('Publishing:')
    print('/pidrone/imu')
    print('/pidrone/mode')
    print('/pidrone/battery')

    # Subscribers
    ############
    rospy.Subscriber("/pidrone/desired/mode", Mode, fc.desired_mode_callback)
    rospy.Subscriber('/pidrone/fly_commands', RC, fc.fly_commands_callback)
    # heartbeat subscribers
    rospy.Subscriber("/pidrone/range", Range, fc.heartbeat_infrared_callback)
    rospy.Subscriber("/pidrone/heartbeat/web_interface", Empty, fc.heartbeat_web_interface_callback)
    rospy.Subscriber("/pidrone/heartbeat/pid_controller", Empty, fc.heartbeat_pid_controller_callback)
    rospy.Subscriber("/pidrone/state", State, fc.heartbeat_state_estimator_callback)


    signal.signal(signal.SIGINT, fc.ctrl_c_handler)
    # set the loop rate (Hz)
    r = rospy.Rate(60)
    try:
        while not rospy.is_shutdown():
            # if the current mode is anything other than disarmed
            # preform as safety check
                # Break the loop if a safety check has failed
            if fc.shouldIDisarm():
                print(("mode", fc.curr_mode))
                break
                
            # update and publish flight controller readings
            fc.update_battery_message()
            fc.update_imu_message()
            imupub.publish(fc.imu_message)
            batpub.publish(fc.battery_message)

            # update and send the flight commands to the board
            fc.update_command()
            fc.send_cmd()

            # publish the current mode of the drone
            fc.modepub.publish(fc.curr_mode)

            # sleep for the remainder of the loop time
            r.sleep()
            
    except SerialException:
        print('\nCannot connect to the flight controller board.')
        print('The USB is unplugged. Please check connection.')
    except Exception as e:
        print(('there was an internal error', e))
        print((traceback.format_exc()))
    finally:
        print('Shutdown received')
        print('Sending DISARM command')
        fc.board.sendCMD(8, MultiWii.SET_RAW_RC, cmds.disarm_cmd)
        fc.board.receiveDataPacket()


if __name__ == '__main__':
    main()