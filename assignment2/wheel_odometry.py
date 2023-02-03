#!/usr/bin/env python3
import numpy as np
import time
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern
from std_msgs.msg import Header, ColorRGBA


class OdometryNode(DTROS):

    def __init__(self, node_name, desired_distance=1.25):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # rospy.init_node('LED_color_changer')

        # Get static parameters
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius')
        self._baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline')
        self.desired_distance = desired_distance

        self.prev_values = {'left': 0, 'right': 0}
        self.d = {'left': 0, 'right': 0}
        self.traveled_distance = {'left': 0, 'right': 0}
        self.robot_frame = {'x': 0, 'y': 0, 'theta': 0}

        # Subscribers
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',
                                                       WheelEncoderStamped, self.cb_encoder_data,
                                                       callback_args='left', queue_size=1)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',
                                                        WheelEncoderStamped, self.cb_encoder_data,
                                                        callback_args='right', queue_size=1)

        # Publishers
        self.pub_wheel_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd',
                                                  WheelsCmdStamped, queue_size=10)
        self.led = rospy.Publisher(f'/{self.veh_name}//led_emitter_node/led_pattern',
                                   LEDPattern, queue_size=1)

    def cb_encoder_data(self, msg, wheel):
        """ 
            Update encoder distance information from ticks.
        """
        # updating to get the intial values
        if self.prev_values[wheel] == 0:
            self.prev_values[wheel] = msg.data
            return

        diff_value = msg.data - self.prev_values[wheel]
        self.prev_values[wheel] = msg.data

        dist = 2 * np.pi * self._radius * diff_value / msg.resolution

        self.d[wheel] = dist
        self.traveled_distance[wheel] += dist

        self.update_coordinates()
        print(self.robot_frame)

    def move(self, vel_left=0.0, vel_right=0.0):
        header = Header()

        self.pub_wheel_commands.publish(
            WheelsCmdStamped(
                header=header,
                vel_left=vel_left,
                vel_right=vel_right
            ))

    def change_led_lights(self, rgba):
        header = Header()

        self.led.publish(
            LEDPattern(
                header=header,
                color_list=[],
                rgb_vals=[ColorRGBA(*rgba), ColorRGBA(*rgba),
                          ColorRGBA(*rgba), ColorRGBA(*rgba)],
                color_mask=[],
                frequency=0,
                frequency_mask=[]
            ))

    def stop(self, seconds=None):
        if seconds is None:
            self.move(vel_left=0.0, vel_right=0.0)
            return

        self.change_led_lights(rgba=(255, 0, 0, 1))

        start = time.time()
        end = start
        while not rospy.is_shutdown() and (end - start) < seconds:
            self.move(vel_left=0.0, vel_right=0.0)
            end = time.time()
            print("STOP!")

    def rotate(self, rate, desired_distance, vel_left=0.2, vel_right=-0.2, direction=None):
        if direction is None:
            while not rospy.is_shutdown() and (self.traveled_distance['left'] < desired_distance and
                                               self.traveled_distance['right'] > -desired_distance):
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()
        else:
            while not rospy.is_shutdown() and (self.traveled_distance['left'] > -desired_distance and
                                               self.traveled_distance['right'] < desired_distance):
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()

    def forward(self, rate, desired_distance, vel_left=0.43, vel_right=0.42):
        while not rospy.is_shutdown() and (self.traveled_distance['left'] < desired_distance and
                                           self.traveled_distance['right'] < desired_distance):
            self.move(vel_left=vel_left, vel_right=vel_right)
            rate.sleep()

    def backward(self, rate, vel_left=-0.43, vel_right=-0.42):
        while not rospy.is_shutdown() and (self.traveled_distance['left'] > 0 and
                                           self.traveled_distance['right'] > 0):
            self.move(vel_left=vel_left, vel_right=vel_right)
            rate.sleep()

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(10)

        # stop for 5 seconds and change light to red
        # self.stop(seconds=5)

        # turning clockwise
        dis_rot_distance = np.pi * self._baseline / 2
        self.rotate(rate, dis_rot_distance, vel_left=0.2, vel_right=0)
        self.stop()

        # move forward
        self.forward(rate, self.desired_distance, vel_left=0.4, vel_right=0.42)

        # turning counter-clockwise
        self.rotate(rate, dis_rot_distance, vel_left=0,
                    vel_right=0.2, direction='counter')

        self.stop()

    def update_coordinates(self):
        delta_theta = (self.d['right'] - self.d['left']) / self._baseline
        delta_A = (self.d['left'] + self.d['right']) / 2
        delta_x = delta_A * np.cos(self.robot_frame['theta'])
        delta_y = delta_A * np.sin(self.robot_frame['theta'])

        self.robot_frame['x'] += delta_x
        self.robot_frame['y'] += delta_y
        self.robot_frame['theta'] = (
            self.robot_frame['theta'] + delta_theta) % (2 * np.pi)


if __name__ == '__main__':
    node = OdometryNode(node_name='wheel_odometry')

    # Keep it spinning to keep the node alive
    node.run()

    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
