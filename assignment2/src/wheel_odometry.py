#!/usr/bin/env python3
import numpy as np
import time
import rospy
import rosbag

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern, Pose2DStamped
from std_msgs.msg import Header, String
from duckietown_msgs.srv import ChangePattern


class OdometryNode(DTROS):

    def __init__(self, node_name: str, desired_distance: float = 1):

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius')
        self._baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline')

        # Assigning variables
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
                                                  WheelsCmdStamped, queue_size=1)
        self.led = rospy.Publisher(f'/{self.veh_name}/led_emitter_node/led_pattern',
                                   LEDPattern, queue_size=1)

        # Services
        led_service = f'/{self.veh_name}/led_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # Bags
        self.bag_name = '/data/bags/robot_data.bag'
        self.bag = rosbag.Bag(self.bag_name, 'w')

    def cb_encoder_data(self, msg: WheelEncoderStamped, wheel: str):
        '''
        Getting the data from the nodes using subscribers.
        Args:
            msg: the latest data from the subscriber
            wheel: an argument to know which wheel is in use
        '''
        # updating to get the intial values
        if self.prev_values[wheel] == 0:
            self.prev_values[wheel] = msg.data
            return

        diff_value = msg.data - self.prev_values[wheel]
        self.prev_values[wheel] = msg.data

        dist = 2 * np.pi * self._radius * diff_value / msg.resolution

        self.d[wheel] = dist
        self.traveled_distance[wheel] += dist

        # updating the robot frame coordinates
        self.update_coordinates()

    def clear(self):
        '''
        Clearing the traveled distance after every command.
        '''
        self.traveled_distance = {'left': 0, 'right': 0}

    def move(self, vel_left: float, vel_right: float):
        '''
        Updating the velocity of the robot.
        Args:
            vel_left: left wheel velocity
            vel_right: right wheel velocity
        '''
        header = Header()

        self.pub_wheel_commands.publish(
            WheelsCmdStamped(
                header=header,
                vel_left=vel_left,
                vel_right=vel_right
            ))

    def change_led_lights(self, color: str):
        '''
        Sends msg to service server
        Colors:
            "off": [0,0,0],
            "white": [1,1,1],
            "green": [0,1,0],
            "red": [1,0,0],
            "blue": [0,0,1],
            "yellow": [1,0.8,0],
            "purple": [1,0,1],
            "cyan": [0,1,1],
            "pink": [1,0,0.5],
        '''
        msg = String()
        msg.data = color
        self.led_pattern(msg)

    def stop(self, seconds: int = None):
        '''
        Sends a velocity of 0 to each wheel of the duckiebot to stop it.
        Args:
            seconds: seconds duckiebot should stop
        '''

        # changing the LED light to red
        self.change_led_lights('red')

        if seconds is None:
            self.move(vel_left=0.0, vel_right=0.0)
            return

        start = time.time()
        end = start

        while not rospy.is_shutdown() and (end - start) < seconds:
            self.move(vel_left=0.0, vel_right=0.0)
            end = time.time()

    def rotate(self, rate: rospy.Rate, desired_distance: float, vel_left: float,
               vel_right: float, clockwise: bool = True):
        '''
        Rotating by the specified distance.
        Args:
            rate: an instance of rospy.Rate
            desired_distance: the desired distance the duckiebot should travel
            vel_left: left wheel velocity
            vel_right: right wheel velocity
            clockwise: and indicator whether the turn is in clockwise direction or not
        '''

        # changing the LED color to blue and clearing the traveled disctance
        self.change_led_lights('blue')
        self.clear()

        if clockwise:
            while self.traveled_distance['left'] < desired_distance:
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()
        else:
            while self.traveled_distance['right'] < desired_distance:
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()

    def forward(self, rate: rospy.Rate, desired_distance: float, vel_left: float, vel_right: float):
        '''
        Going forward by the specified distance.
        Args:
            rate: an instance of rospy.Rate
            desired_distance: the desired distance the duckiebot should travel
            vel_left: left wheel velocity
            vel_right: right wheel velocity
        '''

        # changing the LED color to green and clearing the traveled disctance
        self.change_led_lights('green')
        self.clear()

        while not rospy.is_shutdown() and (self.traveled_distance['left'] < desired_distance or
                                           self.traveled_distance['right'] < desired_distance):
            self.move(vel_left=vel_left, vel_right=vel_right)
            rate.sleep()

    def circle(self, rate: rospy.Rate, radius: float, propotion: float, speed_ratio: float, vel_left: float, vel_right: float):
        '''
        Moving in a circle of the specified radius.
        Args:
            rate: an instance of rospy.Rate
            radius: the desired radius of the circle
            proportion: scaling of the circle
            speed_ration: the weight to give to one of the wheels
            vel_left: left wheel velocity
            vel_right: right wheel velocity
        '''
        # changing the lights to purple and clearning the distance
        self.change_led_lights("purple")
        self.clear()

        left_desired_distance = (
            2 * radius + self._baseline) * np.pi * propotion
        speed_ratio = (radius - self._baseline / 2) / (radius +
                                                       self._baseline / 2) if speed_ratio is None else speed_ratio

        while not rospy.is_shutdown() and self.traveled_distance['left'] < left_desired_distance:
            self.move(vel_left, vel_right * speed_ratio)
            rate.sleep()

    def robot_to_world_frame(self):
        '''
        Matrix (Robot -> World):
        sin(t)  -cos(t) 0.32
        cos(t)   sin(t) 0.32
        0        0      1
        '''
        homo_coord = np.array(
            [self.robot_frame['x'], self.robot_frame['y'], 1.0])
        theta = self.robot_frame['theta']
        
        tranfromation_matrix = np.array([
            [np.sin(theta), -np.cos(theta), 0.32],
            [np.cos(theta),  np.sin(theta), 0.32],
            [0,              0,             1],
        ])
        world_frame = tranfromation_matrix @ homo_coord
        world_theta = theta + (np.pi / 2)
        world_frame[2] = world_theta
        return world_frame

    def write_in_bag(self, tr_matrix: np.Array):
        '''
        Writing in the bag the x and y coordinates of the robot.
        Args:
            tr_matrix: the transformed world frame matrix.
        '''
        try:
            header = Header()
            msg = Pose2DStamped()
            msg.header = header
            msg.x = tr_matrix[0]
            msg.y = tr_matrix[1]
            msg.theta = tr_matrix[2]
            self.bag.write("Coordinate", msg)

        except Exception as e:
            print(f'This is the error message for bag: {e}')

    def run(self):
        '''
        The main running point of the program
        '''
        # publish message every 0.1 second
        rate = rospy.Rate(10)

        # stop for 5 seconds
        self.stop(seconds=5)

        # turning clockwise
        dis_rot_distance = np.pi * (self._baseline + 0.04) / 4
        self.rotate(rate, dis_rot_distance, vel_left=0.45, vel_right=0.0)
        self.stop(seconds=1)

        # move forward
        self.forward(rate, self.desired_distance,
                     vel_left=0.48, vel_right=0.45)
        self.stop(seconds=1)

        # turning counter-clockwise
        self.rotate(rate, dis_rot_distance, vel_left=0,
                    vel_right=0.45, clockwise=False)
        self.stop(seconds=1)

        # move forward
        self.forward(rate, self.desired_distance,
                     vel_left=0.44, vel_right=0.43)
        self.stop(seconds=1)

        # turning counter-clockwise
        self.rotate(rate, dis_rot_distance - 0.03, vel_left=0,
                    vel_right=0.45, clockwise=False)
        self.stop(seconds=1)

        # move forward
        self.forward(rate, self.desired_distance,
                     vel_left=0.5, vel_right=0.45)
        self.stop(seconds=1)

        # getting back to initial position
        # turning counter-clockwise
        self.rotate(rate, dis_rot_distance, vel_left=0,
                    vel_right=0.5, clockwise=False)
        self.stop(seconds=1)

        # move forward
        self.forward(rate, self.desired_distance,
                     vel_left=0.5, vel_right=0.45)

        self.stop(seconds=1)

        # turning clockwise 180 degrees
        self.rotate(rate, 2 * dis_rot_distance + 0.05, vel_left=0,
                    vel_right=0.5, clockwise=False)
        self.stop(seconds=1)

        self.stop(seconds=5)

        # moving in clockwise circular motion
        self.circle(radius=0.3, rate=rate, propotion=1.2,
                    speed_ratio=0.5, vel_left=0.45, vel_right=0.45)
        self.stop(seconds=5)

        self.bag.close()
        rospy.signal_shutdown('Done Everything!')

    def update_coordinates(self):
        '''
        Updating the robot frame coordinates and writing them in a bag
        '''
        delta_theta = (self.d['right'] - self.d['left']) / self._baseline
        delta_A = (self.d['left'] + self.d['right']) / 2
        delta_x = delta_A * np.cos(self.robot_frame['theta'])
        delta_y = delta_A * np.sin(self.robot_frame['theta'])

        self.robot_frame['x'] += delta_x
        self.robot_frame['y'] += delta_y
        self.robot_frame['theta'] = (
            self.robot_frame['theta'] + delta_theta) % (2 * np.pi)

        # recording in the rosbag
        world_frame = self.robot_to_world_frame()
        self.write_in_bag(world_frame)


if __name__ == '__main__':
    node = OdometryNode(node_name='wheel_odometry')

    # Keep it spinning to keep the node alive
    node.run()

    rospy.spin()
    rospy.signal_shutdown('Done Everything!')
