import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

from can import Bus, ThreadSafeBus

from . utils import load_config, handle_message

import threading
import math

class Bridge(Node):

    def __init__(self):
        super().__init__('bridge')

        # declare the parameters
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bustype", "socketcan")
        self.declare_parameter("read_period", 0.005) # can reading period in seconds
        self.declare_parameter("config_path", "docs/ids.json")
        
        # create the publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 'ackermann', 10)
        self.right_wheel_sub = self.create_publisher(Float32, 'right_wheels_speed', 10)
        self.left_wheel_sub = self.create_publisher(Float32, 'left_wheels_speed', 10)
        self.drive_speed_sub = self.create_publisher(Float32, 'drive_speed', 10)
        self.gnd_speed_sub = self.create_publisher(Float32, 'gnd_speed', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)

        # load the CAN configs
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.var_to_can, self.id_to_vars = load_config(self.config_path)

        # create filters for the CAN messages
        self.filters = []
        for var in self.var_to_can:
            id = int(self.var_to_can[var]['ID'], 16)
            self.filters.append({"can_id": id, "can_mask": id})

        # define the bus
        try:
            self.bus = Bus(channel=self.get_parameter("can_channel").value, bustype=self.get_parameter("can_bustype").value, filters=self.filters)
        except:
            raise ConnectionError("Could not connect to CAN bus!")
        
        # create the reading timer
        self.period = self.get_parameter("read_period").value
        self.timer = self.create_timer(self.period, self.timer_callback)

    def val_reading_routine(self, var):

        # publish the ackermann msg
        """
        if var['name'] == "GND_SPEED" and "STEERING_POS" in values:
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.steering_angle = values["STEERING_POS"]
            ack_msg.drive.speed = values["GND_SPEED"]
            self.ackermann_pub.publish(ack_msg)
        """

        # publish the drive speed as redundancy
        if var['name'] == "DRIVE_SPEED":
            drive_msg = Float32()
            drive_msg.data = float(var['value'])
            self.drive_speed_sub.publish(drive_msg)

        # publish the ground speed
        elif var['name'] == "GND_SPEED":
            gnd_msg = Float32()
            gnd_msg.data = float(var['value'])
            self.gnd_speed_sub.publish(gnd_msg)

        # publish the right wheel speed
        elif var['name'] == "RIGHT_DRIVE_SPEED" or var['name'] == "RIGHT_GROUND_SPEED":
            right_msg = Float32()
            right_msg.data = float(var['value'])
            self.right_wheel_sub.publish(right_msg)

        # publish the left wheel speed
        elif var['name'] == "LEFT_DRIVE_SPEED" or var['name'] == "LEFT_GROUND_SPEED":
            left_msg = Float32()
            left_msg.data = float(var['value'])
            self.left_wheel_sub.publish(left_msg)

        # publish the steering angle
        elif var['name'] == "STEERING_POS":
            steering_msg = Float32()
            steering_msg.data = float(var['value'])
            self.steering_pub.publish(steering_msg)


    def timer_callback(self):

        # read a message without blocking
        while True:
            msg = self.bus.recv(0.0)

            # stop when the buffer is already empty
            if msg is None:
                break

            # handle the received message
            values = handle_message(msg, self.id_to_vars, self.var_to_can)

            """
            if len(values) > 0:
                print(values)
            """

            # publish the values
            for var in values:
                # publish the drive speed as redundancy
                if var['name'] == "DRIVE_SPEED":
                    drive_msg = Float32()
                    drive_msg.data = float(var['value'])
                    self.drive_speed_sub.publish(drive_msg)

                # publish the ground speed
                elif var['name'] == "GND_SPEED":
                    gnd_msg = Float32()
                    gnd_msg.data = float(var['value'])
                    self.gnd_speed_sub.publish(gnd_msg)

                # publish the right wheel speed
                elif var['name'] == "RIGHT_DRIVE_SPEED" or var['name'] == "RIGHT_GROUND_SPEED":
                    right_msg = Float32()
                    right_msg.data = float(var['value'])
                    self.right_wheel_sub.publish(right_msg)

                # publish the left wheel speed
                elif var['name'] == "LEFT_DRIVE_SPEED" or var['name'] == "LEFT_GROUND_SPEED":
                    left_msg = Float32()
                    left_msg.data = float(var['value'])
                    self.left_wheel_sub.publish(left_msg)

                # publish the steering angle
                elif var['name'] == "STEERING_POS":
                    steering_msg = Float32()
                    steering_msg.data = float(var['value'])
                    self.steering_pub.publish(steering_msg)
            
    
def main(args=None):
    rclpy.init(args=args)

    bridge = Bridge()

    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()