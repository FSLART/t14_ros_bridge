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
        self.drive_speed_pub = self.create_publisher(Float32, 'drive_speed', 10)
        self.gnd_speed_pub = self.create_publisher(Float32, 'gnd_speed', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)

        self.pubs = {
            "STEERING_POS": self.steering_pub,
            "GND_SPEED": self.gnd_speed_pub,
            "DRIVE_SPEED": self.drive_speed_pub 
        }

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


    def timer_callback(self):

        # read a message without blocking
        while True:
            msg = self.bus.recv(0.0)

            # stop when the buffer is already empty
            if msg is None:
                break

            # handle the received message
            values = handle_message(msg, self.id_to_vars, self.var_to_can)

            # publish the values
            for var in values:

                val = var['value']
                if val is None:
                    continue
                val = float(val)
                # apply the scale and offset
                val = (val * float(var['scale'])) + float(var['offset'])

                pub = self.pubs.get(var['name'], None)
                if pub is not None:
                    msg = Float32()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.data = val
                    pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)

    bridge = Bridge()

    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()