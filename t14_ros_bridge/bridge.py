import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

from can import Bus

from . utils import load_config, handle_message

class Bridge(Node):

    def __init__(self):
        super().__init__('bridge')

        # declare the parameters
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bustype", "socketcan")
        self.declare_parameter("read_period", 0.02) # can reading period in seconds
        self.declare_parameter("config_path", "docs/ids.json")
        
        # create the publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 'ackermann', 10)
        self.right_wheel_sub = self.create_publisher(Float32, 'right_wheels_speed', 10)
        self.left_wheel_sub = self.create_publisher(Float32, 'left_wheels_speed', 10)
        self.drive_speed_sub = self.create_publisher(Float32, 'drive_speed', 10)
        self.gnd_speed_sub = self.create_publisher(Float32, 'gnd_speed', 10)

        # load the CAN configs
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.var_to_can, self.id_to_vars = load_config(self.config_path)

        # define the bus
        try:
            self.bus = Bus(channel=self.get_parameter("can_channel").value, bustype=self.get_parameter("can_bustype").value)
        except:
            raise ConnectionError("Could not connect to CAN bus!")
        
        # create the reading timer
        self.period = self.get_parameter("read_period").value
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):

        # read a message
        msg = self.bus.recv()

        # handle the received message
        values = handle_message(msg, self.id_to_vars, self.var_to_can)

        # publish the values
        # publish the ackermann msg
        if "GND_SPEED" in values and "STEERING_POS" in values:
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.steering_angle = values["STEERING_POS"]
            ack_msg.drive.speed = values["GND_SPEED"]
            self.ackermann_pub.publish(ack_msg)

        # publish the drive speed as redundancy
        if "DRIVE_SPEED" in values:
            drive_msg = Float32()
            drive_msg.data = values["DRIVE_SPEED"]
            self.drive_speed_sub.publish(drive_msg)

        # publish the ground speed
        if "GND_SPEED" in values:
            gnd_msg = Float32()
            gnd_msg.data = values["GND_SPEED"]
            self.gnd_speed_sub.publish(gnd_msg)

        # publish the right wheel speed
        if "RIGHT_DRIVE_SPEED" in values:
            right_msg = Float32()
            right_msg.data = values["RIGHT_DRIVE_SPEED"]
            self.right_wheel_sub.publish(right_msg)

        # publish the left wheel speed
        if "LEFT_DRIVE_SPEED" in values:
            left_msg = Float32()
            left_msg.data = values["LEFT_DRIVE_SPEED"]
            self.left_wheel_sub.publish(left_msg)
    
def main(args=None):
    rclpy.init(args=args)

    bridge = Bridge()

    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()