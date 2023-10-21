from can import Bus
from utils import load_config, handle_message

# load the configs
var_to_can, id_to_vars = load_config()

# print the configs
print(var_to_can)
print("-------------")
print(id_to_vars)

# define the bus
try:
    bus = Bus(channel='can0', bustype='socketcan')
except:
    raise ConnectionError("Could not connect to CAN bus!")

while True:

    # read a message
    msg = bus.recv()

    # handle the received message
    vals = handle_message(msg, id_to_vars, var_to_can)

    print(vals)

