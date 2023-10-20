from can import Message
import json

# load IDs from a file
def load_config(path: str = "docs/ids.json") -> dict:

    # open the file
    try:
        f = open(path)
    except:
        raise FileNotFoundError("Configuration file not found!")
    
    # read the file JSON
    raw_json = f.read()

    # parse the JSON
    try:
        config = json.loads(raw_json)
    except:
        raise ValueError("Invalid JSON file encoding!")
    
    return config


# extract a value from a message
# the tuple are the starting and ending bytes of the value
def extract_value_from_msg(message: Message, byte_range: tuple) -> int:
    
    # verify the range
    if byte_range[0] > byte_range[1]:
        raise ValueError("Invalid byte range! The first value must be smaller than the second value")
    if byte_range[0] < 0 or byte_range[1] > 8:
        raise ValueError("Invalid byte range! CAN frames have 8 bytes")
    
    # extract the value
    value = 0
    # iterate the byte range
    for i in range(byte_range[0], byte_range[1] + 1):
        # "shift" and "or" each byte
        value += message.data[i] << (8 * (byte_range[1] - i))

    return value
