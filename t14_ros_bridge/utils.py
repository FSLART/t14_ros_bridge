from can import Message
import json

# load IDs from a file
def load_config(path: str = "docs/ids.json"):

    # open the file
    try:
        f = open(path)
    except:
        raise FileNotFoundError("Configuration file not found!")
    
    # read the file JSON
    raw_json = f.read()

    # parse the JSON
    # relates variable names to CAN IDs and bytes
    try:
        var_to_can = json.loads(raw_json)
    except:
        raise ValueError("Invalid JSON file encoding!")
    
    id_to_vars = {}
    for var in var_to_can:
        if not var_to_can[var]['ID'] in id_to_vars:
            id_to_vars[var_to_can[var]['ID']] = [var]
        else:
            id_to_vars[var_to_can[var]['ID']].append(var)

    return var_to_can, id_to_vars


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

# handle an incoming message
# receives the message, a mapping of IDs to variable names, and a mapping of variable names to CAN IDs and bytes
def handle_message(message: Message, id_to_vars: dict, var_to_can: dict):
    
    # if the ID is not used by any variable, discard it
    if not hex(message.arbitration_id) in id_to_vars:
        return
    
    # get the variable names that use this ID
    variables = id_to_vars[hex(message.arbitration_id)]

    # extract each of the variables to a list
    values = []
    for var in variables:
        new_val = {
            'name': var,
            'value': extract_value_from_msg(message, tuple(var_to_can[var]['bytes']))
        }
        values.append(new_val)
