def parse_get_pose(return_msg):
    """
    Parse the return message from GetPose() to extract the Cartesian coordinates.

    Parameters:
    return_msg (str): The return message string from GetPose()

    Returns:
    tuple: a tuple containing x, y, z, and r values.
    """
    try:
        # Extract the substring containing the coordinates
        coords_list = return_msg.split(",")[1:5]        
        coords_list[0] = coords_list[0].strip("{")

        # Convert the string values to floats
        x, y, z, r = map(float, coords_list)

        # Return the coordinates in a dictionary
        return x, y, z, r
    except (IndexError, ValueError) as e:
        print(f"Error parsing return message: {e}")
        return None
    
x, y, z, r = parse_get_pose("0,{273.902090,9.995497,80.141739,-180.000198,0.000000,0.000000},GetPose()")
print(x, y, z, r)