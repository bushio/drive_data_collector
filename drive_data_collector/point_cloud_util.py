from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sys
import time
import array

def point_cloud2_to_array(msg):
    """
    Convert a sensor_msgs/PointCloud2 message to a NumPy array. The fields
    in the PointCloud2 message are mapped to the fields in the NumPy array
    as follows:
    * x, y, z -> X, Y, Z
    * intensity -> I
    * other fields are ignored
    """
    intensity_flag = False

    for field in msg.fields:           
        if field.name == "intensity":
            intensity_offset = field.offset
            intensity_flag = True

        if field.name == "ring":
            ring_offset = field.offset
            ring_flag = True

    # Convert the PointCloud2 message to a NumPy array
    pc_data = np.frombuffer(
        msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = pc_data[:, 0:12].copy().view(dtype=np.float32).reshape(-1, 3)

    if intensity_flag:
        intensity = pc_data[:, intensity_offset:intensity_offset + 2].copy().view(dtype=np.uint16)

    # return the arrays in a dictionary
    if intensity_flag:
        return {"xyz": xyz, "intensity": intensity}

    else:
        return {"xyz": xyz}

def array_to_point_cloud2(np_array, frame_id='base_link'):
    """
    Convert a numpy array to a PointCloud2 message. The numpy array must have a "xyz" field
    and can optionally have a "intensity" field.
    """

    # Check if the "intensity" field is present
    intensity_flag = "intensity" in np_array.keys()

    np_array["xyz"] = np.nan_to_num(np_array["xyz"])

    if intensity_flag:
        np_array["intensity"] = np.nan_to_num(np_array["intensity"])

    # Create the PointCloud2 message
    msg = PointCloud2()
    header= Header()
    header.frame_id = "base_link"

    current_time = time.time()
    msg.header = header
    msg.header.stamp.sec = int(current_time)
    msg.header.stamp.nanosec = int((current_time - msg.header.stamp.sec) * 1e9)
    msg.height = 1
    msg.width = np_array["xyz"].shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

    msg.fields.append(PointField(name='intensity', 
                                offset=12,
                                datatype=PointField.UINT8, 
                                count=1))
    msg.is_bigendian = sys.byteorder != 'little'

    # Check if message is dense
    msg.is_dense = not np.isnan(np_array["xyz"]).any()

    # Calculate the point_step and row_step
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    memory_view = memoryview(np.hstack([np_array["xyz"].astype(np.float32), 
                                        np_array["intensity"].astype(np.uint8)]).tobytes())

    if memory_view.nbytes > 0:
        array_bytes = memory_view.cast("B")
    else:
        # Casting raises a TypeError if the array has no elements
        array_bytes = b""

    as_array = array.array("B")
    as_array.frombytes(array_bytes)
    msg.data = as_array

    return msg
