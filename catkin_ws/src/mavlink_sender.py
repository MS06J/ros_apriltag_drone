import time
import numpy as np
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

def quaternion_to_rotation_matrix(q):
    # Extract the components of the quaternion
    w, x, y, z = q

    # Compute the rotation matrix
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])

    return rotation_matrix

def build_transformation_matrix(translation, quaternion):
    # Get the 3x3 rotation matrix from the quaternion
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # Create a 4x4 identity matrix
    transformation_matrix = np.identity(4)
    
    # Assign the rotation matrix to the upper-left 3x3 part of the transformation matrix
    transformation_matrix[:3, :3] = rotation_matrix
    
    # Assign the translation vector to the rightmost column of the transformation matrix
    transformation_matrix[:3, 3] = translation
    
    return transformation_matrix

def rotation_matrix_to_quaternion(matrix):
    # Ensure the matrix is 3x3
    R = matrix[:3, :3]

    # Calculate the trace of the matrix
    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return np.array([qw, qx, qy, qz])

def get_quaternion_from_transformation_matrix(matrix):
    # Extract the rotation matrix from the transformation matrix
    rotation_matrix = matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    return quaternion

class MavlinkSender(): 
    def __init__(self, ip='udp:192.168.1.142:14551'): #your PC's ip
        self.ip = ip
        self.source_system = 254
        # Connect to the vehicle
        connection = mavutil.mavlink_connection(self.ip, source_system = self.source_system, baud=57600)
        # Wait for the first heartbeat, so we know the system IDs
        print('waiting for heartbeat...')
        connection.wait_heartbeat(timeout=1)
        print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
        self.connection = connection

    def set_home(self, home_lat=27.380341 * 1e7, home_lon=33.632152 * 1e7, home_alt=488.0):
        # Set home position relative to current position (0 for global, 1 for current)
        use_current_position = 0

        # Send the MAV_CMD_DO_SET_HOME command
        connection = self.connection
        connection.mav.command_long_send(
            connection.target_system,    # Target system (0 for broadcast)
            connection.target_component, # Target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # Command
            0,  # Confirmation (0: First transmission)
            use_current_position,  # Param 1 (1 to use current position, 0 to use specified position)
            home_lat,  # Param 2 (Latitude)
            home_lon,  # Param 3 (Longitude)
            home_alt,  # Param 4 (Altitude)
            0, 0, 0    # Param 5, 6, 7 (unused)
        )

        # Confirm the command was received
        ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        print(ack_msg)
        time.sleep(0.1)

    def set_ekf_home(self, home_lat=27.380341 * 1e7, home_lon=33.632152 * 1e7, home_alt=488.0):
        # Send the SET_GPS_GLOBAL_ORIGIN message
        connection = self.connection
        connection.mav.set_gps_global_origin_send(
            connection.target_system,  # Target system (0 for broadcast)
            int(home_lat),             # Latitude in int32 (degrees * 1e7)
            int(home_lon),             # Longitude in int32 (degrees * 1e7)
            int(home_alt * 1000)       # Altitude in int32 (millimeters)
        )

        # Confirm the command was received (OPTIONAL) only support be some FCU
        ack_msg = connection.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        print(ack_msg)

    def send_pose(self, msg): #only support single tag in a frame
        #parameters for transformation from drone to world
        camera_yaw = 0 #yaw of camera w.r.t north in gradient
        tag_tilt = 0 #tilt of tag onboard. tilting toward right when viewing in front of the tag results in positive tile value
        tag_offset = np.array([0, 0, 0]) #offset of tag w.r.t center of drone in m, in tag coordinate x,y,z direction

        #Three transformation matrixs needed
        camera2world=build_transformation_matrix([0,0,0], [0.5, 0.5, 0.5, 0.5])
        tag2camera=0
        drone2tag=build_transformation_matrix(tag_offset, [0.5, 0.5, 0.5, -0.5])

        connection = self.connection
        try:
            pose = msg.pose_array.poses[0] #only consider the first tag
            # Define the vision position estimate data
            # Quaternion components representing the vehicle's attitude
            quat = pose.orientation
            q = QuaternionBase([quat.x, quat.y, quat.z, quat.w])

            x = pose.position.x  # X position in meters (forward)
            y = pose.position.y  # Y position in meters (right)
            z = pose.position.z # Z position in meters (down)

            covariance = [0] * 21  # Covariance matrix (not used here)

            tag2camera = build_transformation_matrix([x,y,z], q)
            drone2world = np.matmul(np.matmul(drone2tag, tag2camera), camera2world)
            q = get_quaternion_from_transformation_matrix(drone2world)
            x = drone2world[0, 3]
            y = drone2world[1, 3]
            z = drone2world[2, 3]

            usec = int(time.time() * 1e6)  # Current time in microseconds

            # Send the ATT_POS_MOCAP message
            connection.mav.att_pos_mocap_send(
                usec,      # Time since system boot in microseconds
                q,         # Quaternion components [w, x, y, z]
                x,         # X position in meters
                y,         # Y position in meters
                z,         # Z position in meters
                covariance # Pose covariance matrix (optional)
            )

            # Print a status message
            print(f"Sent ATT_POS_MOCAP: x={x}, y={y}, z={z}, q={q}")
        except Exception as error:
            print(error)

    def send_pose_dummy(self, x=1.0, y=2.0, z=-1.0):
        connection = self.connection
        # Define the vision position estimate data
        usec = int(time.time() * 1e6)  # Current time in microseconds

        # Quaternion components representing the vehicle's attitude
        q = QuaternionBase([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z] where w is the scalar component
        
        # X position in meters (forward)
        # Y position in meters (right) 
        # Z position in meters (down)

        covariance = [0] * 21  # Covariance matrix (not used here)

        # Send the VISION_POSITION_ESTIMATE message
        connection.mav.att_pos_mocap_send(
            usec,      # Time since system boot in microseconds
            q,         # Quaternion components [w, x, y, z]
            x,         # X position in meters
            y,         # Y position in meters
            z,         # Z position in meters
            covariance # Pose covariance matrix (optional)
        )

        # Print a status message
        print(f"Sent ATT_POS_MOCAP: x={x}, y={y}, z={z}, q={q}")

    def close(self):
        self.connection.close()

if __name__ == '__main__':
    sender = MavlinkSender()
    sender.set_ekf_home()
    sender.set_home()
    time.sleep(0.5)
    x = 0
    try:
        while(True):
            sender.send_pose_dummy(x)
            x=x+5
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Graceful exit on Ctrl+C
        print("Exiting...")
    finally:
        sender.close()

