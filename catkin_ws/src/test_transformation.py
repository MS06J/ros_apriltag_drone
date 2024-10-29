import numpy as np
from pymavlink.quaternion import QuaternionBase

def quaternion_to_rotation_matrix(q):
    """
    Converts a quaternion in (x, y, z, w) format into a 3x3 rotation matrix.

    Parameters:
    - q: Tuple or list of 4 elements (x, y, z, w) representing the quaternion.

    Returns:
    - A 3x3 NumPy array representing the rotation matrix.
    """
    # Extract the components of the quaternion
    x, y, z, w = q

    # Compute the rotation matrix directly
    rotation_matrix = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)]
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

    return np.array([qx, qy, qz, qw])

def get_quaternion_from_transformation_matrix(matrix):
    # Extract the rotation matrix from the transformation matrix
    rotation_matrix = matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    return quaternion

class Position():
    def __init__(self, x, y, z):
        self.x=x
        self.y=y
        self.z=z

    def __str__(self):
        return f"Position:\n  x: {self.x}\n  y: {self.y}\n  z: {self.z}"

class Orientation():
    def __init__(self, x, y, z, w):
        self.x=x
        self.y=y
        self.z=z
        self.w=w
    
    def __str__(self):
        return f"Orientation:\n  x: {self.x}\n  y: {self.y}\n  z: {self.z}\n  w: {self.w}"

class Pose():
    def __init__(self, pos, ori):
        self.position = pos
        self.orientation = ori
    
    def __str__(self):
        return (f"------\n"
                f"{self.position}\n"
                f"{self.orientation}\n"
                f"------")

def pose_trans(pose):
    #parameters for transformation from drone to world
    camera_yaw = 0 #yaw of camera w.r.t north in gradient
    tag_tilt = 0 #tilt of tag onboard. tilting toward right when viewing in front of the tag results in positive tile value
    tag_offset = np.array([0, 0, 0]) #offset of tag w.r.t center of drone in m, in tag coordinate x,y,z direction

    #Three transformation matrixs needed
    camera2world=build_transformation_matrix([0,0,0], [0.5, 0.5, 0.5, 0.5])
    tag2camera=0
    drone2tag=build_transformation_matrix(tag_offset, [0.5, 0.5, 0.5, -0.5])

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

    new_pose = Pose(Position(x,y,z), Orientation(*q))
    return new_pose

pose1 = Pose(Position(-0.12,0.0389,0.4579), 
             Orientation(-0.135,0.0305,0.04239,0.989))

pose2 = Pose(Position(1,0,0), 
             Orientation(0,0,0,1))
pose = pose2
print('coordinate before: \n', pose)
print('coordinate after: \n', pose_trans(pose))