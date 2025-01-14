import numpy as np
import math as m

# Dictionary of AprilTag IDs to their (x, y, z, alpha, beta) coordinates
# Alpha is the yaw angle, beta is the pitch angle in radians
# Positive yaw is to the right, positive pitch is down

apriltag_cords = {
    1:  (657.37,  025.80,  58.50,  2.19911,  0.000000),
    2:  (657.37,  291.20,  58.50,  4.08407,  0.000000),
    3:  (455.15,  317.15,  51.25,  4.71239,  0.000000),
    4:  (365.20,  241.64,  73.54,  0.00000,  0.523599),
    5:  (365.20,  075.39,  73.54,  0.00000,  0.523599),
    6:  (530.49,  130.17,  12.13,  5.23599,  0.000000),
    7:  (546.87,  158.50,  12.13,  0.00000,  0.000000),
    8:  (530.49,  186.83,  12.13,  1.04720,  0.000000),
    9:  (497.77,  186.83,  12.13,  2.09440,  0.000000),
    10: (481.39,  158.50,  12.13,  3.14159,  0.000000),
    11: (497.77,  130.17,  12.13,  4.18879,  0.000000),
    12: (33.510,  025.80,  58.50,  0.94248,  0.000000),
    13: (33.510,  291.20,  58.50,  5.34071,  0.000000),
    14: (325.68,  241.64,  73.54,  3.14159,  0.523599),
    15: (325.68, -075.39,  73.54,  3.14159,  0.523599),
    16: (235.73, -000.15,  51.25,  1.57080,  0.000000),
    17: (160.39,  130.17,  12.13,  4.18879,  0.000000),
    18: (144.00,  158.50,  12.13,  3.14159,  0.000000),
    19: (160.39,  186.83,  12.13,  2.09440,  0.000000),
    20: (193.10,  186.83,  12.13,  1.04720,  0.000000),
    21: (209.49,  158.50,  12.13,  0.00000,  0.000000),
    22: (193.10,  130.17,  12.13,  5.23599,  0.000000),
}

#Creates a pitch-yaw transformation matrix as 4x4 numpy array
def pitch_yaw_transform(alpha, beta):
    #Returns a 4x4 affine transformation matrix that rotates by alpha and beta
    return np.array(
        [ [m.cos(alpha)*m.cos(beta), -m.sin(alpha), m.cos(alpha)*m.sin(beta), 0],
         
          [m.sin(alpha)*m.cos(beta),  m.cos(alpha), m.sin(alpha)*m.sin(beta), 0],

          [-m.sin(beta),              0,            m.cos(beta)             , 0],
          
          [0,                         0,            0                       , 1]
        ]
    )
#Creates a translation matrix as 4x4 numpy array
def translation_matrix(translation):
    return np.array(
        [ [1, 0, 0, translation[0]],
          [0, 1, 0, translation[1]],
          [0, 0, 1, translation[2]],
          [0, 0, 0, 1]
        ]
    )

#Combines the pitch-yaw transformation and translation matrix
def transform(pose):
    x, y, z, alpha, beta = pose
    return  pitch_yaw_transform(alpha, beta) @ translation_matrix((x, y, z))

#Swaps the z and y axis of a 4x4 matrix to account for camera basis(with z pointing out of the camera)
def swap_zy_row(matrix):
    swap_matrix = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    return swap_matrix @ matrix

#Extracts the 2d pose from a 4x4 transformation matrix because robot has no vertical movment or pitch
def extract_2d(transformation):
    return np.delete(np.delete(transformation, 2, axis=0), 2, axis=1)
