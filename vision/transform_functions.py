import numpy as np
import math as m

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
