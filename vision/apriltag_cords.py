import numpy as np

apriltag_cords = {
    1:  (657.37,  25.80,   58.50,  126,  0),
    2:  (657.37,  291.20,  58.50,  234,  0),
    3:  (455.15,  317.15,  51.25,  270,  0),
    4:  (365.20,  241.64,  73.54,  0,    30),
    5:  (365.20,  75.39,   73.54,  0,    30),
    6:  (530.49,  130.17,  12.13,  300,  0),
    7:  (546.87,  158.50,  12.13,  0,    0),
    8:  (530.49,  186.83,  12.13,  60,   0),
    9:  (497.77,  186.83,  12.13,  120,  0),
    10: (481.39,  158.50,  12.13,  180,  0),
    11: (497.77,  130.17,  12.13,  240,  0),
    12: (33.51,   25.80,   58.50,  54,   0),
    13: (33.51,   291.20,  58.50,  306,  0),
    14: (325.68,  241.64,  73.54,  180,  30),
    15: (325.68,  75.39,   73.54,  180,  30),
    16: (235.73, -0.15,    51.25,  90,   0),
    17: (160.39,  130.17,  12.13,  240,  0),
    18: (144.00,  158.50,  12.13,  180,  0),
    19: (160.39,  186.83,  12.13,  120,  0),
    20: (193.10,  186.83,  12.13,  60,   0),
    21: (209.49,  158.50,  12.13,  0,    0),
    22: (193.10,  130.17,  12.13,  300,  0),
}

def Pose_to_global(x, y, z, theta, phi, id):
    # (x, y, z) is the position of the tag relative to the camera
    # (theta, phi) is the orientation of the tag relative to the camera
    # theta is horizontal angle, phi is vertical angle with (0,0) being dead on after translation
    
    if id not in apriltag_cords:
        raise ValueError("Invalid AprilTag ID")
    
    tag_x, tag_y, tag_z, tag_theta, tag_phi = apriltag_cords[id]
    
    # Convert angles to radians
    theta_rad = np.radians(theta)
    phi_rad = np.radians(phi)
    tag_theta_rad = np.radians(tag_theta)
    tag_phi_rad = np.radians(tag_phi)
    
    # Rotation matrices
    R_theta = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0],
        [np.sin(theta_rad), np.cos(theta_rad), 0],
        [0, 0, 1]
    ])
    
    R_phi = np.array([
        [1, 0, 0],
        [0, np.cos(phi_rad), -np.sin(phi_rad)],
        [0, np.sin(phi_rad), np.cos(phi_rad)]
    ])
    
    R_tag_theta = np.array([
        [np.cos(tag_theta_rad), -np.sin(tag_theta_rad), 0],
        [np.sin(tag_theta_rad), np.cos(tag_theta_rad), 0],
        [0, 0, 1]
    ])
    
    R_tag_phi = np.array([
        [1, 0, 0],
        [0, np.cos(tag_phi_rad), -np.sin(tag_phi_rad)],
        [0, np.sin(tag_phi_rad), np.cos(tag_phi_rad)]
    ])
    
    # Combine rotations
    R = R_tag_theta @ R_tag_phi @ R_theta @ R_phi
    
    # Translate position
    relative_position = np.array([x, y, z])
    global_position = np.array([tag_x, tag_y, tag_z]) + R @ relative_position
    global_position = tuple(global_position)
    
    # Combine orientations
    global_theta = (tag_theta + theta) % 360
    global_phi = (tag_phi + phi) % 360
    
    #returns global position and orientation as x, y, z, theta, phi
    return global_position[0], global_position[1], global_position[2], global_theta, global_phi