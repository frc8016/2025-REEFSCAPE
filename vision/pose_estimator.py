import apriltag
import cv2
import numpy as np
import math
import argparse
import apriltag_transform as at
import apriltag_cords as ac

#PROPERTIES
tag_family = "tag36h11"
april_tag_width = 0.1651

focal_length_pixels = (1314, 1304)
camera_resolution = (1280, 800)
camera_center = (640, 400)
camera = (focal_length_pixels[0], focal_length_pixels[1], camera_center[0], camera_center[1])

decimation = 1
e0_threshhold = 0.1
e1_threshhold = 0.1

image = cv2.imread("apriltagrobots_overlay.jpg", cv2.IMREAD_GRAYSCALE)
image = cv2.resize(image, (int(image.shape[1]/decimation), int(image.shape[0]/decimation)))

options = apriltag.DetectorOptions(families=tag_family)
detector = apriltag.Detector(options)

apriltags = detector.detect(image)

global_poses = []

for tag in apriltags:
    id = tag.tag_id
    pose, e0, e1 = detector.detection_pose(tag, camera, april_tag_width)

    if e0 > e0_threshhold and e1 > e1_threshhold:
        global_poses.append((np.linalg.inv(np.array(ac.swap_z_y_axis(pose))) @ at.apriltag_tranform[id]), e0, e1)

if len(global_poses) == 0:
    print("No AprilTags found")
elif len(global_poses) == 1:
    print("One AprilTag found")
    print("Pose: ", global_poses[0][0])
    print("Error: ", global_poses[0][1], global_poses[0][2])
elif len(global_poses) == 2:
    print("Two AprilTags found")
    # Print mean of both poses
    mean_pose = (global_poses[0][0] + global_poses[1][0]) / 2
    print("Mean Pose: ", mean_pose)
else:
    # Sort the poses based on the sum of e0 and e1 errors
    global_poses.sort(key=lambda x: x[1] + x[2])
    
    # Take the mean of the two poses with the lowest errors
    mean_pose = (global_poses[0][0] + global_poses[1][0]) / 2
    
    print("More than two AprilTags found")
    print("Mean Pose of two lowest error readings: ", mean_pose)