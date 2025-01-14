import cv2
import argparse
import apriltag
import time

# retrieving the image path
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to input image containing AprilTag")
ap.add_argument("-d", "--decimation", type=float, default=0.5, help="decimation factor")
args = vars(ap.parse_args())

decimation_factor = args["decimation"]
#Each reduction by a factor of 2 reduces runtime by factor of 8.8

image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.resize(gray, dsize=(0, 0), fx=decimation_factor, fy=decimation_factor)

detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

results = detector.detect(gray)

#draws lines and prints tags
for r in results:
    (ptA, ptB, ptC, ptD) = r.corners

    ptA = (int(ptA[0] / decimation_factor), int(ptA[1] / decimation_factor))
    ptB = (int(ptB[0] / decimation_factor), int(ptB[1] / decimation_factor))
    ptC = (int(ptC[0] / decimation_factor), int(ptC[1] / decimation_factor))
    ptD = (int(ptD[0] / decimation_factor), int(ptD[1] / decimation_factor))

    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)

    tag = str(r.tag_id)
    (cX, cY) = (int(r.center[0] / decimation_factor), int(r.center[1] / decimation_factor))
    cv2.putText(image, tag, (cX - 10, cY + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

cv2.imwrite("output.jpg", image)