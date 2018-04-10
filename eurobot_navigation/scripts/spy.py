#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point
import numpy as np
from scipy.optimize import least_squares
from sklearn.cluster import DBSCAN


MIN_INTENSITY = 3500
LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_X = -0.094
LIDAR_Y = 0.050
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)
FIELD_X = 3
FIELD_Y = 2
BEYOND_FIELD = 0.1
R = 0.08
visualization = False


def scan_callback(scan):
    lidar_data = np.array([np.array(scan.ranges), scan.intensities]).T
    landmarks = filter(lidar_data)

    if visualization == True:
        # create and pub PointArray of detected beacon points
        points = [Point(x=landmarks[i, 0], y=landmarks[i, 1], z=0) for i in range(len(landmarks))]
        array = PointCloud(points=points)
        array.header.frame_id = "map"
        pub_landmarks.publish(array)

    centers = []
    if landmarks.shape[0] > 0:
        # clustering
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(landmarks)
        labels = db.labels_
        unique_labels = set(labels)

        for l in unique_labels:
            if l == -1:
                # noise
                continue

            class_member_mask = (labels == l)

            center = get_center(landmarks[class_member_mask])
            centers.append(Point(x=center.x[0], y=center.x[1], z=0))
        
    # create and pub PointArray of detected centers
    array = PointCloud(points=centers)
    array.header.frame_id = "map"
    array.header.stamp = rospy.Time.now()
    pub_center.publish(array)


def filter(scan):
    """Filters scan to get only landmarks (bright) located on field."""
    ind = np.where(scan[:, 1] > MIN_INTENSITY)[0]
    a = LIDAR_DELTA_ANGLE * ind
    d = scan[ind, 0]
   
    x = d * np.cos(a + LIDAR_START_ANGLE) + LIDAR_X
    y = d * np.sin(a + LIDAR_START_ANGLE) + LIDAR_Y

    # inside field only
    ind = np.where(np.logical_and(np.logical_and(x < FIELD_X + BEYOND_FIELD, x > -BEYOND_FIELD), np.logical_and(y < FIELD_Y + BEYOND_FIELD, y > - BEYOND_FIELD)))

    x = x[ind]
    y = y[ind]
    return np.array([x, y]).T


def get_center(landmarks):
    #bounds = (point - np.array([0.1, 0.1]), point + np.array([0.15, 0.15])
    med = np.median(landmarks, axis=0)
    dist = np.sum(med ** 2) ** .5
    center_by_med = med + R * np.array([med[0] / dist, med[1] / dist])
    center = least_squares(fun, center_by_med, args=[landmarks])#, loss="linear", bounds = bounds, args = [landmarks], ftol = 1e-3)
    return center


def fun(point, landmarks):
    return np.sum((landmarks - point) ** 2, axis=1) ** .5 - R


if __name__ == "__main__":
    rospy.init_node("spy", anonymous=True)
    R = rospy.get_param("R")
    eps = rospy.get_param("clustering/eps")
    min_samples = rospy.get_param("clustering/min_samples")
    rospy.Subscriber("scan", LaserScan, scan_callback, queue_size = 1)
    pub_center = rospy.Publisher("detected_robots", PointCloud, queue_size=1)
    if visualization == True:
        pub_landmarks = rospy.Publisher("landmarks", PointCloud, queue_size=1)

    rospy.spin()
