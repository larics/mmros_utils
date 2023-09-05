#!/usr/bin/env python

import rospy
from mmros_utils.msg import DetectedCentroid, DetectedCentroidArray
import random

class publishDummyCentroid:
    def __init__(self):
        rospy.init_node('centroid_publisher', anonymous=True)
        self.pub = rospy.Publisher('detected_objects', DetectedCentroidArray, queue_size=10)
        self.rate = rospy.Rate(1)  # Publish at 1 Hz
        self.max_w = 1280
        self.max_h = 720 

    def create_random_detected_centroid(self, id):
        centroid = DetectedCentroid()
        centroid.px = random.uniform(0, self.max_w)
        centroid.py = random.uniform(0, self.max_h)
        centroid.id = id
        return centroid

    def run(self):
        while not rospy.is_shutdown():
            detected_centroids = DetectedCentroidArray()

            for i in range(3):  # Populate with 5 objects for example
                detected_centroids.objects.append(self.create_random_detected_centroid(i))

            self.pub.publish(detected_centroids)
            rospy.loginfo("Published a DetectedCentroidArray message")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        pDC = publishDummyCentroid()
        pDC.run()
    except rospy.ROSInterruptException:
        pass
