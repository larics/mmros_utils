#!/usr/bin/env python

import rospy
import numpy as np
from mmros_utils.msg import CentroidPoseEstimate, CentroidPoseEstimateArray, DetectedCentroid, DetectedCentroidArray
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2

class TargetPoseEstimator:
    def __init__(self):
        rospy.init_node('target_pose_estimator', anonymous=True)
        self.detected_centroids = []
        self.detected_obj_sub = rospy.Subscriber('detected_objects', DetectedCentroidArray, self.detected_objects_cb)
        self.depth_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.dpth_points_cb)
        self.pose_pub = rospy.Publisher("detected_poses", CentroidPoseEstimateArray)
        self.detected_centroids_reciv = False
        self.dpth_reciv = False

    def detected_objects_cb(self, data):
        # Clear the previously detected centroids
        self.detected_centroids = []
        self.detected_centroids_reciv = True

        # Extract the detected centroids into paired tuples (px, py)
        for centroid in data.objects:
            self.detected_centroids.append((centroid.px, centroid.py))

    def dpth_points_cb(self, data): 
        self.pcl = data
        self.dpth_reciv = True

    def get_depths(self, pcl, indices, axis="z"):

        # Get current depths from depth cam --> TODO: Change read_points with cam_homography
        depths = pc2.read_points(pcl, [axis], False, uvs=indices)

        return depths

    def check_nan(self, val): 
        print(val)
        if np.isnan(np.array(val)): 
            return 1000.0
        else: 
            return float(val)

    def run(self):
        rate = rospy.Rate(10)  # Update rate, adjust as needed

        while not rospy.is_shutdown():
            # Process the detected centroids if needed
            start_condition = self.detected_centroids and self.dpth_reciv
            if start_condition:
                rospy.loginfo("Detected Centroids: {}".format(self.detected_centroids))
                cp_array = CentroidPoseEstimateArray()
                for i, axis in enumerate("xyz"):
                    gen_dpths =  self.get_depths(self.pcl, self.detected_centroids, axis="{}".format(axis))
                    # Assign values for each of the detected points
                    if axis == "x": 
                        for kx, x in enumerate(gen_dpths): 
                            cp_array.poses.append(CentroidPoseEstimate())
                            cp_array.poses[kx].pose.x = self.check_nan(x[0])
                    if axis == "y": 
                        for ky, y in enumerate(gen_dpths):
                            cp_array.poses[ky].pose.y = self.check_nan(y[0])
                    if axis == "z": 
                        for kz, z in enumerate(gen_dpths): 
                            cp_array.poses[kz].pose.z = self.check_nan(z[0])

                    self.pose_pub.publish(cp_array)

                    #_dpth = [i for i in gen_dpths]
                    #print("Calculated first point depth is: {}".format(_dpth))
                    
            else: 
                rospy.logwarn("Depth reciv: {}, Detection reciv: {}".format(self.dpth_reciv, self.detected_centroids_reciv))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TargetPoseEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
