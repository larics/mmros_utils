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
        self.uav_pose_sub = rospy.Subscriber('/falconsilver/vrpn_client/estimated_odometry', Odometry, self.uav_pose_cb)  
        self.pose_pub = rospy.Publisher("detected_poses", CentroidPoseEstimateArray)
        self.detected_centroids_reciv = False
        self.dpth_reciv = False
        
        # TODO: Create transformations 
        self.T_world_uav = None # TODO: that's from the
        pitch_ang = 60 
        # Remove cam rotation (pitch 60)
        self.T_cam_camr = self.getRotXT(np.radians(-pitch_angle))
        # Move cam to the base of the UAV
        self.T_uav_cam = np.array([[0, 0, 1, 0], 
                                   [-1, 0, 0, 0], 
                                   [0, -1, 0, 0], 
                                   [0, 0, 0, 1]])        

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
            
    def getRotXT(self, angle): 
        
        return np.array([[1, 0, 0, 0], 
                         [0, np.cos(angle), -np.sin(angle), 0], 
                         [0, np.sin(angle), np.cos(angle), 0], 
                         [0, 0, 0, 1]]
                         
    def createCentroidPositionArray(self, transform=None):
        cp_array = CentroidPoseEstimateArray()
        for i, axis in enumerate('xyz'): 
            gen_dpths = self.get_depths(self.pcl, self.detected_centroids, axis='{}'.format(axis))
            # Assign values for each of the detected points
            if axis == 'x':
                for kx, x in enumerate(gen_dpths): 
                    cp_array.poses.append(CentroidPoseEstimate())
                    cp_array.poses[kx].pose.x = self.check_nan(x[0])
            if axis == 'y':
                for ky, y in enumerate(gen_dpths): 
                    cp_array.poses[ky].pose.y = self.check_nan(y[0])
            if axis == 'z': 
                for kz,z in enumerate(gen_dpths): 
                    cp_array.poses[kz].pose.z = self.check_nan(z[0])
            if transform: 
                for i, pose in enumerate(cp_array.poses):
                    p = np.array([pose.x, pose.y, pose.z, 1])
                    p = np.matmul(self.T_cam_camr, p)
                    p_new = np.matmul(self.T_uav_cam, p) 
                    cp_array.poses[i].x = p_new[0]
                    cp_array.poses[i].y = p_new[1]
                    cp_array.poses[i].z = p_new[2]
        return cp_array
                    

    def run(self):
        rate = rospy.Rate(10)  # Update rate, adjust as needed
        while not rospy.is_shutdown():
            # Process the detected centroids if needed
            start_condition = self.detected_centroids and self.dpth_reciv
            if start_condition:
                rospy.loginfo("Detected Centroids: {}".format(self.detected_centroids))
                cp_array = createCentroidPositionArray(transform = True)
                self.pose_pub.publish(cp_array) 
            else: 
                rospy.logwarn("Depth reciv: {}, Detection reciv: {}".format(self.dpth_reciv, self.detected_centroids_reciv))
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TargetPoseEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
