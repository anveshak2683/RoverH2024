#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import LaserScan, PointCloud2

class LidarToPointCloud:
    def __init__(self) -> None:
        self.lp = lg.LaserProjection()
        rospy.Subscriber("/galileo/laser_scan", LaserScan, self.laser_callback)
        self.pc_pub = rospy.Publisher("/points", PointCloud2, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def convert_pc2_to_open3d(self, pointcloud):
        points_list = []
        for p in pc2.read_points(pointcloud, skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array(points_list)))
    
    def laser_callback(self, msg):
        pc2_msg = self.lp.projectLaser(msg)
        self.pc_pub.publish(pc2_msg)
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("lidar_pointcloud", anonymous=False)
        lp = LidarToPointCloud()
        lp.run()
    except rospy.ROSInterruptException:
        pass
    