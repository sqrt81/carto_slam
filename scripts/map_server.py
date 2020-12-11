#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/python2.7/dist-packages')

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3

import cv2

def BuildFakeMapData(map_file):
    map_img = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
    
    map_data = OccupancyGrid()
    map_data.header.frame_id = 'map'
    map_data.header.stamp = rospy.Time.now()
    map_data.info.map_load_time = rospy.Time.now()
    map_data.info.width = map_img.shape[1]
    map_data.info.height = map_img.shape[0]
    map_data.info.resolution = 1.
    map_data.info.origin.position.x = -125
    map_data.info.origin.position.y = -827
    map_data.info.origin.position.z = 0
    map_data.info.origin.orientation.x = 0
    map_data.info.origin.orientation.y = 0
    map_data.info.origin.orientation.z = 0
    map_data.info.origin.orientation.w = 1.
    map_data.data = list()
    
    for i in range(1, map_img.shape[0] + 1):
        for j in range(map_img.shape[1]):
            if map_img[- i, j] == 128:
                map_data.data.append(-1)
            else:
                map_data.data.append(100 - int(map_img[- i, j] / 2.55))
    
    return map_data


def main():
    rospy.init_node('map_server', anonymous=True)
    pub = rospy.Publisher("map", OccupancyGrid, queue_size=1, latch=True)
    map_file = rospy.get_param("~map_file")
    
    rospy.loginfo("loading map from %s", map_file)
    
    map_data = BuildFakeMapData(map_file)
    pub.publish(map_data)
    
    rospy.spin()

if __name__ == '__main__':
    main()
