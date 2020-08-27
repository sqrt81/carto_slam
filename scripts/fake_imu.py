#!/usr/bin/python

import sys

sys.path.append('/usr/local/lib/python2.7/dist-packages')

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3

def BuildFakeImuData():
    imu_data = Imu()
    imu_data.header.frame_id = 'imu_link'
    imu_data.orientation.x = 0
    imu_data.orientation.y = 0
    imu_data.orientation.z = 0
    imu_data.orientation.w = 1
    imu_data.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 1000]
    imu_data.angular_velocity.x = 0
    imu_data.angular_velocity.y = 0
    imu_data.angular_velocity.z = 0
    imu_data.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 1000]
    imu_data.linear_acceleration.x = 0
    imu_data.linear_acceleration.y = 0
    imu_data.linear_acceleration.z = 9.8
    imu_data.linear_acceleration_covariance = [1000, 0, 0, 0, 1000, 0, 0, 0, 0.1]
    
    return imu_data


def main():
    pub = rospy.Publisher("Imu", Imu, queue_size=1)
    rospy.init_node('fake_imu', anonymous=True)
    rate = rospy.Rate(1)
    header_cnt = 0
    
    rate.sleep()
    rate.sleep()
    
    imu_data = BuildFakeImuData()
    
    while not rospy.is_shutdown():
        imu_data.header.seq = header_cnt
        imu_data.header.stamp = rospy.Time.now()
        header_cnt += 1
        pub.publish(imu_data)
        rate.sleep()

if __name__ == '__main__':
    main()
