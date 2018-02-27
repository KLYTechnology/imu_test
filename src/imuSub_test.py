#!/usr/bin/env python
import sys
import rospy
from imu_test.msg import imuData

def callback(data):
    #~ rospy.loginfo("%s is age: %d" % (data.name, data.age))
	print('x_accel={0:0.2F} y_accel={1:0.2F} z_accel={2:0.2F}'.format(data.x_accel, data.y_accel, data.z_accel))
	print('x_gyro={0:0.2F} y_gyro={1:0.2F} z_gyro={2:0.2F}'.format(data.x_gyro, data.y_gyro, data.z_gyro))
	print('x_mag={0:0.2F} y_mag={1:0.2F} z_mag={2:0.2F}\n'.format(data.x_mag, data.y_mag, data.z_mag))

def listener():
    rospy.init_node('imu_Sub_test', anonymous=True)
    rospy.Subscriber('/imuData_raw', imuData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
