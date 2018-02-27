#!/usr/bin/env python
import sys
import time
import rospy
import numpy as np
#~ from imu_test.msg import imuData
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

from Adafruit_BNO055 import BNO055

def init_BNO():
	bno = BNO055.BNO055(serial_port='/dev/ttyUSB0')
	
	# Initialize the BNO055 and stop if something went wrong.
	if not bno.begin():
		raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

	# Print system status and self test result.
	status, self_test, error = bno.get_system_status()
	print('System status: {0}'.format(status))
	print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
	# Print out an error if system status is in error mode.
	if status == 0x01:
		print('System error: {0}'.format(error))
		print('See datasheet section 4.3.59 for the meaning.')
	# Print BNO055 software revision and other diagnostic data.
	sw, bl, accel, mag, gyro = bno.get_revision()
	print('Software version:   {0}'.format(sw))
	print('Bootloader version: {0}'.format(bl))
	print('Accelerometer ID:   0x{0:02X}'.format(accel))
	print('Magnetometer ID:    0x{0:02X}'.format(mag))
	print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
	
	return bno


def imuRead(bno):
	# Read the Euler angles for heading, roll, pitch (all in degrees).
	#~ heading, roll, pitch = bno.read_euler()
	# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
	#~ sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
	#~ print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          #~ heading, roll, pitch, sys, gyro, accel, mag))
	# Other values you can optionally read:
    # Orientation as a quaternion:
	x_qt,y_qt,z_qt,w_qt = bno.read_quaternion()
	q = Quaternion()
	q.x = x_qt
	q.y = y_qt
	q.z = z_qt
	q.w = w_qt
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
	#~ x_mag,y_mag,z_mag = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
	x_gyro,y_gyro,z_gyro = bno.read_gyroscope()
	#~ convert to rad/s
	x_gyro = x_gyro/180*np.pi
	y_gyro = y_gyro/180*np.pi
	z_gyro = z_gyro/180*np.pi
	ang_accel = Vector3()
	ang_accel.x = x_gyro
	ang_accel.y = y_gyro
	ang_accel.z = z_gyro
    # Accelerometer data (in meters per second squared):
	x_accel,y_accel,z_accel = bno.read_accelerometer()
	lr_accel = Vector3()
	lr_accel.x = x_accel
	lr_accel.y = y_accel
	lr_accel.z = z_accel
	print("x: {0}".format(lr_accel.x))
	print("y: {0}".format(lr_accel.y))
	print("z: {0}".format(lr_accel.z))
	# Linear acceleration data (i.e. acceleration from movement, not gravity--
	# returned in meters per second squared):
	#x,y,z = bno.read_linear_acceleration()
	# Gravity acceleration data (i.e. acceleration just from gravity--returned
	# in meters per second squared):
	#x,y,z = bno.read_gravity()
	return q, ang_accel, lr_accel

def imuPub(bno):
	pub = rospy.Publisher('/imu0', Imu, queue_size=200)
	rospy.init_node('imu_BNO_pub', anonymous = True)
	r = rospy.Rate(20)
	msg = Imu()
	
	while not rospy.is_shutdown():
		#print('publishing imu data')
		q, ang_accel, lr_accel = imuRead(bno)
		
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'BNO055'
		msg.orientation = q
		msg.angular_velocity = ang_accel
		msg.linear_acceleration = lr_accel
		
		pub.publish(msg)
		r.sleep()

if __name__ == '__main__':
	bno = init_BNO()
	imuPub(bno)

		
		
		
		
		
