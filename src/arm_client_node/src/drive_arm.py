#!/usr/bin/env python
import time
import smbus
import rospy
from geometry_msgs.msg import Point
from arm_client_node.srv import *
from std_msgs.msg import Bool

bus = smbus.SMBus(1)
address = 0x04

def StringToBytes(val):
        retVal = []
        for c in val:
                retVal.append(ord(c))
        return retVal

def writeData(value):
	byteValue = StringToBytes(value)
	bus.write_i2c_block_data(address, 0x00, byteValue)
	return -1

def compose_byte_message(servo_num, angle):
	msg = ""
	msg+=str(servo_num)
	msg+=str(angle)
	return msg

def full_conf_msg(point):
	msg_array = []
	for i,pt in enumerate(point):
		msg_array.append(compose_byte_message(i+1,pt))
	return msg_array

def send_msgs(msgs):
	for msg in msgs:
		writeData(msg)
		time.sleep(0.1)

def conf_callback(req):
	print("received")
	data = req.conf
	data_array = [data.x, data.y, data.z]
	msgs = full_conf_msg(data_array)
	send_msgs(msgs)
	status = Bool()
	status.data = True
	return ConfResponse(status)

if __name__ == "__main__":
	rospy.init_node("left_arm_node")
	s = rospy.Service("conf_channel", Conf, conf_callback)
	rospy.spin()
