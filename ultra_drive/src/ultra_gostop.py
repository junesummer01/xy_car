#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

motor_msg = xycar_motor()

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data 

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
def start():
    global motor

    rospy.init_node('ultra_driver')
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("Ultrasonic Ready ----------") 
    drive(0,0)
    time.sleep(2)

    while not rospy.is_shutdown():   
        ok = 0
        if (0 < ultra_msg[2] < 20):
            drive(angle=0, speed=0)
        else:    
            drive(angle=0, speed=18)

if __name__ == '__main__':
    start()
