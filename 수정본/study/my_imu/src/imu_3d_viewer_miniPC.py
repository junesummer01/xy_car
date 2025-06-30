#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Ctrl-C 시그널 핸들링을 위한 함수
def handle_close(event):
    plt.ioff()  # 인터랙티브 모드 종료
    plt.close()  # 창 닫기

Imu_msg = None

def imu_callback(data):
    global Imu_msg
    Imu_msg = [data.orientation.x, data.orientation.y, 
               data.orientation.z, data.orientation.w] 

def main():
    rospy.init_node("Imu_Print")
    rospy.Subscriber("imu", Imu, imu_callback)

    rospy.wait_for_message("imu", Imu)
    print("IMU Ready ----------") 

    # Define vertices of the cube

    vertices = np.array([[1, 0.5, -0.2], [1, -0.5, -0.2], [-1, -0.5, -0.2], [-1, 0.5, -0.2],
                         [1, 0.5, 0.2],  [1, -0.5, 0.2],  [-1, -0.5, 0.2],  [-1, 0.5, 0.2]])

    # Define edges of the cube
    edges = np.array([[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]])

    # Define colors for each edge
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 이벤트 핸들러 등록
    plt.gcf().canvas.mpl_connect('close_event', handle_close)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)

    while not rospy.is_shutdown():
        (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)
        print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (roll, pitch, yaw))

        # Define transformation matrix based on roll, pitch, yaw
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        # Apply transformation to vertices
        R = np.dot(R_z, np.dot(R_y, R_x))
        transformed_vertices = np.dot(vertices, R)

        # Clear previous plot and plot the transformed cube
        ax.clear()
        for edge_num, edge in enumerate(edges):
            ax.plot(transformed_vertices[edge, 0], transformed_vertices[edge, 1], 
                    transformed_vertices[edge, 2], color=colors[edge_num % len(colors)])

        # Set equal scales for x, y, and z axes
        ax.set_box_aspect((np.ptp(transformed_vertices[:,0]), np.ptp(transformed_vertices[:,1]), np.ptp(transformed_vertices[:,2])))
 
        plt.pause(0.01)
      

if __name__ == '__main__':
    main()

