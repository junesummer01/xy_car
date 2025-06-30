# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

# import rospy
# from std_msgs.msg import Int32MultiArray

# class SonicDriver:
#     def __init__(self):
#         self.ultra_msg = None
#         rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultra_callback, queue_size=1)
#         rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
#         print("UltraSonic Ready ----------")

#     def ultra_callback(self, data):
#         self.ultra_msg = data.data

#     def get_drive_values(self):
#         if self.ultra_msg is None:
#             return 0, 0

#         left = self.ultra_msg[1]
#         right = self.ultra_msg[3]

#         if left > right:
#             if left > 5:
#                 return -10, 30
#             elif left < 5:
#                 return -30, 20
#             elif left < 3:
#                 return -90, 5
#             else:
#                 return -40, 5

#         elif right > left:
#             if right > 5:
#                 return 10, 30
#             elif right < 5:
#                 return 30, 20
#             elif right < 3:
#                 return 90, 5
#             else:
#                 return 40, 5

#         return 0, 23
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

class SonicDriver:
    def __init__(self):
        self.ultra_msg = None
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultra_callback)
        rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
        print("Ultrasonic Corner Driver Ready ----------")

    def ultra_callback(self, data):
        self.ultra_msg = data.data

    def get_drive_values(self):
        if self.ultra_msg is None:
            return 0, 0
        
        ok = 0
        left = self.ultra_msg[1]
        right = self.ultra_msg[3]

        if 0 < left < 45 and left < right:
            ok = 1
        elif 0 < right < 45 and left > right:
            ok = 2

        if ok == 1:
            speed = min(left,20)
            angle = 100 - 1.0 * left
            return angle, speed
        elif ok == 2:
            speed = min(right,20)
            angle = 100 - 1.0 * right
            return -angle, speed
        else:
            return 0, 30
