#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
last_left = 50.0
last_right = 50.0

#=============================================
# 장애물까지의 거리 값이 5센치~90센치인 경우엔 True 반환
# 그 이외의 값이면 (너무 가깝거나 너무 멀면) False 반환
#=============================================
def is_valid(val):
    return 5.0 <= val <= 90.0
	
#=============================================
# 장애물까지의 거리 값이 너무 가깝거나 멀면
# 앞서 저장해 놓은 거리값으로 대체 
#=============================================
def filter_distance(raw, last):
    return raw if is_valid(raw) else last

#=============================================
# 거리센서를 이용해서 장애물까지의 거리를 알아내서
# 장애물과 충돌하지 않으며 주행하도록 조향값을 반환 
#=============================================
def sonic_drive(ultra_msg, orig_angle):

    global last_left, last_right
    
    raw_left = float(ultra_msg[1])
    raw_right = float(ultra_msg[3])

    valid_left = is_valid(raw_left)
    valid_right = is_valid(raw_right)

    left = filter_distance(raw_left, last_left)
    right = filter_distance(raw_right, last_right)

    last_left = left
    last_right = right

    print(f"Left: {left} -- Right: {right}") 

    if (left < right):
        if (left < 40):
            angle = +(40-left) * 2.0
        
    elif (left > right):
        if (right < 40):
            angle = -(40-right) * 2.0
        
    angle = max(min(angle, 100.0), -100.0)
    print("angle:", angle)

    return angle 
