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

    # 왼쪽이 오른쪽보다 30센치 이상 멀리 있으면, 왼쪽으로
    if (left-30 > right):
        angle = -90
        
    # 왼쪽이 오른쪽보다 20센치 이상 멀리 있으면, 왼쪽으로
    elif (left-20 > right):
        angle = -60

    # 왼쪽이 오른쪽보다 10센치 이상 멀리 있으면, 왼쪽으로
    elif (left-10 > right):
        angle = -30
        
    # 왼쪽보다 오른쪽이 30센치 이상 멀리 있으면, 오른쪽으로
    elif (left < right-30):
        angle = +90
                
    # 왼쪽보다 오른쪽이 20센치 이상 멀리 있으면, 오른쪽으로
    elif (left < right-20):
        angle = +60
        
    # 왼쪽보다 오른쪽이 10센치 이상 멀리 있으면, 오른쪽으로
    elif (left < right-10):
        angle = +30

    # 위에 해당하지 않는 경우, 즉 차이가 10센치 이내라면 직진 주행
    else:
        angle = 0        

    angle = max(min(angle, 100.0), -100.0)
    print("angle:", angle)
    return angle
