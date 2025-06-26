#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================

#=============================================
# AR 패지키지가 발행하는 토픽을 받아서 
# 제일 가까이 있는 AR Tag에 적힌 ID 값을 반환하는 함수
# 거리값과 좌우치우침값을 함께 반환
#=============================================
def check_AR(ar_msg):

    ar_data = ar_msg
    id_value = 99

    if (len(ar_data["ID"]) == 0):
        # 발견된 AR태그가 없으면 
        # ID값 99, Z위치값 500cm, X위치값 500cm로 리턴
        return 99, 500, 500  

    # 새로 도착한 AR태그에 대해서 아래 작업 수행
    z_pos = 500  # Z위치값을 500cm로 초기화
    x_pos = 500  # X위치값을 500cm로 초기화
    
    for i in range(len(ar_data["ID"])):
        # 발견된 AR태그 모두에 대해서 조사

        if(ar_data["DZ"][i] < z_pos):
            # 더 가까운 거리에 AR태그가 있으면 그걸 사용
            id_value = ar_data["ID"][i]
            z_pos = ar_data["DZ"][i]
            x_pos = ar_data["DX"][i]

    # ID번호, 거리값, 좌우치우침값 리턴
    return id_value, round(z_pos,2), round(x_pos,2)
