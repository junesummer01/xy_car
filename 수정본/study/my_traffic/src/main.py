#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from dbscan_driver import HullClusterDriver # hull_cluster_driver.py 파일에서 HullClusterDriver 클래스를 가져옵니다.

if __name__ == "__main__":
    """
    프로그램의 메인 실행 블록입니다.
    """
    try:
        # HullClusterDriver 클래스의 인스턴스(객체)를 생성합니다.
        # 이 시점에 __init__ 메소드가 호출되어 노드 초기화, 퍼블리셔/서브스크라이버 설정 등이 이루어집니다.
        driver_node = HullClusterDriver()
        
        # run 메소드를 호출하여 메인 루프(영상 처리 및 주행 제어)를 시작합니다.
        driver_node.run()
        
    except rospy.ROSInterruptException:
        # Ctrl+C 등 ROS 노드가 중단될 때 발생하는 예외를 처리합니다.
        pass