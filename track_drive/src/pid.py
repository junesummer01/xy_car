#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np

#=============================================
# 프로그램에서 사용할 PID 클래스
#=============================================  
class PID:

    # 클래스 생성 초기화 함수 (게인값과 오차값 초기화)
    def __init__(self, kp, ki, kd):
        # PID 게인 초기화
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        # 이전 오차 초기화
        self.cte_prev = None  # None으로 설정하여 초기 상태 확인
        # 각 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        # 적분 오차 제한값 설정
        self.i_min = -10
        self.i_max = 10

    def pid_control(self, cte, dt=1.0):
        # 초기 호출 시 이전 오차를 현재 오차로 설정
        if self.cte_prev is None:
            self.cte_prev = cte

        # 미분 오차 계산 (dt 고려)
        self.d_error = (cte - self.cte_prev) / dt

        # 비례 오차 계산
        self.p_error = cte

        # 적분 오차 계산 및 제한 적용
        self.i_error += cte * dt
        self.i_error = max(min(self.i_error, self.i_max), self.i_min)

        # 이전 오차 업데이트
        self.cte_prev = cte

        # PID 제어 출력 계산
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
