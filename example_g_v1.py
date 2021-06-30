from math import pi
import time
import sys
import csv
from typing import Union

import urx  # urx 라이브러리 가져오기
import logging
import numpy as np
import sympy as sp
from urx import URRobot, Robot
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
if sys.version_info[0] < 3:  # support python v2
    input = 2

def wait():
    if do_wait:
        print("Click enter to continue")
        input()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    do_wait = True
    if len(sys.argv) > 1:
        do_wait = False

    rob: Union[Robot, URRobot] = urx.Robot("192.168.56.5")  # 연결 설정  @@@@@@@@@ IP주소 변경
    # rob = urx.Robot(“local host”)
    robotiqgrip = Robotiq_Two_Finger_Gripper(robot=rob)
    rob.set_tcp((0, 0, 0.289, 0, 0, 0))  # 로봇 플랜지를 공구 팁 변형으로 설정 TCP위치
    rob.set_payload(0.98, (0, 0, 0))  # payload를 kg 단위로 설정
    # Robot payload = 0.09kg(bracket) + 0.65kg(RG2) + ??? (work piece)
    try:
        # ----------------- 코딩 가능 구간 ---------------------
        f = open('D:\\sampling.csv', 'w', newline="")  # csv 파일 작성 가능하게 해주는 명령어
        # open(' 경로 + 파일명(변경가능).csv', 'w', newline="")
        makefile = csv.writer(f)

        makefile.writerow(['시간', '베이스', '숄더', '엘보우', '손목1', '손목2', '손목3'])
        # writerow(배열항목) 파일에 새로운row(열)로 배열항목을 추가 (줄바꿈)
        # 생성하는 파일의 첫 행에 들어갈 7개 항목 입력. ,로 열 추가

         # 1970년 1월 1일 자정 이후로 누적된 초를 float 단위로 반환
        t_begin = time.time()  # 현재 시간 읽어오기
        t_now = time.time()

        # @@@@@@@@@@@@@@@@@ 이 위로는 절대 삭제 금지
        """
        # 값을 읽어오는 함수
        joint = rob.getj()  # 로봇팔 관절마다의 각(라디안)을 가져옵니다 [베이스, 숄더, 엘보우, 손목1, 손목2, 손목3]
        tcp_po = rob.getl()  # TCP의 위치를 가져옵니다. [ x, y, z, Rx, Ry, Rz ] (x,y,z는 m단위) (R~ 의 단위도 라디안)
        tcp_cur = rob.get_pose()  # get current transform from base to to tcp
        tool_pos = rob.get_pos()  # get tool tip pos(x,y,z) in base coordinate system
        tool_ori = rob.get_orientation()  # get tool orientation in base coordinate system

        print("get joint radius : getj()")
        print(joint)
        print(" ")
        print("get tcp position : getl()")
        print(tcp_po)
        print(" ")
        print("get current transform from base to to tcp : get_pose()")
        print(tcp_cur)
        print(" ")
        print("get tool tip pos(x,y,z) in base coordinate system : get_pos()")
        print(tool_pos)
        print(" ")
        print("get tool orientation in base coordinate system : get_orientation()")
        print(tool_ori)
        print(" ")
        print("Move URsim view front of me")
        wait()  # 엔터를 입력받을때까지 대기. 화면에는 Click enter to continue 출력.
        wait()

        t_now = time.time() - t_begin # 초기 시간으로부터 지금까지의 시간
        joint_now = rob.getj()  # 현재의 조인트 값을 받아옴
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # 시간 및 각 조인트들의 값을 파일에 기록

        # 움직이는 명령어
        rob.movej([1.570796, -1.570796, 0, -1.570796, 0, 0], acc=1.5, vel=1, wait=False)
        # 각 관절에 라디안으로 값을 주어 움직이는 명령어 movej
        # ( [베이스, 숄더, 엘보우, 손목1, 손목2, 손목3], acc=가속도 rad/s^2, vel= 속도 rad/s, wait=False)
        print("move position.")
        wait()

        t_now = time.time() - t_begin # 초기 시간으로부터 지금까지의 시간
        joint_now = rob.getj()  # 현재의 조인트 값을 받아옴
        makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
        # 시간 및 각 조인트들의 값을 파일에 기록
        """
        pre = [0, -1.57, 0, -1.57, 0, 0]
        rob.movej(pre, acc=1.5, vel=1, wait=False)
        print("Move by time")
        wait()  # 아래 창에 "Move by time"이 나오고 수식에 따른 움직임(아래 eq1_~)을 주기 이전에 enter키를 누르기 전까지 대기
        t_begin = time.time()  # 현재 시간 읽어오기
        x = sp.symbols('x')  # x를 미지수 변수로 사용하겠다고 선언언

        # @@@@@@@@@@@@@@@@@@본격 사용하게 될 부분 (아래는 경로1)

        t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
        eq1_1 = 0 * x
        eq1_2 = -1.57 + 0 * x
        eq1_3 = -1.57 * sp.sin(0.314 * x)
        eq1_4 = -1.57 + 0 * x
        eq1_5 = 0 * x
        eq1_6 = 0 * x

        while t_now <= 5:  # while문 내부를 5초동안 동작
            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
            joint_now = rob.getj()  # 현재의 조인트 값을 받아옴
            makefile.writerow(
                [t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
            # 시간 및 각 조인트들의 값을 파일에 기록

            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간

            rob.movej([eq1_1.subs(x, t_now), eq1_2.subs(x, t_now), eq1_3.subs(x, t_now), eq1_4.subs(x, t_now),
                       eq1_5.subs(x, t_now), eq1_6.subs(x, t_now)], acc=1, vel=5, wait=False)

            # 지정된 조인트 각으로부터 (시간*상수)값을 더해주면서 로봇팔 구동
            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
            time.sleep(0.5)  # 대기시간 0.5초

        # @@@@@@@@@@@@@@@@@@@@@@@@ 위까지 경로 1

        time.sleep(1)  # 대기시간 2초
        robotiqgrip.gripper_action(255)  # 그리퍼 완전 열기

        # @@@@@@@@@@@@@@@@@@@@@아래부터 경로2

        t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
        eq2_1 = 1.57 * sp.sin(0.314 * x)-1.57
        eq2_2 = -1.57 + 0 * x
        eq2_3 = -1.57 + 0 * x
        eq2_4 = -1.57 + 0 * x
        eq2_5 = 0 * x
        eq2_6 = 0 * x

        while t_now <= 10:  # while문 내부를 5초동안 동작 (10초 - 이전루프시간 5초)
            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
            time_while_2 = t_now - 5  # 만약 해당 루프만의 시간을 따로 측정 혹은 활용하고 싶다면 time_while_2를 활용
                                    # 내용은 초기부터 현재까지의 시간 - 5초 (여기서 5초는 이전 루프에서 5초를 사용했기 때문)
                                    # ex 현재까지 8초라면 time_while_2의 값은 8초 - 5초이므로 3초
            joint_now = rob.getj()  # 현재의 조인트 값을 받아옴
            makefile.writerow([t_now, joint_now[0], joint_now[1], joint_now[2], joint_now[3], joint_now[4], joint_now[5]])
            # 시간 및 각 조인트들의 값을 파일에 기록

            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간

            rob.movej([eq2_1.subs(x, t_now), eq2_2.subs(x, t_now), eq2_3.subs(x, t_now), eq2_4.subs(x, t_now),
                       eq2_5.subs(x, t_now), eq2_6.subs(x, t_now)], acc=1, vel=5, wait=False)

        # 지정된 조인트 각으로부터 (시간*상수)값을 더해주면서 로봇팔 구동
            t_now = time.time() - t_begin  # 초기 시간으로부터 지금까지의 시간
            time.sleep(0.5)  # 대기시간 0.5초

        # @@@@@@@@@@@@@@@@@@@@@@@@ 위까지 경로 2

        time.sleep(2)  # 대기시간 2초
        robotiqgrip.gripper_action(0)  # 그리퍼 완전 닫기


        # 이후로는 건드리지 말 것
        f.close()
        # 파일 작성 종료
    # -------------- 코딩 가능 구간 끝 -------------------

    finally:
        rob.close()  # 로봇 연결 해제