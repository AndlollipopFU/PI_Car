import wiringpi as wp
import multiprocessing as mt
from motor import Motor
from distance_cal import Distance_Module
from gy271m import GY271M
from maps import Maps
import time
import numpy as np
import os
import cv2
import pyzbar.pyzbar as pyzbar
from spin import Spin
import math

# define multiprocess information queue
# queue for distance to motor
# queue1 for main to motor
# queue2 for camera to main
# queue3 for distance to camera
# queue4 for motor to camera
queue = mt.Queue(1)
queue1 = mt.Queue(1)
queue2 = mt.Queue(1)
queue3 = mt.Queue(1)
queue4 = mt.Queue(1)

# define map
dist_matrix = [[0, 6, -1, -1, 5, -1],
               [6, 0, 8, -1, 2, -1],
               [-1, 8, 0, 4, -1, 3],
               [-1, -1, 4, 0, 2, 4],
               [5, 2, -1, 2, 0, -1],
               [-1, -1, 3, 4, -1, 0]]
mag_matrix = [[0, 140, -1, -1, 188, -1],
              [344, 0, 206, -1, 288, -1],
              [-1, 356, 0, 245, -1, 320],
              [-1, -1, 75, 0, 344, 11],
              [45, 103, -1, 176, 0, -1],
              [-1, -1, 162, 197, -1, 0]]
dist_matrix = np.array(dist_matrix)
Map = Maps(dist_matrix, mag_matrix)

# define motor
pwm0 = 1
pwm1 = 23
in1 = 29
in2 = 28
in3 = 27
in4 = 26
m1 = Motor(pwm0, pwm1, in1, in2, in3, in4)

# Define module pin
dist_head_trig = 2
dist_head_echo = 3
# dist_right_trig = 12
# dist_right_echo = 13
# dist_left_trig = 24
# dist_left_echo = 25
dist_upright_trig = 14
dist_upright_echo = 30
dist_upleft_trig = 21
dist_upleft_echo = 22
dist_head = Distance_Module(dist_head_trig, dist_head_echo)
dist_upright = Distance_Module(dist_upright_trig, dist_upright_echo)
dist_upleft = Distance_Module(dist_upleft_trig, dist_upleft_echo)

# define magnet
device = 0x1e
mag = GY271M(device)

# define spin
spinleft_out = 25
spinright_out = 24
spinleft = Spin(spinleft_out)
spinright = Spin(spinright_out)


def Get_Task():
    '''
    没有任务时应阻塞不返回
    返回值为任务ID和到达地点（int）
    不接受参数
    '''
    # TODO Acquiring Tasks
    pass


def Finish_Task():
    '''
    接受参数为任务ID
    返回值自定
    '''
    # TODO Tasks Response
    pass


def Camera():
    cap = cv2.VideoCapture(0)
    distance_data = 15.0
    motor_speed = 0
    while(True):
        ret, frame = cap.read()
        if ret:
            frame = cv2.flip(frame, 0)
            if queue2.empty():
                queue2.put(frame)
            if not queue3.empty():
                distance_data = queue3.get()
            if not queue4.empty():
                motor_speed = queue4.get()
            # TODO Vedio Transmission
            # return


def Distance_Data():
    print('Distance PID:%i' % os.getpid())
    while(True):
        f = 0
        datas = [0, 0, 0]
        distance_head = dist_head.dist()
        distance_upright = dist_upright.dist()
        distance_upleft = dist_upleft.dist()
        datas[0] = distance_head
        datas[1] = distance_upright
        datas[2] = distance_upleft
        # print(datas)
        for index, item in enumerate(datas):
            if item <= 0.2:
                f = 1
                break
        if queue.empty():
            queue.put(f)
        if queue3.empty():
            queue3.put(datas)
        # time.sleep(0.5)
    return


def Magnet_Data():
    print('Magnet PID:%i' % os.getpid())
    current_target = -1
    speed_left = 1024
    speed_right = 512
    flag = 0
    time_stop = 1
    while(True):
        if not queue.empty():
            flag = queue.get()
        if flag == 1:
            # print('Attack!')
            m1.Break()
            left_stat = spinleft.read_Data()
            right_stat = spinright.read_Data()
            count_left = 0
            count_right = 0
            start = time.time()
            if queue4.empty():
                queue4.put(0)
        elif flag == 0:
            if current_target != -1:
                if not queue1.empty():
                    m1.Break()
                    current_target = -1
                    continue
                count_left += spinleft.read_Data() ^ left_stat
                count_right += spinright.read_Data() ^ right_stat
                left_stat = spinleft.read_Data()
                right_stat = spinright.read_Data()
                if time.time() - start > time_stop:
                    count = (count_left + count_right)//2
                    sped = (count / 20)*math.pi*6.5*2
                    count_left = count_right = 0
                    if queue4.empty():
                        queue4.put(sped)
                    speed_right = 0
                    if time.time() - start > time_stop+0.15:
                        speed_right = 512
                        start = time.time()
                    # print('Left Count:%i Right Count:%i' %
                    #       (count_left, count_right))
                    # if abs(count_left - count_right) > 3:
                    #     if count_left > count_right:
                    #         speed_left = speed_left + 20 if speed_left <= 1004 else speed_left
                    #     elif count_left < count_right:
                    #         speed_left = speed_left - 20 if speed_left >= 532 else speed_left
                    # print('Left Speed:%i Right Speed%i' %
                    #       (speed_left, speed_right))
                    # count_left = count_right = 0

                #print('Magnet angle:%f' % datas)
                # if datas - current_target > 3:  # 偏右
                #     speed_left = 0
                #     speed_right = 768
                # elif current_target - datas > 3:  # 偏左
                #     speed_right = 0
                #     speed_left = 768
                # else:
                #     speed_left = speed_right = 768
                m1.Accelerate(speed_left, speed_right)
            elif current_target == -1:
                #m1.Accelerate(speed_left, speed_right)
                m1.Break()
                if queue4.empty():
                    queue4.put(0)
                while(queue1.empty()):
                    pass
                current_target = queue1.get()
                if current_target == -1:
                    continue
                datas = mag.Get_Data()
                # print(datas)
                while(abs(datas - current_target) > 2):
                    if queue4.empty():
                        queue4.put(0)
                    datas = mag.Get_Data()
                    # print(datas)
                    m1.Turn(512, 512, 0)
                m1.Break()
                left_stat = spinleft.read_Data()
                right_stat = spinright.read_Data()
                count_left = 0
                count_right = 0
                start = time.time()


def Get_Init():
    speed = 512
    cap = cv2.VideoCapture(0)
    start = time.time()
    while(True):
        m1.Break()
        ret, frame = cap.read()
        if ret:
            frame = cv2.flip(frame, 0)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            barcodes = pyzbar.decode(gray)
            data = -1
            if barcodes != None:
                for barcode in barcodes:
                    (x, y, w, h) = barcode.rect
                    print(w, h)
                    data = barcode.data.decode('utf-8')
                    if (w < 50) or (h < 50):
                        pass
            if data != -1:
                m1.Drift()
                cap.release()
                return int(data) - 1
            if (time.time() - start < 1.5):
                continue
            m1.Turn(speed, speed, 0)
            time.sleep(0.3)
            start = time.time()


if __name__ == "__main__":
    print('Main PID:%i' % os.getpid())
    # try:
    #     while(1):
    #         startpoint = Get_Init()
    #         print(startpoint)
    # except KeyboardInterrupt:
    #     m1.Accelerate(1024, 512)
    #     time.sleep(0.8)
    #     m1.Break()
    startpoint = Get_Init()
    p = mt.Pool(3)
    p.apply_async(func=Magnet_Data)
    p.apply_async(func=Distance_Data)
    p.apply_async(func=Camera)
    # videowriter = cv2.VideoWriter('test2.avi', cv2.VideoWriter_fourcc(
    #     'I', '4', '2', '0'), 30, (640, 480))
    flag = 0
    while(True):
        try:
            #taskid, endpoint = Get_Task()
            endpoint = 2
            taskpath = Map.Get_Best(startpoint, endpoint)[1]
            print(taskpath)
            startpoint = endpoint
            for i in range(len(taskpath) - 1):
                st = taskpath[i]
                ed = taskpath[i+1]
                path_mag = Map.mag_matrix[st, ed]
                print(path_mag)
                while(queue1.full()):
                    m1.Break()
                    pass
                queue1.put(path_mag)
                while(queue1.full()):
                    pass
                while(True):
                    if not queue2.empty():
                        frame = queue2.get()
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        barcodes = pyzbar.decode(gray)
                        data = -1
                        if barcodes != None:
                            for barcode in barcodes:
                                (x, y, w, h) = barcode.rect
                                data = barcode.data.decode('utf-8')
                                if (w < 100) or (h < 100):
                                    pass
                        if data != -1:
                            data = int(data) - 1
                            if data == ed:
                                time.sleep(0.8)
                                queue1.put(-1)
                                break
            Finish_Task()
        except KeyboardInterrupt:
            m1.Break()
            p.close()
            p.terminate()
            break
