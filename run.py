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
    while(True):
        try:
            gettask_url = 'http://' + ip_addr + '/task_api/get_task'
            gettask_reqres = requests.get(gettask_url)
            if gettask_reqres.status_code == 200:
                task = json.loads(gettask_reqres.text)
                if task['status'] == True:
                    return task['task_id'], task['dest']
        except Exception as e:
            pass


def Finish_Task():
    '''
    接受参数为任务ID
    返回值自定
    '''
    while(True):
        try:
            taskid_json = json.dumps({'task_id': taskid})
            finishtask_url = 'http://' + ip_addr + '/task_api/finish_task'
            finishtask_reqres = requests.post(finishtask_url, taskid_json)
            if finishtask_reqres.status_code == 200:
                break
        except Exception as e:
            pass


def Camera():
    cap = cv2.VideoCapture(0)
    distance_data = 15.0
    motor_speed = 0
    address = (ip_addr, 2020)
    tcpClientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while(True):
        try:
            tcpClientSocket.connect(address)
        except Exception as e:
            continue
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
                try:
                    _, jpg_data = cv2.imencode('.jpg', frame)
                    pkg_num = math.ceil(jpg_data.shape[0]/pkg_size)
                    temp = np.array(
                        [pkg_num, jpg_data.shape[0]-pkg_size*(pkg_num-1)], dtype=np.uint16).tostring()
                    data = begin_msg+temp
                    tcpClientSocket.send(data)
                    for i in range(pkg_num-1):
                        data = middle_msg + \
                            jpg_data[pkg_size*i:pkg_size*(i+1)].tostring()
                        tcpClientSocket.send(data)
                    data = end_msg + \
                        jpg_data[pkg_size*(pkg_num-1)
                                           :jpg_data.shape[0]].tostring()
                    tcpClientSocket.send(data)
                    detection_res, _ = tcpClientSocket.recvfrom(512)
                    try:
                        detection_res = json.loads(
                            str(detection_res, encoding='utf-8'))
                        for detection in detection_res['detections']:
                            ids, x1, y1, x2, y2 = detection[0], detection[1], detection[2], detection[3], detection[4]
                            if abs((x1+x2)//2 - 320) < 20:
                                if queue.empty():
                                    queue.put(1)
                    except Exception as e:
                        pass
                    end_time = time.perf_counter()
                except Exception as e:
                    tcpClientSocket.close()
                    break
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
    flag = 0
    time_stop = 0.5
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
                datas = mag.Get_Data()
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
                    start = time.time()
                print('Magnet angle:%f' % datas)
                if datas - current_target > 3:  # 偏右
                    speed_left = 0
                    speed_right = 768
                elif current_target - datas > 3:  # 偏左
                    speed_right = 0
                    speed_left = 768
                else:
                    speed_left = speed_right = 768
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
    startpoint = Get_Init()
    p = mt.Pool(3)
    p.apply_async(func=Magnet_Data)
    p.apply_async(func=Distance_Data)
    p.apply_async(func=Camera)
    flag = 0
    while(True):
        try:
            taskid, endpoint = Get_Task()
            taskpath = Map.Get_Best(startpoint, endpoint)[1]
            startpoint = endpoint
            for i in range(len(taskpath) - 1):
                st = taskpath[i]
                ed = taskpath[i+1]
                path_mag = Map.mag_matrix[st, ed]
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
