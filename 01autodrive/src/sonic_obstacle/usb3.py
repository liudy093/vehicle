import serial # 导入串口包
import time # 导入时间包
import threading
import os
import signal
import numpy as np
from multiprocessing import Process 

main_flag = True
sonic_flag = True
sonic_num = 4

all_sonics = []
for i in range(sonic_num):
    sonic_serial = serial.Serial()
    sonic_serial.port = "/dev/sonic" + str(i+1)
    sonic_serial.baudrate = 9600
    sonic_serial.timeout = 1
    sonic_serial.rtscts = True
    sonic_serial.dsrdtr = True
    all_sonics.append(sonic_serial)

all_sonic_datas = np.zeros((4,sonic_num))

def my_signal(sig, frame):
    global sonic_flag
    sonic_flag = False
    print("close sonic !!!")

def thread_serial():
    global main_flag
    
    cnt_err = np.zeros(sonic_num)
    while sonic_flag:
        time.sleep(0.01)

        for i in range(sonic_num):
            
            cnt_err[i] += 1
            sonic_serial = all_sonics[i]

            # if cnt_err[i] > 50: # timeout, restart the serial
            #     try:
            #         sonic_serial.close()
            #         sonic_serial.open()
            #         print("reopen serial %d !!!"%(i+1))
            #     except:
            #         print("failed to start sonic aaaa %d"%(i+1))
            #     cnt_err[i] = 0

            # check the data in buffer
            if sonic_serial.isOpen():
                n_ = sonic_serial.in_waiting
                if n_ > 0:
                    # 读出串口数据
                    recv_ = sonic_serial.read_all()
                    sonic_serial.flushInput()
                    cnt_err[i] = 0
                    print("------ sonic %d"%(i+1), recv_)

    
    for i in range(sonic_num):
        print("close sonic %d"%(i+1))
        all_sonics[i].close()
        print("has closed sonic %d"%(i+1))
        

    main_flag = False

def main():
    # while True:
    #     count = ser.inWaiting() # 获取串口缓冲区数据
    #     if count !=0 :
    #         recv = ser.read(ser.in_waiting) # 读出串口数据，数据采用gbk编码
    #         # ser.write(b'get')
    #         print(time.time()," --- recv --> ", recv) # 打印一下子
    #     time.sleep(0.1) # 延时0.1秒，免得CPU出问题

    signal.signal(signal.SIGINT, my_signal)

    for i in range(sonic_num):
        try:
            all_sonics[i].open()
            print("sucess !!!! %d"%(i+1))
        except:
            print("first time failed to start sonic %d"%(i+1))

    
    # t_ = threading.Thread(target=thread_serial, args=())
    # t_.setDaemon(True)
    # t_.start()

    p_ = Process(target=thread_serial,args=())
    p_.start()
    
    while main_flag or p_.is_alive():
        time.sleep(1)
        print("ssssssssssssssssssss main_flag = %s, t = %s, sonic = %s"%(str(main_flag), str(p_.is_alive()), str(sonic_flag)))
    
    p_.join()

if __name__ == '__main__':
    print('父进程pid: %d' % os.getpid())    
    main()
