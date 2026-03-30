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
    sonic_serial.timeout = 0
    sonic_serial.rtscts = True
    sonic_serial.dsrdtr = True
    all_sonics.append(sonic_serial)


def thread_serial():
    global main_flag
    
    cnt_err = np.zeros(sonic_num)
    while sonic_flag:
        time.sleep(0.01)

        for i in range(sonic_num):
            
            cnt_err[i] += 1
            sonic_serial = all_sonics[i]

            if cnt_err[i] > 50: # timeout, restart the serial
                try:
                    sonic_serial.close()
                    sonic_serial.open()
                    print("reopen serial %d !!!"%(i+1))
                except:
                    print("failed to start sonic aaaa %d"%(i+1))
                cnt_err[i] = 0

            # check the data in buffer
            if sonic_serial.isOpen():
                n_ = sonic_serial.in_waiting
                if n_ > 0:
                    # 读出串口数据
                    recv_ = sonic_serial.read(n_)
                    sonic_serial.reset_input_buffer()
                    cnt_err[i] = 0
                    print("------ sonic %d"%(i+1), recv_)

    
    for i in range(sonic_num):
        print("close sonic %d"%(i+1))
        all_sonics[i].close()
        print("has closed sonic %d"%(i+1))
        
    main_flag = False

def main():

    for i in range(sonic_num):
        try:
            all_sonics[i].open()
            print("sucess !!!! %d"%(i+1))
        except:
            print("first time failed to start sonic %d"%(i+1))

    
    t_ = threading.Thread(target=thread_serial, args=())
    t_.setDaemon(True)
    t_.start()
    
    while main_flag or t_.is_alive():
        time.sleep(1)
        print("ssssssssssssssssssss main_flag = %s, t = %s, sonic = %s"%(str(main_flag), str(t_.is_alive()), str(sonic_flag)))
    

if __name__ == '__main__':
    print('父进程pid: %d' % os.getpid())    
    main()
