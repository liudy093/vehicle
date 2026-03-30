import serial # 导入串口包
import time # 导入时间包
import threading
import os
import signal
import numpy as np
from multiprocessing import Process 

main_flag = True
sonic_flag = True

all_sonics = []
for i in range(2):

    sonic_serial = serial.Serial()
    sonic_serial.port = "/dev/sonic"+str(i+1)
    sonic_serial.baudrate = 9600
    sonic_serial.timeout = 0
    sonic_serial.rtscts = True
    sonic_serial.dsrdtr = True

    all_sonics.append(sonic_serial)

def thread_serial():
    global main_flag
    
    cnt_err = np.zeros()
    while sonic_flag:
        time.sleep(0.01)
        cnt_err += 1

        if cnt_err > 50: # timeout, restart the serial
            try:
                sonic_serial.close()
                time.sleep(0.1)
                sonic_serial.open()
                print("reopen serial %d !!!"%(sonic_id))
            except:
                print("failed to start sonic aaaa %d"%(sonic_id))
            cnt_err = 0

        # check the data in buffer
        if sonic_serial.isOpen():
            n_ = sonic_serial.in_waiting
            if n_ > 0:
                # 读出串口数据
                recv_ = sonic_serial.read(n_)
                sonic_serial.reset_input_buffer()
                cnt_err = 0
                print("------ sonic %d"%(sonic_id), recv_)
            else:
                # print("data is empty")
                pass
        else:
            print("sonic %d is not open"%(sonic_id))

    
    sonic_serial.close()        
    main_flag = False

def main():
    # while True:
    #     count = ser.inWaiting() # 获取串口缓冲区数据
    #     if count !=0 :
    #         recv = ser.read(ser.in_waiting) # 读出串口数据，数据采用gbk编码
    #         # ser.write(b'get')
    #         print(time.time()," --- recv --> ", recv) # 打印一下子
    #     time.sleep(0.1) # 延时0.1秒，免得CPU出问题

    # signal.signal(signal.SIGINT, my_signal)


    sonic_serial.open()   
    t_ = threading.Thread(target=thread_serial, args=())
    t_.setDaemon(True)
    t_.start()

    while main_flag or t_.is_alive():
        time.sleep(1)
        print("ssssssssssssssssssss main_flag = %s, t = %s, sonic = %s"%(str(main_flag), str(t_.is_alive()), str(sonic_flag)))
    
if __name__ == '__main__':
    print('父进程pid: %d' % os.getpid())    
    main()
