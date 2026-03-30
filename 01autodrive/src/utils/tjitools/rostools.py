import os
import time

log_dir = os.getcwd() + '/datas/logs/'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

def ros_log(name, info):
    log_file = log_dir + name + '.log'
    now_ts = time.strftime('%Y-%m-%d %H:%M:%S')
    now_ms = str(round(time.time() * 1000) % 1000).zfill(3)    
    with open(log_file, mode='a', encoding='utf-8') as f_:
        f_.write('[%s:%s]%s\n'%(now_ts, now_ms, info))
