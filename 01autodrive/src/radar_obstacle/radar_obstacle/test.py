"""
Author: chen fang
CreateTime: 2023/3/20 上午11:02
File: test.py
Description: 
"""
import yaml
import os
from easydict import EasyDict

if __name__ == '__main__':
    root_path = os.getcwd()
    # ======================================================== #
    # 从config文件中读取超参数
    yaml_file = root_path+'/config.yaml'
    f = open(yaml_file)
    config = yaml.load(f)
    config = EasyDict(config)
    print(config.TRACK.MIN_HITS)
