from setuptools import setup
import os
from glob import glob

package_name = 'position'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 对应源码目录名
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装脚本到lib目录（确保可执行）
        (os.path.join('lib', package_name), glob('position/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Position data collection node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式：节点名=包名.文件名:main函数
            'position = position.position:main',
        ],
    },
)