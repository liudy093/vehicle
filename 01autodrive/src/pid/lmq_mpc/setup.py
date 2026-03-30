from setuptools import setup

package_name = 'pid'

setup(
    name=package_name,
    version='0.0.0',
    # 声明所有子包，使 colcon 能够识别子模块
    packages=[
        package_name,           # 主包 'pid'
        f'{package_name}.new',  # 旧子包 'pid.new' (保留兼容)
        'lmq_mpc',              # 新包 'lmq_mpc' (推荐使用)
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tji_lab',
    maintainer_email='tji_lab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 集成控制节点 (横向PID + 纵向MPC)
            'lmq_ctrl = lmq_mpc.lmq_integrated_control:main',
            # 原始 PID 控制节点 (横向+纵向控制)
            'pid = pid.pid:main',
        ],
    },
)
