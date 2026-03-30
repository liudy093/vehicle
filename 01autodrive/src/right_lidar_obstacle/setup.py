from setuptools import setup

package_name = 'right_lidar_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='my',
    maintainer_email='1363218630@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "right_lidar_obstacle = right_lidar_obstacle.right_lidar_obstacle:main",
        ],
    },
)
