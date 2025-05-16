from setuptools import find_packages, setup

package_name = 'autonomous_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zach',
    maintainer_email='23177104@student.uwa.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = autonomous_nav.navigator:main',
            'demo = autonomous_nav.demo:main',
            'get_gps_loc = autonomous_nav.get_gps_loc:main',
            'gps_driving = autonomous_nav.gps_driving:main',
            'imu_phidget = autonomous_nav.imu_phidget:main',
            'imu_publisher = autonomous_nav.imu_publisher:main',
            'test_nav = autonomous_nav.test_nav:main',
            'heading = autonomous_nav.heading:main',
            'lidar_testing = autonomous_nav.lidar_testing:main',
            'joy_control = autonomous_nav.joy_control:main',
            'summary = autonomous_nav.summary:main',
            'lidar_obstacle = autonomous_nav.lidar_obstacle:main'
        ],
    },
)
