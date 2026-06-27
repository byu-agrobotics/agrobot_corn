from setuptools import find_packages, setup

package_name = 'agrobot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='w2boy',
    maintainer_email='wesleyt.wardle@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'input = agrobot_nav.publisher_member_function:main',
            'drive_control = agrobot_nav.subscriber_member_function:main',
            'tof_sensor = agrobot_nav.tof_sensor_node:main',
            'nav = agrobot_nav.nav_state_machine:main',
            'camera = agrobot_nav.camera_node:main',
        ],
    },
)
