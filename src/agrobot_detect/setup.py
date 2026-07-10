from setuptools import find_packages, setup

package_name = 'agrobot_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/hsv.db'])
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lcd_display_publisher = agrobot_detect.lcd_display_publisher:main',
            'lcd_display_subscriber = agrobot_detect.lcd_display_subscriber:main',
            'vision_application_publisher = agrobot_detect.vision_application_publisher:main',
	    'beam_break_node = agrobot_detect.beam_break_node:main',
        ],
    },
)
