import os
from glob import glob
from setuptools import setup

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*.*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aayush Mishra',
    maintainer_email='root@todo.todo',
    description='the pick_and_place package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = pick_and_place.object_detector:main',
            'state_machine = pick_and_place.pick_and_place_state_machine:main',
        ],
    },
)
