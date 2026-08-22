from glob import glob
import os
import distutils.core
from setuptools import find_packages, setup

package_name = 'r2_joy_serial'

setup_args = dict(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_runner = r2_joy_serial.joy_run:main',
            'serial_printer = r2_joy_serial.serial_print:main',
            'serial_read = r2_joy_serial.serial_read:main',
        ],
    },
)

setup(**setup_args)

try:
    if getattr(distutils.core.setup, '__name__', '') != 'setup':
        distutils.core.setup(**setup_args)
except Exception:
    pass
