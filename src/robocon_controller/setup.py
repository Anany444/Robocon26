from setuptools import find_packages, setup

package_name = 'robocon_controller'

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
    maintainer='robot',
    maintainer_email='ananysfunique1511@gmail.com',
    description='TODO: Package robocon_description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_node = robocon_controller.gui_node:main',
            'goal_pose_controller = robocon_controller.goal_pose_controller:main',
            'controller_node = robocon_controller.controller_node:main',
        ],
    },
)
