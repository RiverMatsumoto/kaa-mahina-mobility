from setuptools import find_packages, setup

package_name = 'cr_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='river',
    maintainer_email='matsumotorjames@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_driver = cr_hardware.arduino_driver:main',
            'joy_control_node = cr_hardware.joy_to_twist_node:main',
            'moisture_sensor = cr_hardware.moisture_sensor:main'
            # 'camera_node = cr_hardware.camera:main',
        ],
    },
)
