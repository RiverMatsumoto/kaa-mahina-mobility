from setuptools import find_packages, setup

package_name = 'cr_hardware'
submodules = 'cr_hardware/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
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
        ],
    },
)
