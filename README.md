# Kaa Mahina

#### Repository for the Robotics Space Exploration Lab at UH Manoa
There are two rovers using the software in this repository, the Active Learning Rover and Mobility Rover.
Each rover requires a LAN to connect to the ground station computer running Ubuntu 22.04 and ROS Humble.
There will be an installation script will soon be added to this repository for the ground station computer
to install the required Vicon shared object libraries and python packages.

If you have the rover and are trying to use a new laptop to use the rovers, follow the instructions for 
installation for the ground station computer and ignore the Raspberry Pi software installation steps.

## Mobility Rover

### Build Software for Mobility Rover (Ground station computer and rover Raspberry Pis)
#### Ground station computer
Clone repository -> run installation script -> run the build script
You must use the --recursive option to ensure that the ros2-vicon-receiver package is downloaded for ./install_libs.sh and to properly build everything.
```
git clone --recursive git@github.com:RiverMatsumoto/kaa-mahina.git
cd kaa-mahina
./install_libs.sh
./ccbuild.bash
```

It is normal to have some stderr output after building

#### For Rover Raspberry Pis
Clone the repository and run the build script:
```
git clone -b rover git@github.com:RiverMatsumoto/kaa-mahina.git
cd kaa-mahina
./ccbuild.bash
```

## Rover Hardware Components Used
### Mobility Rover
Motor Controllers - 2x Roboclaw 2x7A Motor Controllers
IMU - BNO055
Cameras - IMX219 Webcam 1x per Raspberry Pi
2x Raspberry Pi (1 USB to UART to communicate with motor controllers)
Battery with 2 USB A outputs

### Active Learning Rover
Motor Controllers - 2x Roboclaw 2x7A Motor controllers (Onboard computers communicate with USB to UART)
IMU - BNO055
GPS - 2x ZED_F9P RTK GPS Boards (and antennas)
Arduino Nano with Moisture Sensor (powered from connection to Raspberry Pi)
Ubuntu 22.04 Laptop with ROS Humble installed
Windows Laptop with Ublox's `u-center` GPS utilities app installed for GPS RTK correction for about 1-10cm GPS accuracy
Batteries with 1 USB A output, 1 AC output, 1 DC output
