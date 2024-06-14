#include <chrono>
#include <iostream>
#include <vector>
#include <libudev.h> // for finding which port the roboclaw usb is on

#include "libroboclaw/roboclaw_driver.hpp"
#include "libroboclawConfig.h"

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        // report version
        std::cout << "Not command line arguments provided. Using defaults values:" << std::endl;
        std::cout << "baudrate=115200, " << "address=128" << std::endl;
        std::cout << argv[0] << " Version " << libroboclaw_VERSION_MAJOR << "."
                  << libroboclaw_VERSION_MINOR << std::endl;
        std::cout << "Usage: " << argv[0] << " serial_port baudrate address" << std::endl;
        std::cout << "Example: " << argv[0] << " 115200 128" << std::endl;
        return 1;
    }
    std::vector<std::string> roboclaw_devices = libroboclaw::util::find_roboclaw_devices();
    // just use the first device found
    std::string serial_port = roboclaw_devices[0];
    unsigned int baudrate = argc > 1 ? 115200u : std::stoi(argv[2]);
    unsigned int addr = argc > 2 ? 128u : std::stoi(argv[3]);
    std::cout << "attempting to establish connection to port " << serial_port << " with baud rate "
              << baudrate << " at address " << addr << "..." << std::endl;
    libroboclaw::driver *roboclaw_connections = new libroboclaw::driver(serial_port, baudrate);

    std::pair<int, int> encoders = roboclaw_connections->get_encoders(addr);
    float voltage = roboclaw_connections->get_main_battery_voltage(addr);
    std::cout << "Enc1: " << encoders.first << ", Enc2: " << encoders.second << std::endl;
    std::cout << "Voltage: " << voltage << std::endl;

    // get version (warning: brittle!)
    std::cout << "version: " << roboclaw_connections->get_version(addr) << std::endl;

    delete roboclaw_connections;
    return 0;
}
