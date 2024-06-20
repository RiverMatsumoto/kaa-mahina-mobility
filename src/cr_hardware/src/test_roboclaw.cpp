#include <iostream>
#include <string>
#include "cr_hardware/roboclaw.h"

int main(int argc, char const *argv[])
{
    int address = 128;
    int baudrate = 115200;
    const char *device = "/dev/ttyACM0";
    if (argc < 3)
    {
        printf("Usage: %s <device> <address> <baudrate>\n", argv[0]);
        printf("Example: %s /dev/ttyACM0 128 115200\n", argv[0]);
        printf("Using default values: /dev/ttyACM0 128 115200\n");
    }
    else
    {
        device = argv[1];
        address = std::stoi(argv[2]);
        baudrate = std::stoi(argv[3]);
    }
    struct roboclaw *rc;
    rc = roboclaw_init(device, baudrate);
    if (!rc)
    {
        std::cerr << "Serial device \"" << device << "\" not found. Exiting" << std::endl;
        return 1;
    }
    std::cout << "Opened roboclaw connection." << std::endl;

    float voltage;
    int32_t enc_m1, enc_m2;
    roboclaw_main_battery_voltage(rc, address, &voltage);
    roboclaw_encoders(rc, address, &enc_m1, &enc_m2);
    std::cout << "Main battery voltage: " << voltage << std::endl;
    std::cout << "Encoder M1: " << enc_m1 << std::endl;
    std::cout << "Encoder M2: " << enc_m2 << std::endl;

    if (roboclaw_close(rc) == ROBOCLAW_OK)
        std::cout << "Closed roboclaw connection" << std::endl;
    else
        std::cerr << "Failed to close roboclaw connection" << std::endl;

    return 0;
}
