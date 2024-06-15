#include <iostream>
#include "cr_hardware/roboclaw.h"

int main(int argc, char const *argv[])
{
    int address = 128;
    struct roboclaw *rc;
    rc = roboclaw_init("/dev/ttyUSB0", 115200);
    if (!rc)
    {
        std::cerr << "Serial device not found. Exiting" << std::endl;
        return 1;
    }
    std::cout << "Opened roboclaw connection." << std::endl;

    float voltage;
    int32_t enc_m1, enc_m2;
    roboclaw_main_battery_voltage(rc, address, &voltage);
    roboclaw_encoders(rc, 128, &enc_m1, &enc_m2);
    std::cout << "Main battery voltage: " << voltage << std::endl;
    std::cout << "Encoder M1: " << enc_m1 << std::endl;
    std::cout << "Encoder M2: " << enc_m2 << std::endl;

    if (roboclaw_close(rc) == ROBOCLAW_OK)
        std::cout << "Closed roboclaw connection" << std::endl;
    else
        std::cerr << "Failed to close roboclaw connection" << std::endl;

    return 0;
}
