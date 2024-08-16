/*
 * Modified by River Matsumoto <matsumotorjames@gmail.com>
 * roboclaw-test example for roboclaw library
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

/*
 *  This example:
 *  -initializes communication with roboclaw unit
 *  -reads & prints battery voltage
 *  -in a loop
 *    - reads duty cycle from user [-100, 100]%
 *    - sets this duty cycle to motors
 *    - breaks the loop on non numeric input
 *  -stops the motors
 *  -cleans after library
 *
 *  Program expects terminal device, baudrate and roboclaw address (from 0x80 to
 * 0x87, ignored in USB mode) e.g.
 *
 *  roboclaw-test /dev/ttyACM0 38400 0x80
 *
 */
#define _POSIX_C_SOURCE 199309L // Ensure POSIX.1-2001 compliance

#include "roboclaw.h"

#include <stdio.h>  //printf
#include <stdlib.h> //exit
#include <time.h>
#include <unistd.h> //sleep

void usage(char **argv);
void informative_terminate(struct roboclaw *rc);

int main(int argc, char **argv)
{

    struct roboclaw *rc;
    uint8_t address = 128; // address of roboclaw unit
    int baudrate = 115200;
    int16_t voltage;
    float voltage_float;
    int duty_cycle;
    char *device = "/dev/ttyUSB0";

    // device
    if (argc == 1)
    {
        usage(argv);
        printf("Using default device: /dev/ttyACM0\n");
        printf("Using default baud rate: %d\n", baudrate);
        printf("Using default address: %d\n", address);
    }
    // don't change default device
    if (argc == 2)
    {
        printf("Using default baud rate: %d\n", baudrate);
        printf("Using default address: %d\n", address);
    }
    // baud rate
    if (argc == 3)
    {
        baudrate = (int)strtol(argv[2], NULL, 10);
        printf("Using default address: %d\n", address);
    }
    // address
    if (argc == 4)
    {
        baudrate = (int)strtol(argv[2], NULL, 10);
        address = (uint8_t)strtol(argv[3], NULL, 0);
    }
    if (argc > 4)
    {
        usage(argv);
        exit(0);
    }

    // initialize at supplied terminal at specified baudrate
    rc = roboclaw_init(argv[1], baudrate);

    if (rc == NULL)
    {
        perror("unable to initialize roboclaw");
        exit(1);
    }

    printf("initialized communication with roboclaw\n");

    // read the battery voltage
    if (roboclaw_main_battery_voltage(rc, address, &voltage) != ROBOCLAW_OK)
        informative_terminate(rc);

    voltage_float = (float)voltage / 10.0f;
    printf("battery voltage is : %f V\n", voltage_float);

    printf("Getting polling rate...\n");
    // Get polling rate
    int count = 0;
    time_t start, end;
    float seconds = 3.0f;
    start = time(NULL);
    do
    {
        int res = roboclaw_main_battery_voltage(rc, address, &voltage);
        if (res != ROBOCLAW_OK)
            printf("Failed main battery read at: %d", count);
        count++;
        end = time(NULL);
    } while (difftime(end, start) < seconds);
    double frequency = count / seconds;
    printf("Total function calls: %d\n", count);
    printf("Function calls per second (Hz): %f\n", frequency);
    // function calls per second * 6 bytes per function call * 8 bits per byte
    printf("Approximately %f bits per second\n", frequency * 8 * 6);

    // make sure the motors are stopped before leaving
    printf("stopping the motors..\n");
    roboclaw_duty_m1m2(rc, address, 0, 0);

    if (roboclaw_close(rc) != ROBOCLAW_OK)
        perror("unable to shutdown roboclaw cleanly");

    printf("bye...\n");

    return 0;
}

void usage(char **argv)
{
    printf("Usage:\n");
    printf("%s terminal_device baudrate address\n\n", argv[0]);
    printf("examples:\n");
    printf("%s /dev/ttyACM0 38400 0x80\n", argv[0]);
    printf("%s /dev/ttyAMA0 460800 0x81\n", argv[0]);
    printf("%s /dev/tty_in1 115200 0x82\n", argv[0]);
}

void informative_terminate(struct roboclaw *rc)
{
    fprintf(stderr, "problem communicating with roboclaw\n");
    fprintf(stderr, "make sure you are:\n");
    fprintf(stderr, "-using correct tty device\n");
    fprintf(stderr, "-using correct address\n");
    fprintf(stderr, "-set correct roboclaw baudrate\n");
    fprintf(stderr, "-wired things correctly\n");
    fprintf(stderr, "terminating...\n");
    roboclaw_close(rc);
    exit(1);
}
