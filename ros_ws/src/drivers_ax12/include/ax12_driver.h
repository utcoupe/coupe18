
#ifndef DRIVERS_AX12_AX12_DRIVER_H
#define DRIVERS_AX12_AX12_DRIVER_H

#include "dynamixel.h"
#include "ax12_table.h"

class Ax12Driver
{
public:

    const static uint8_t SCAN_RANGE = 25; // the scan pings motors with id 1 to SCAN_RANGE
    const static uint8_t PING_PASS_NBR = 20; // number of times a motor is pinged to make sure it is connected
    const static uint8_t BAUD_RATE_INDEX = 1; // baudrate = 2000000 / (index + 1)
    const static int PING_SLEEP = 800; // microsec to sleep between pings

protected:
    uint8_t motor_count;
    uint8_t motor_ids[SCAN_RANGE];

public:
    bool initialize(uint8_t port_index);
    void scan_motors();
    bool write_register(uint8_t motor_id, const Ax12Table::Register reg, int16_t value);
    bool read_register(uint8_t motor_id, const Ax12Table::Register reg, int16_t &value);
    bool motor_id_exists(uint8_t motor_id);
    bool motor_id_connected(uint8_t motor_id);
    bool toggle_torque(bool enable);
    bool joint_mode(uint8_t motor_id, uint16_t min_angle=1, uint16_t max_angle=1023);
    bool wheel_mode(uint8_t motor_id);
    uint8_t get_motor_count();
    uint8_t* get_motor_ids();


    Ax12Driver() : motor_count(0) {}

};

#endif //DRIVERS_AX12_AX12_DRIVER_H
