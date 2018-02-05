#include "ax12_driver.h"
#include <unistd.h>

using namespace Ax12Table;

bool Ax12Driver::initialize(uint8_t port_index)
{
    return (bool)dxl_initialize(port_index, BAUD_RATE_INDEX);
}

void Ax12Driver::scan_motors()
{
    for(uint8_t i = 1; i <= SCAN_RANGE; i++)
    {
        for(uint8_t j = 0; j < PING_PASS_NBR; j++)
        {
            dxl_ping(i);
            usleep(PING_SLEEP);
            if (dxl_get_result() == COMM_RXSUCCESS)
            {
                motor_ids[motor_count++] = i;
                break;
            }
        }

    }
}

bool Ax12Driver::write_register(uint8_t motor_id, Register reg, int16_t value)
{
    if(reg.access == READ)
        return false;

    if(reg.size == BYTE)
        dxl_write_byte(motor_id, reg.address, value);
    else
        dxl_write_word(motor_id, reg.address, value);

    return dxl_get_result() == COMM_TXSUCCESS;
}

bool Ax12Driver::read_register(uint8_t motor_id, Register reg, int16_t &value)
{
    if(reg.size == BYTE)
        value = dxl_read_byte(motor_id, reg.address);
    else
        value = dxl_read_word(motor_id, reg.address);

    return dxl_get_result() == COMM_RXSUCCESS;
}

bool Ax12Driver::motor_id_exists(uint8_t motor_id)
{
    /*
     * Returns true if the motor was successfully detected
     */

    for(uint8_t i = 0; i < motor_count; i++)
    {
        if(motor_ids[i] == motor_id)
            return true;
    }

    return false;
}

bool Ax12Driver::motor_id_connected(uint8_t motor_id)
{
    /*
     * Returns true if the motor is still connected
     */

    for(uint8_t j = 0; j < PING_PASS_NBR; j++)
    {
        dxl_ping(motor_id);
        usleep(500);

        if (dxl_get_result() == COMM_RXSUCCESS)
            return true;
    }

    return false;
}

bool Ax12Driver::toggle_torque(bool enable)
{
    bool success = true;

    for(uint8_t i = 0; i < motor_count; i++)
        success &= write_register(motor_ids[i], TORQUE_ENABLE, enable);

    return success;
}

bool Ax12Driver::joint_mode(uint8_t motor_id, uint16_t min_angle, uint16_t max_angle)
{
    bool success = true;

    success &= write_register(motor_id, TORQUE_ENABLE, 0);
    success &= write_register(motor_id, CW_ANGLE_LIMIT, min_angle);
    success &= write_register(motor_id, CCW_ANGLE_LIMIT, max_angle);
    success &= write_register(motor_id, TORQUE_ENABLE, 1);

    return success;
}

bool Ax12Driver::wheel_mode(uint8_t motor_id)
{
    return joint_mode(motor_id, 0, 0); // wheel mode is set with min = max = 0
}

uint8_t Ax12Driver::get_motor_count()
{
    return motor_count;
}