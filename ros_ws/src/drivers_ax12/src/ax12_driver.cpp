#include "ax12_driver.h"
#include <unistd.h>

using namespace Ax12Table;

bool Ax12Driver::initialize(const std::string &port_name)
{
    port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packet_handler = dynamixel::PacketHandler::getPacketHandler(1.0);

    return port_handler->openPort() &&
            port_handler->setBaudRate(1000000 / BAUD_RATE_INDEX);
}

void Ax12Driver::scan_motors()
{
    uint8_t err = 0;
    int result = 0;
    for(uint8_t i = 1; i <= SCAN_RANGE; i++)
    {
        for(uint8_t j = 0; j < PING_PASS_NBR; j++)
        {
            result = packet_handler->ping(port_handler, i, &err);

            if (result == COMM_SUCCESS)
            {
                motor_ids[motor_count++] = i;
                break;
            }
            usleep(PING_SLEEP);
        }
    }
}

bool Ax12Driver::write_register(uint8_t motor_id, Register reg, uint16_t value)
{
    if(reg.access == READ)
        return false;

    int result;

    if(reg.size == BYTE)
        result = packet_handler->write1ByteTxRx(port_handler, motor_id, reg.address,
                                                static_cast<uint8_t>(value));
    else
        result = packet_handler->write2ByteTxRx(port_handler, motor_id, reg.address, value);

    return result == COMM_SUCCESS;
}

bool Ax12Driver::read_register(uint8_t motor_id, Register reg, uint16_t &value)
{
    int result;

    if(reg.size == BYTE) {
        uint8_t ret_val;
        result = packet_handler->read1ByteTxRx(port_handler, motor_id, reg.address, &ret_val);
        value = ret_val;
    } else {
        uint16_t ret_val;
        result = packet_handler->read2ByteTxRx(port_handler, motor_id, reg.address, &ret_val);
        value = ret_val;
    }

    return result == COMM_SUCCESS;
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
    int result;

    for(uint8_t j = 0; j < PING_PASS_NBR; j++)
    {
        result = packet_handler->ping(port_handler, motor_id);
        if (result == COMM_SUCCESS)
            return true;

        usleep(PING_SLEEP);
    }

    return false;
}

bool Ax12Driver::toggle_torque(bool enable)
{
    bool success = true;

    for(uint8_t i = 0; i < motor_count; i++) {
        success &= write_register(motor_ids[i], TORQUE_ENABLE, static_cast<uint16_t>(enable));
    }

    return success;
}

bool Ax12Driver::joint_mode(uint8_t motor_id, uint16_t min_angle, uint16_t max_angle)
{
    bool success = true;

//    success &= write_register(motor_id, TORQUE_ENABLE, 0);
    success &= write_register(motor_id, CW_ANGLE_LIMIT, min_angle);
    success &= write_register(motor_id, CCW_ANGLE_LIMIT, max_angle);
//    success &= write_register(motor_id, TORQUE_ENABLE, 1);

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

uint8_t* Ax12Driver::get_motor_ids()
{
    return motor_ids;
}