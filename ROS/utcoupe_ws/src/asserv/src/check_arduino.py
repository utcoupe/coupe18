#!/usr/bin/env python

"""
Small script to detect the port of the connected Arduino.
"""

import serial
import glob

__date__ = 05/12/2016
__author__ = "Thomas Fuhrmann"


# arduino is "asserv" or "others"
def get_arduino_port(arduino):
    # List of all serial ports
    ports_list = glob.glob('/dev/tty[A-Z]*')
    # List of available serial ports
    available_port_list = []
    # Dictionary of (arduino_id, arduino_port) Ex : (grobot_asserv, /dev/ttyACM0)
    arduino_port_dict = {}
    # Return value
    return_value = ""

    # Create the list of available serial ports
    for port in ports_list:
        try:
            com_line = serial.Serial(port, 57600)
            com_line.close()
            available_port_list.append(port)
        except serial.SerialException:
            pass

    # Create the dictionary of connected arduino
    arduino_id = ""
    for port in available_port_list:
        # The timeout is necessary and must be greater than the sending time of the Arduino (currently it's set to 100ms)
        # With tests, it seems like it has to be greater than 0.6s, no mater the frequency of the sending Arduino ID...
        com_line = serial.Serial(port, 57600, timeout=0.6)
        try:
            # First line may be empty
            com_line.readline()
            arduino_id = com_line.readline()
        except :
            pass
        com_line.close()
        if arduino_id != "":
            arduino_id = arduino_id.replace("\r\n", "")
            arduino_port_dict[arduino_id] = port

    # Check if the asked arduino is in the dictionary
    for connected_arduino in arduino_port_dict:
        if connected_arduino.find(arduino):
            return_value = arduino_port_dict[connected_arduino]
            print "Arduino " + arduino + " found on port : " + arduino_port_dict[connected_arduino]

    return return_value


if __name__ == "__main__":
    # TODO ask arduino as a parameter
    print get_arduino_port("asserv")
