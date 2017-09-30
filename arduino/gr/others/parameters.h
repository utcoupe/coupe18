//
// Created by tfuhrman on 21/04/17.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#define BAUDRATE 57600
#define SERIAL_TYPE SERIAL_8N1
#define ARDUINO_ID "gr_others"

#define NB_SERVO 4

// Set the pins corresponding to the used servo motors
#define GR_SWEEPER_PIN  2
#define GR_CANON_PIN    3
#define GR_ROCKET_PIN   5
#define GR_LOADER_PIN   4

//todo remove ?
#define HZ 100
#define DT (1.0/HZ)

//todo add an enum with the different debug levels available
#define DEBUG_LEVEL 2

#endif //PARAMETERS_H
