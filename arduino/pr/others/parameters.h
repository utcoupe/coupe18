//
// Created by tfuhrman on 21/04/17.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#define BAUDRATE 57600
#define SERIAL_TYPE SERIAL_8N1
#define ARDUINO_ID "pr_others"

#define NB_SERVO 4

// Set the pins corresponding to the used servo motors
#define PR_MODULE_ARM_PIN           9
#define PR_MODULE_DROP_R_PIN        6
#define PR_MODULE_DROP_L_PIN        5
#define PR_MODULE_ROTATE_PIN        3
#define PR_MODULE_ARM_ROTATE_PIN    11

//todo remove ?
#define HZ 100
#define DT (1.0/HZ)

//todo add an enum with the different debug levels available
#define DEBUG_LEVEL 2

#endif //PARAMETERS_H
