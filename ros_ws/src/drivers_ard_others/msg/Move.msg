# Topic to move an actuator. Specify the type, actuator id and desired position.

uint8 TYPE_DIGITAL = 0 # On/Off devices
uint8 TYPE_PWM     = 1 # PWM regulated devices (DC motors...)
uint8 TYPE_SERVO   = 2 # Real servos (drived with Servo.h in arduino)
uint8 type

uint8 id

int16 dest_value