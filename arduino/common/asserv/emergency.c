#include "emergency.h"
#include "pins.h"
#include "compat.h"
#include "parameters.h"
#include "Arduino.h"

emergency_status_t emergency_status[2] = {
	{.phase = NO_EMERGENCY, .in_use = USE_SHARP, .total_time = 0},
	{.phase = NO_EMERGENCY, .in_use = USE_SHARP, .total_time = 0},
};

void ComputeEmergencyOnPin(int pin, emergency_status_t *status);

void EmergencySetStatus(int enable) {
	emergency_status[EM_FORWARD].in_use = enable;
	emergency_status[EM_BACKWARD].in_use = enable;
	if (!enable) {
		emergency_status[EM_FORWARD].phase = NO_EMERGENCY;
		emergency_status[EM_BACKWARD].phase = NO_EMERGENCY;
	}
}

void ComputeEmergency(void) {
#ifdef PIN_SHARP_FORWARD
	ComputeEmergencyOnPin(PIN_SHARP_FORWARD, &emergency_status[EM_FORWARD]);
#endif
#ifdef PIN_SHARP_BACKWARD
	ComputeEmergencyOnPin(PIN_SHARP_BACKWARD, &emergency_status[EM_BACKWARD]);
#endif
}

void ComputeEmergencyOnPin(int pin, emergency_status_t *status) {
	float analog, distance, voltage;
	long now;

	if (!status->in_use)
		return;

	now = timeMillis();
	analog = analogRead(pin);
//	voltage = 5.0*analog/1023.0;
//	if (voltage == 0) {
//		distance = 1000;
//	} else {
//		distance = 0.123/voltage;
//	}
//	switch (status->phase) {
//		case NO_EMERGENCY:
//			if (distance < EMERGENCY_STOP_DISTANCE) {
//				if (status->start_detection_time < 0) {
//					status->start_detection_time = now;
//				} else if (now - status->start_detection_time > 300) {
//					status->start_detection_time = -1;
//					status->phase = FIRST_STOP;
//				}
//			}
//			break;
//		case FIRST_STOP:
//			status->total_time += DT;
//			if (distance > EMERGENCY_STOP_DISTANCE) {
//				status->phase = NO_EMERGENCY;
//			}
//			if ((status->total_time) > (EMERGENCY_WAIT_TIME)) {
//				status->phase = SLOW_GO;
//			}
//			break;
//		case SLOW_GO:
//			if (distance > EMERGENCY_STOP_DISTANCE) {
//				status->phase = NO_EMERGENCY;
//			}
//			break;
//	}
    // if (analog > 185) {
    //     status->phase = FIRST_STOP;
    // } else {
    //     status->phase = NO_EMERGENCY;
    // }
	status->phase = FIRST_STOP;
}
