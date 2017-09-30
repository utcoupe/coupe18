#ifndef EMERGENCY_H
#define EMERGENCY_H

#define EM_FORWARD 0
#define EM_BACKWARD 1

typedef enum emergency_phase {
	NO_EMERGENCY,
	FIRST_STOP,
	SLOW_GO
} emergency_phase_t;

typedef struct emergency_status {
	int in_use;
	long start_detection_time;
	double total_time;
	emergency_phase_t phase;
} emergency_status_t;

extern emergency_status_t emergency_status[2];

#ifdef __cplusplus
extern "C" void EmergencySetStatus(int enable);
extern "C" void ComputeEmergency(void);
#else
void EmergencySetStatus(int enable);
void ComputeEmergency(void);
#endif

#endif
