/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "parameters.h"
#include "brushlessMotor.h"

//Controleur :
//0:127   : Marche arriere
//127:255 : Marche avant

void set_pwm(int side, int pwm) {
	if (side == MOTOR_LEFT) {
		//les moteurs sont faces à face, pour avancer 
		//il faut qu'il tournent dans un sens différent
		pwm = -pwm;
	}

	pwm = (int)(pwm/2.0) + 127;

	if(pwm > 255)
		pwm = 255;
	else if(pwm < 0)
		pwm = 0;
    // Test to make break better and no more free wheel
//    if ((pwm > 121) && (pwm < 133)) {
//        pwm = 121;
//    }
	BrushlessMotorSetPwm(side, pwm);
}

void get_breaking_speed_factor(float *angular_speed, float *linear_speed) {
    *angular_speed = (float)5.0;
    *linear_speed = (float)5.0;
}

void apply_break(int pwm) {
    BrushlessMotorBreak(pwm);
}
