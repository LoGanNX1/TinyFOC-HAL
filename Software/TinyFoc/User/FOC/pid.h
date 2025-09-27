#ifndef __PID__H
#define __PID__H

#include "stm32f4xx_hal.h"
#include "utils.h"
#include "motor.h"
#include "main.h"
#include "foc.h"


extern struct PIDController vel_loop;
extern struct PIDController angle_loop;
extern struct PIDController current_loop;

struct PIDController
{
	float P; //!< ��������(P������)
	float I; //!< �������棨I�����棩
	float D; //!< ΢�����棨D�����棩
	float output_ramp; 
	float limit; 
	float ramp; 

	float error_prev; //!< ���ĸ������ֵ
	float output_prev;  //!< ���һ�� pid ���ֵ
	float integral_prev; //!< ���һ�����ַ���ֵ
	unsigned long timestamp_prev; //!< �ϴ�ִ��ʱ���
};

void foc_set_vel_pid(float P, float I, float D, float ramp, float limit, float alpha) ;
void foc_set_angle_pid(float P,float I,float D,float ramp,float limit);
void foc_set_current_pid(float P,float I,float D,float ramp);

float PIDController_Update(struct PIDController* pid, float error);
void motor_pid_init(float tor_p, float tor_i, float vel_p, float vel_i, float pos_p);
#endif