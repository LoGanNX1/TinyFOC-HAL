#include "pid.h"

#define MAX_ANGLE_SPEED        100.0f          // ���Ƕ��ٶ��޷�
#define MAX_IQ_CURRENT         LIMIT_CURRENT   // ���Iq�����޷�
#define MAX_MODULATION         12 * 0.9f       // ����������������Ƶ�ѹ

struct PIDController angle_loop  = {.P = 2.0f, .I = 0.0f,  .limit = MAX_ANGLE_SPEED};  // ���Iq(A)
struct PIDController vel_loop    = {.P = 2.0f, .I = 20.0f, .limit = MAX_IQ_CURRENT};   // ���Iq(A)
struct PIDController current_loop= {.P = 1.0f, .I = 10.0f, .limit = MAX_MODULATION};   // �����һ����ѹ(0~1)

/**
 * @brief   ���õ���ٶȻ��� PID ����
 * @param   P       �������� 
 * @param   I       ��������
 * @param   D       ΢������ 
 * @param   ramp    ���б������ ��λ/��
 * @param   limit   ����޷�
 * @param   alpha   �ٶȲ���ֵ�ĵ�ͨ�˲�ϵ��
 */
void foc_set_vel_pid(float P, float I, float D, float ramp, float limit, float alpha)   
{
	vel_loop.P=P;
	vel_loop.I=I;
	vel_loop.D=D;
	vel_loop.output_ramp=ramp;
	vel_loop.limit=limit;
	motor_control.vel_lowpass_alpha = alpha;
}

/**
 * @brief   ���õ���ǶȻ��� PID ����
 * @param   P       ��������
 * @param   I       ��������
 * @param   D       ΢������
 * @param   ramp    ���б������
 * @param   limit   ����޷�
 */
void foc_set_angle_pid(float P,float I,float D,float ramp,float limit)   
{
	angle_loop.P=P;
	angle_loop.I=I;
	angle_loop.D=D;
	angle_loop.output_ramp=ramp;
	angle_loop.limit=limit;
}    

/**
 * @brief	���õ���������� PID ����
 * @param   P       ��������
 * @param   I       ��������
 * @param   D       ΢������
 * @param   ramp    ���б������
 */
void foc_set_current_pid(float P,float I,float D,float ramp)  
{
	current_loop.P=P;
	current_loop.I=I;
	current_loop.D=D;
	current_loop.output_ramp=ramp;
}

/**
 * @brief   ���� PID ��������״̬���������
 * @param   pid     ָ�� PID �������ṹ���ָ��
 * @param   error   ��ǰ���������
 * @return  float   PID �������ļ������
 */
float PIDController_Update(struct PIDController* pid, float error) {
    // ��ȡ��ǰʱ���
    unsigned long timestamp_now = dwt_get_micros(); 

    // ��ֹϵͳ��������ʱ�����ʱ���ֳ������
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // ������
    float proportional = pid->P * error;

    // ������ ʹ��Tustin�����Σ�����������ɢ�� �ȼ򵥵�ǰ��ŷ��������ȷ
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    if (integral > pid->limit) integral = pid->limit;
    else if (integral < -pid->limit) integral = -pid->limit;

    // ΢����
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // ���������
    float output = proportional + integral + derivative;

    // ����޷�
    if (output > pid->limit) output = pid->limit;
    else if (output < -pid->limit) output = -pid->limit;

    // ���б������ ��ֹ���ֵͻ�� ʹϵͳ��Ӧ��ƽ��
    if (pid->output_ramp > 0) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    // ������״̬������Ϊ��һ�ε�����׼��
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;

    return output;
}

/**
 * @brief   ���ݸ߼������������ʼ�����е�� PID ������
 * @param   tor_p   ת�ػ�(������) P����
 * @param   tor_i   ת�ػ�(������) I����
 * @param   vel_p   �ٶȻ� P����
 * @param   vel_i   �ٶȻ� I����
 * @param   pos_p   λ�û�(�ǶȻ�) P����
 */
void motor_pid_init(float tor_p, float tor_i, float vel_p, float vel_i, float pos_p)
{
	motor_config.torque_gain = tor_p;
	motor_config.torque_integrator_gain = tor_i;
	motor_config.vel_gain = vel_p;
	motor_config.vel_integrator_gain = vel_i;
	motor_config.pos_gain = pos_p;
	
	foc_set_angle_pid(motor_config.pos_gain, 0, 0, 100000, LIMIT_CURRENT);  // λ�û��ڻ�Ϊ������
    foc_set_vel_pid(motor_config.vel_gain, motor_config.vel_integrator_gain , 0.00, 100000, LIMIT_CURRENT, VEL_ALPHA); 
    foc_set_current_pid(motor_config.torque_gain, motor_config.torque_integrator_gain, 0, 0);
}

