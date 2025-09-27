#ifndef __SAMPLE__H
#define __SAMPLE__H

#include "motor.h"
#include "stdio.h"
#include "main.h"
#include "adc.h"

// ������·����
#define R_SHUNT 0.01f              // ������������
#define OP_GAIN 100.0f             // �˷ŷŴ���
#define ADC_REFERENCE_VOLT 3.2f    // ��������adc�ο���ѹ
#define ADC_BITS 4096.0f           // ADC����
#define ADC_SCALE (ADC_REFERENCE_VOLT / ADC_BITS)
#define CURRENT_FILTER_ALPHA 0.9f  // �˲�ϵ��

extern volatile uint16_t adc_raw_ia;
extern volatile uint16_t adc_raw_ic;
extern volatile uint8_t current_loop_enable;

void ADC_Calibration(uint16_t cnt);
float VoltageToCurrent(uint32_t ADCValue, uint16_t offset_adc);

#endif

