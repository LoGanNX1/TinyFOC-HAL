#include "sample.h"

// ������ʹ�ܱ�־
volatile uint8_t current_loop_enable = 0;

// ����ȫ�ֱ������洢ADCԭʼֵ
volatile uint16_t adc_raw_ia = 0;
volatile uint16_t adc_raw_ic = 0;

// У׼����оƬ���ֵ
void ADC_Calibration(uint16_t cnt) {
    HAL_Delay(10);  // �ȴ��Ŵ����ȶ�

    int32_t ia_sum = 0;
    int32_t ic_sum = 0;

    for (uint16_t i = 0; i < cnt; i++) 
	  {  
        ia_sum += adc_raw_ia;
        ic_sum += adc_raw_ic;

        HAL_Delay(1);  // ��ADC�ȶ�ʱ��
    }

    motor_control.IphA_offset = ia_sum / cnt;
    motor_control.IphC_offset = ic_sum / cnt;

    printf("ADC Zero Calibration Done!\r\n");
	printf("Mid_A:%d\tMid_C:%d\r\n",motor_control.IphA_offset,motor_control.IphC_offset);
}

// ����������ģ���ѹֵת��Ϊʵ�ʵ���ֵ
float VoltageToCurrent(uint32_t ADCValue, uint16_t offset_adc) {
    int adcval_diff = (int)ADCValue - (int)offset_adc;
    float amp_voltage = (ADC_REFERENCE_VOLT / ADC_BITS) * adcval_diff;  // ʵ�����ǷŴ��ĵ�ѹ
    float current = amp_voltage / (OP_GAIN * R_SHUNT);  			    // ��������ͷ�������
    return current;
}

/**
  * @brief  ע��ת����ɻص�����
  * @param  hadc ADC���ָ��
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    // ��ע�����ȡ��ֵ
    adc_raw_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc_raw_ic = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	  // ��ȡ�����������
    motor_control.IphA = VoltageToCurrent(adc_raw_ia, motor_control.IphA_offset);
    motor_control.IphC = - VoltageToCurrent(adc_raw_ic, motor_control.IphC_offset);
    motor_control.IphB = - motor_control.IphC - motor_control.IphA; // ��·���C���A����������෴
    // �ײ������
    if(current_loop_enable == 1) {
      foc_current_loop();
    }
  }
}