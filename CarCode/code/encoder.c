#include "encoder.h"

int32 encoder1;
int32 encoder2;

void Encoder_Init()
{
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);  //������������ʼ��
	encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7);   //�����������ʼ��
 	pit_ms_init(TIM6_PIT, 5);                                                 //Ӳ����ʱ������1ms
	interrupt_set_priority(TIM6_IRQn, 2);                                           
   
}

int Encoder_GetInfo_L(void)
{
     return encoder2;
}
int Encoder_GetInfo_R(void)
{
    return encoder1;
}