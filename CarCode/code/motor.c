#include "motor.h"

#define DIR_L               (A0 )
#define PWM_L               (TIM5_PWM_CH2_A1)

#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH4_A3)


bool dir = true;
void motor_init()
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_L, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM_R, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
}
//PWM_DUTY_MAX=10000

void motor_run(int16 a,int16 b) 
{

    if(0 <= a)                                                           // ��ת
        {
            gpio_set_level(DIR_L, GPIO_HIGH);                                   // DIR����ߵ�ƽ
            pwm_set_duty(PWM_L, (uint16)(a * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
                 // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {
            gpio_set_level(DIR_L, GPIO_LOW);                                    // DIR����͵�ƽ
            pwm_set_duty(PWM_L, (uint16)((-a) * (PWM_DUTY_MAX / 10000)));                // ����ռ�ձ�

                // ����ռ�ձ�
        }
     if(0 <= b)                                                           // ��ת
        {

            gpio_set_level(DIR_R, GPIO_HIGH);                                   // DIR����ߵ�ƽ
            pwm_set_duty(PWM_R, (uint16)(b * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {

            gpio_set_level(DIR_R, GPIO_LOW);                                    // DIR����͵�ƽ
            pwm_set_duty(PWM_R, (uint16)((-b) * (PWM_DUTY_MAX / 10000)));                // ����ռ�ձ�
        }
}