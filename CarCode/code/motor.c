#include "motor.h"

#define PWM_L1               (TIM5_PWM_CH4_A3)
#define PWM_L2               (TIM5_PWM_CH2_A1)


#define PWM_R1               (TIM5_PWM_CH1_A0)
#define PWM_R2               (TIM5_PWM_CH3_A2)


void motor_init()
{
    pwm_init(PWM_L1, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_L2, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_R1, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_R2, 17000, 0);  
}

void motor_run(int16 a,int16 b) 
{

    if(0 <= a)                                                           // 正转
        {
            pwm_set_duty(PWM_L2,0);
            pwm_set_duty(PWM_L1, (uint16)(a * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
                 // 计算占空比 bS
        }
        else                                                                    // 反转
        {

            pwm_set_duty(PWM_L1,0);
            pwm_set_duty(PWM_L2, (uint16)(-a * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
                // 计算占空比
        } 
 
     if(0 <= b)                                                           // 正转
        {
            pwm_set_duty(PWM_R2,0);
            pwm_set_duty(PWM_R1, (uint16)(b * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
        }
        else// 反转
        {
 
            pwm_set_duty(PWM_R1,0);
            pwm_set_duty(PWM_R2, (uint16)(-b * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
        }
}