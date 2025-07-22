#include "motor.h"

#define DIR_L               (A0 )
#define PWM_L               (TIM5_PWM_CH2_A1)

#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH4_A3)


bool dir = true;
void motor_init()
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
}
//PWM_DUTY_MAX=10000

void motor_run(int16 a,int16 b) 
{

    if(0 <= a)                                                           // 正转
        {
            gpio_set_level(DIR_L, GPIO_HIGH);                                   // DIR输出高电平
            pwm_set_duty(PWM_L, (uint16)(a * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
                 // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR_L, GPIO_LOW);                                    // DIR输出低电平
            pwm_set_duty(PWM_L, (uint16)((-a) * (PWM_DUTY_MAX / 10000)));                // 计算占空比

                // 计算占空比
        }
     if(0 <= b)                                                           // 正转
        {

            gpio_set_level(DIR_R, GPIO_HIGH);                                   // DIR输出高电平
            pwm_set_duty(PWM_R, (uint16)(b * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
        }
        else                                                                    // 反转
        {

            gpio_set_level(DIR_R, GPIO_LOW);                                    // DIR输出低电平
            pwm_set_duty(PWM_R, (uint16)((-b) * (PWM_DUTY_MAX / 10000)));                // 计算占空比
        }
}