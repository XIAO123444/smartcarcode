#include "speed.h"
#include "track.h"
#include "photo_chuli.h"
#include "menu.h"
#include "steer_pid.h"

extern bool start_flag;
extern float error;
extern int32 speed;

uint8 car_situation=0;
int32 speed_stragety;

void Velocity_Control(void)//���������б���ѡ���ٶ�
{
    if(start_flag==true)
    {
        if(error>40||error<-40)
        {
            car_situation=1;//���
        }
        else
        {
            car_situation=0;
        }
        if(car_situation==0)//ֱ��
        {
            speed_stragety=speed*1.5;
        }
        if(car_situation==1)
        {
            speed_stragety=speed;
        }
        
        
    }

}
