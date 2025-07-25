#include "speed.h"
#include "track.h"
#include "photo_chuli.h"
#include "menu.h"
#include "steer_pid.h"

extern bool start_flag;
extern float error;
extern int32 speed;             //基础速度
extern int32 speed_stragety;    //决策速度

uint8 car_situation=0;

void Velocity_Control(void)//赛道类型判别，来选定速度
{
    
    if(start_flag==true)
    {
        if(error>10||error<-10)
        {
            car_situation=1;//弯道
        } 
        else
        {
            car_situation=0;
        } 
        if(car_situation==0)//直道
        {
            speed_stragety=speed*1.2;
        }
        if(car_situation==1)
        {
            speed_stragety=speed*0.9;
        }
        
        
    }

}
