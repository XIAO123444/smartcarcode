#include "zf_common_headfile.h"
#include "encoder.h"

 float ERR=0.0,LastERR=0.0,LastLastERR=0.0;
 float pwm_ccr=0,add_ccr=0;
 float i1,d1;
  float ERR1=0.0,LastERR1=0.0,LastLastERR1=0.0;
 float pwm_ccr1=0,add_ccr1=0;
 float i2,d2;
 
 

  struct pid_v
{
    float output_max;       //输出限幅   
    float p;            //比例系数
    float i;            //积分系数
    float d;            //微分系数
    float i_max;        //积分限幅
    float d_max;        //微分限幅
    
}PID_V;
void pid_vparam_init(void)
{
    PID_V.d=0;
    PID_V.d_max=0.1;
    PID_V.i=0;
    PID_V.i_max=10;
    PID_V.output_max=100;
    PID_V.p=0;
}

struct pid_v* PID_vget_param(void)
{
    return &PID_V;
}
void PID_vset_param(struct pid_v* p)
{
    PID_V.p         =p          ->p;
    PID_V.d         =p          ->d;
    PID_V.d_max     =p          ->d_max;
    PID_V.i         =p          ->i;
    PID_V.i_max     =p          ->i_max;
    PID_V.output_max=p          ->output_max;

}
//int pid_control1(int target1)//左轮
//{
//     ERR= (float)target1 - (float)Encoder_GetInfo_L();
//    add_ccr=PID_V.p*(ERR-LastERR)+PID_V.i*ERR+PID_V.d*(ERR+LastLastERR-2*LastERR);
//    if(1)
//    {
//        pwm_ccr+=add_ccr;
//    }
//    if(pwm_ccr>5000)
//        pwm_ccr=5000;
//    if(pwm_ccr<-5000)
//        pwm_ccr=-5000;
//    LastLastERR=LastERR;
//    LastERR=ERR;
//    return (int)pwm_ccr;
//}



//int pid_control2(int target2)//右轮
//{
//     ERR1= (float)target2 - (float)Encoder_GetInfo_R();  
//    add_ccr1=PID_V.p*(ERR1-LastERR1)+PID_V.i*ERR1+PID_V.d*(ERR1+LastLastERR1-2*LastERR1);
//    if(add_ccr1<-1||add_ccr1>1)
//    {
//        pwm_ccr1+=add_ccr1;
//    }
//    if(pwm_ccr1>5000)
//        pwm_ccr1=5000;
//    if(pwm_ccr1<-5000)
//        pwm_ccr1=-5000;
//    LastLastERR1=LastERR1;
//    LastERR1=ERR1;
//    return (int)pwm_ccr1;
//}



int pid_V_comon(int target)
{
    ERR=(float)(target*2)-(float)(Encoder_GetInfo_L()+Encoder_GetInfo_R());
    d1=ERR-LastERR;
    i1+=ERR;
    if(i1>=4000)
    {
        i1=4000;
    }
    if(i1<=-4000)
    {
        i1=-4000;
    }

    int result=(int)(PID_V.p*ERR+PID_V.i*i1+PID_V.d*d1);
        if(result>=5000)
    {
        result=5000;
    }    
        if(result<=-5000)
    {
        result=-5000;
    }
    return result;
}





//int pid_control2(int target1)//右轮
//{
//     ERR1= (float)target1 - (float)Encoder_GetInfo_R();
//    d2=ERR1-LastERR1;
//    i2+=ERR1;
//    if(i2>=2000)
//    {
//        i2=2000;
//    }
//    if(i2<-2000)
//    {
//        i2=-2000; 
//    }
//    pwm_ccr1=PID_V.p*ERR1+PID_V.i*i2+PID_V.d*d2; 
//    if(pwm_ccr1>5000)
//        pwm_ccr1 =5000;
//    if(pwm_ccr1<-5000)
//        pwm_ccr1=-5000;
//    LastERR1=ERR1;
//    return (int)pwm_ccr1;
//}





//int pid_control1(int target1)//左轮
//{
//    int a=Encoder_GetInfo_L();
//     ERR= (float)target1 - (float)a;
//    d1=ERR-LastERR;
//    i1+=ERR;
//    if(i1>=2000)
//    {
//        i1=2000;
//    }
//    if(i1<-2000)
//    {
//        i1=-2000; 
//    }
//    pwm_ccr=PID_V.p*ERR+PID_V.i*i1+PID_V.d*d1; 
//    if(pwm_ccr>5000)
//        pwm_ccr=5000;
//    if(pwm_ccr<-5000)
//        pwm_ccr=-5000;
//    LastERR=ERR;
//    printf("%d,%d\n",target1,a);
//    return (int)pwm_ccr;
//}




































void pid_add_p(void){
    
    PID_V.p+=0.1;
}
void pid_add_i(void) {
    PID_V.i += 0.1;  


}
void pid_add_d(void) {
    PID_V.d += 0.1;  

}    
void pid_add_i_max(void) {
    PID_V.i_max += 0.1; 
}
void pid_add_d_max(void) {
    PID_V.d_max += 0.1;
}
void pid_add_output_max(void) {
    PID_V.output_max += 1.0;
}
void pid_sub_p(void) {
    if (PID_V.p > 0.1) {  
        PID_V.p -= 0.1;   
        if (PID_V.p < 0) PID_V.p = 0;  
    }
}
void pid_sub_i(void) {
    if (PID_V.i > 0.1) {
        PID_V.i -= 0.1;
        if (PID_V.i < 0) PID_V.i = 0;
    }
}
void pid_sub_d(void) {
    if (PID_V.d > 0.1) {
        PID_V.d -= 0.1;
        if (PID_V.d < 0) PID_V.d = 0;
        // 微分限幅保护
        }
}

void pid_sub_d_max(void) {
    if (PID_V.d_max > 0.1) {
        PID_V.d_max -= 0.1;
        if (PID_V.d_max < 0) PID_V.d_max = 0;
    }
}
void pid_sub_i_max(void) {
    if (PID_V.i_max > 0.1) {
        PID_V.i_max -= 0.1;
        if (PID_V.i_max < 0) PID_V.i_max = 0;
    }
}
void pid_sub_output_max(void) {
    if (PID_V.output_max > 0.1) {  // 输出限幅通常需大于执行机构最小需求
        PID_V.output_max -= 0.1;
    }
}







