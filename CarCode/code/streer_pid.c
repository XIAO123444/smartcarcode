#include "steer_pid.h"
#include "photo_chuli.h"
#include "track.h"

struct steer_pid S_PID;
struct steer_pid S_PID1;
void PID_init(void)
{
    S_PID.p=2;
    S_PID.i=0;
    S_PID.d=0.01;
    S_PID.outputmax=140;
    S_PID.outputmin=-140;
    S_PID.target=140/2;
}
void PID2_init(void)
{
    S_PID1.p=2;
    S_PID1.i=0;
    S_PID1.d=0.01;
    S_PID1.outputmax=140;
    S_PID1.outputmin=-140;
    S_PID1.target=140/2;
}   
float LastLasterror=0;

volatile float derivative=0;
volatile float intgral=0;
volatile float error=0;
volatile float Lasterror=0;


volatile float derivative1=0;
volatile float intgral1=0;  
volatile float error1=0;
volatile float Lasterror1=0;

struct steer_pid* SPID_vget_param(void)
{
    return &S_PID;
}
struct steer_pid* SPID1_vget_param(void)
{
    return &S_PID1;
}

int S_PID_CAL()
{
    //int16 measure=output_middle(); 
    int16 measure =output_middle2();//补线成功就用这个代码
    error = 80-(float)measure;//大于0的时候是左偏移<0右偏
    intgral+=error;
    derivative=error-Lasterror;
    int result=( int )(S_PID.p*error+S_PID.i*intgral+S_PID.d*derivative);
    if(intgral>18)
    {
        intgral=18;
    }
    if(intgral<-18)
    {
        intgral= -18 ;
    }
    if(result>90)
    {
        result=90;
    }
    if(result<-90)
    {
        result=-90;

    }
    Lasterror=error;
//    printf("%d",result);
    return result;
}
int S_PID1_CAL()
{
    int16 measure=output_middle2();
    error1 = 80-(float)measure;
    intgral1+=error1;
    derivative1=error1-Lasterror1;
    int result=( int )(S_PID1.p*error1+S_PID1.i*intgral1+S_PID1.d*derivative1);
    if(intgral1>18)
    {
        intgral1=18;
    }
    if(intgral1<-18)
    {
        intgral1= -18 ;
    }
    if(result>90)
    {
        result=90;
    }
    if(result<-90)
    {
        result=-90;

    }
    Lasterror1=error1;
    return result;
}



void S_PIDadd_p(void){
    
    S_PID.p+=0.1;
}
void S_PIDadd_i(void) {
    S_PID.i += 0.1;  


}
void S_PIDadd_d(void) {
    S_PID.d += 0.01;  

}    


void S_PIDadd_outputmax(void) {
    S_PID.outputmax += 100;
    S_PID.outputmin -= 100;

}
void S_PIDsub_p(void) {
    if (S_PID.p > 0.1) {  
        S_PID.p -= 0.1;   
        if (S_PID.p < 0) S_PID.p = 0;  
    }
}
void S_PIDsub_i(void) {
    if (S_PID.i > 0.01) {
        S_PID.i -= 0.01;
        if (S_PID.i < 0) S_PID.i = 0;
    }
}
void S_PIDsub_d(void) {
    if (S_PID.d > 0.01) {
        S_PID.d -= 0.01;
        if (S_PID.d < 0) S_PID.d = 0;
        // 微分限幅保护
        }
}


void S_PIDsub_outputmax(void) {
    if (S_PID.outputmax > 100) {  
        S_PID.outputmax -= 100;
        S_PID.outputmin += 100;

    }
}


void S_PID1add_p(void){
    
    S_PID1.p+=0.1;
}
void S_PID1add_i(void) {
    S_PID1.i += 0.1;  
}

void S_PID1add_d(void) {
    S_PID1.d += 0.01;  

}   
void S_PID1add_outputmax(void) {
    S_PID1.outputmax += 100;
    S_PID1.outputmin -= 100;

}
void S_PID1sub_p(void) {
    if (S_PID1.p > 0.1) {  
        S_PID1.p -= 0.1;   
        if (S_PID1.p < 0) S_PID1.p = 0;  
    }
}



void S_PID1sub_i(void) {
    if (S_PID1.i > 0.01) {
        S_PID1.i -= 0.01;
        if (S_PID1.i < 0) S_PID1.i = 0;
    }
}   
void S_PID1sub_d(void) {
    if (S_PID1.d > 0.01) {
        S_PID1.d -= 0.01;
        if (S_PID1.d < 0) S_PID1.d = 0;
        // 微分限幅保护
        }
}

void S_PID1sub_outputmax(void) {
    if (S_PID1.outputmax > 100) {  
        S_PID1.outputmax -= 100;
        S_PID1.outputmin += 100;

    }
}
    