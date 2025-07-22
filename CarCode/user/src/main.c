
#include "zf_common_headfile.h"
#include "menu.h"
#include "encoder.h"
#include "key.h"
#include "pid_v.h"
#include "flash.h"//老大
#include "motor.h"
#include "photo_chuli.h"
#include "screen.h"
#include "track.h"
#include "steer_pid.h"
#include "buzzer.h"
bool save_flag=false;
bool stop_flag1;                            //停�?�标志�??
extern bool start_flag;                     //发车标识�?

extern uint8 leftline_num;//左线点数�?
extern uint8 rightline_num;//右线点数�?
extern struct pid_v PID_V;                  //pid_V
extern struct steer_pid S_PID;
extern int current_state;
extern int speed; 
extern int forwardsight;
extern int encodercounter1;

void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);//必须最先开�?时钟
    debug_init();
    Menu_Screen_Init();             //屏幕显示初�?�化
	system_delay_ms(300);
    flash_init();    
    Key_init();                     //������ʼ��
    BUZZ_init();
    motor_init();
    S_PID_CAL_init();
    while(1)//摄像�?... 
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            pit_ms_init(TIM7_PIT, 5);                                                   //�?件定时，周期5ms
            interrupt_set_priority(TIM7_IRQn, 1);

            break;
        }
        system_delay_ms(50);
        
    }

}

void flash_save(void)
 {
    if(save_flag)
    {
        if(flash_check(100, 0)){flash_erase_page(100, 0);}
        flash_buffer_clear();
        
        //100,0flashдPID_V����
        flash_union_buffer[0].float_type=PID_V.p;           //����ϵ��
        flash_union_buffer[1].float_type=PID_V.i;           //����ϵ��
        flash_union_buffer[2].float_type=PID_V.d;           //΢��ϵ��
        flash_union_buffer[3].float_type=PID_V.i_max;       //�����޷�
        flash_union_buffer[4].float_type=PID_V.d_max;       //΢���޷�
        flash_union_buffer[5].float_type=PID_V.output_max;  //����޷�
        
        flash_erase_page(100,0);
        flash_write_page_from_buffer(100,0);        // flashд

        
        
        if(flash_check(100, 1)){flash_erase_page(100, 1);}
        flash_buffer_clear();
        //100,1 flashдS_PID����
        flash_union_buffer[0].float_type=S_PID.p;           //����ϵ��     
        flash_union_buffer[1].float_type=S_PID.i;           //����ϵ��
        flash_union_buffer[2].float_type=S_PID.d;           //΢��ϵ��
        flash_union_buffer[3].float_type=S_PID.outputmax;   //����޷�
        flash_union_buffer[4].float_type=S_PID.outputmin;   //����޷�
        
        flash_erase_page(100,1);
        flash_write_page_from_buffer(100,1);        // flashд

        if(flash_check(100, 2)){flash_erase_page(100, 2);}
        flash_buffer_clear();

        flash_union_buffer[0].int32_type=speed;             //���ٶ�״̬
        flash_union_buffer[1].int32_type=forwardsight;      //ǰհ״̬
        
        flash_erase_page(100,2);
        flash_write_page_from_buffer(100,2);        // falshд

        save_flag=false;
    }
}

int main (void)
{
    all_init();
    stop_flag1=false;
    while(1)
    { 
        Key_Scan();             //����ɨ��
        Menu_control();         //�˵�����
        flash_save();           //���汣��
		BUZZ_cycle();           //������ѭ��
        

    }
        
}
