
#include "zf_common_headfile.h"
#include "menu.h"
#include "encoder.h"
#include "key.h"
#include "pid_v.h"
#include "flash.h"//鑰佸ぇ
#include "motor.h"
#include "photo_chuli.h"
#include "screen.h"
#include "track.h"
#include "steer_pid.h"
#include "buzzer.h"
bool save_flag=false;
bool stop_flag1;                            //鍋滄?㈡爣蹇楃??
extern bool start_flag;                     //鍙戣溅鏍囪瘑绗?

extern uint8 leftline_num;//宸︾嚎鐐规暟閲?
extern uint8 rightline_num;//鍙崇嚎鐐规暟閲?
extern struct pid_v PID_V;                  //pid_V
extern struct steer_pid S_PID;
extern int current_state;
extern int speed; 
extern int forwardsight;
extern int encodercounter1;

void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);//蹇呴』鏈�鍏堝紑鍚?鏃堕挓
    debug_init();
    Menu_Screen_Init();             //灞忓箷鏄剧ず鍒濆?嬪寲
	system_delay_ms(300);
    flash_init();    
    Key_init();                     //按键初始化
    BUZZ_init();
    motor_init();
    S_PID_CAL_init();
    while(1)//鎽勫儚澶?... 
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            pit_ms_init(TIM7_PIT, 5);                                                   //纭?浠跺畾鏃讹紝鍛ㄦ湡5ms
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
        
        //100,0flash写PID_V参数
        flash_union_buffer[0].float_type=PID_V.p;           //比例系数
        flash_union_buffer[1].float_type=PID_V.i;           //积分系数
        flash_union_buffer[2].float_type=PID_V.d;           //微分系数
        flash_union_buffer[3].float_type=PID_V.i_max;       //积分限幅
        flash_union_buffer[4].float_type=PID_V.d_max;       //微分限幅
        flash_union_buffer[5].float_type=PID_V.output_max;  //输出限幅
        
        flash_erase_page(100,0);
        flash_write_page_from_buffer(100,0);        // flash写

        
        
        if(flash_check(100, 1)){flash_erase_page(100, 1);}
        flash_buffer_clear();
        //100,1 flash写S_PID参数
        flash_union_buffer[0].float_type=S_PID.p;           //比例系数     
        flash_union_buffer[1].float_type=S_PID.i;           //积分系数
        flash_union_buffer[2].float_type=S_PID.d;           //微分系数
        flash_union_buffer[3].float_type=S_PID.outputmax;   //输出限幅
        flash_union_buffer[4].float_type=S_PID.outputmin;   //输出限幅
        
        flash_erase_page(100,1);
        flash_write_page_from_buffer(100,1);        // flash写

        if(flash_check(100, 2)){flash_erase_page(100, 2);}
        flash_buffer_clear();

        flash_union_buffer[0].int32_type=speed;             //车速度状态
        flash_union_buffer[1].int32_type=forwardsight;      //前瞻状态
        
        flash_erase_page(100,2);
        flash_write_page_from_buffer(100,2);        // falsh写

        save_flag=false;
    }
}

int main (void)
{
    all_init();
    stop_flag1=false;
    while(1)
    { 
        Key_Scan();             //按键扫描
        Menu_control();         //菜单控制
        flash_save();           //闪存保存
		BUZZ_cycle();           //蜂鸣器循环
        

    }
        
}
