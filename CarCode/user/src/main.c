/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
* 
* �?文件�? MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
* 
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
* 
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
* 
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
* 
* 文件名称          main
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* �?改�?�录
* 日期              作�?                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执�?�以下操�?
// �?一�? 关闭上面所有打开的文�?
// �?二�?? project->clean  等待下方进度条走�?


// *************************** 例程�?件连接�?�明 ***************************
//      单排排针 SPI 两�?�屏 �?件引�?
//      SCL                 查看 zf_device_ips200.h �? IPS200_SCL_PIN_SPI  宏定�? A5
//      SDA                 查看 zf_device_ips200.h �? IPS200_SDA_PIN_SPI  宏定�? A7
//      RST                 查看 zf_device_ips200.h �? IPS200_RST_PIN_SPI  宏定�? A6
//      DC                  查看 zf_device_ips200.h �? IPS200_DC_PIN_SPI   宏定�? D0
//      CS                  查看 zf_device_ips200.h �? IPS200_CS_PIN_SPI   宏定�? A4
//      BL                  查看 zf_device_ips200.h �? IPS200_BLk_PIN_SPI  宏定�? D1
//      GND                 核心板电源地 GND
//      3V3                 核心�? 3V3 电源



// *************************** 例程测试说明 ***************************
// 1.核心板烧录本例程 插在主板�? 2寸IPS 显示模块插在主板的屏幕接口排座上 请注意引脚�?�应 不�?�插�?
// 
// 2.电池供电 上电�? 2寸IPS 屏幕�?�? 显示字�?�数字浮点数和波形图
// 
// 如果发现现象与�?�明严重不�?? 请参照本文件最下方 例程常�?�问题�?�明 进�?�排�?


// **************************** 代码区域 ****************************
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
extern int image_threshold;
extern uint8 dis_image[MT9V03X_H][MT9V03X_W];
void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);//必须最先开�?时钟
    debug_init();
    Menu_Screen_Init();             //屏幕显示初�?�化
	system_delay_ms(300);
    flash_init();    
    Key_init();                     //按键初�?�化
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
        
        //100,0储存pid_v的数�?
        flash_union_buffer[0].float_type=PID_V.p;
        flash_union_buffer[1].float_type=PID_V.i;    
        flash_union_buffer[2].float_type=PID_V.d;
        flash_union_buffer[3].float_type=PID_V.i_max;
        flash_union_buffer[4].float_type=PID_V.d_max;    
        flash_union_buffer[5].float_type=PID_V.output_max;
        
        flash_erase_page(100,0);
        flash_write_page_from_buffer(100,0);        // 向指�? Flash 扇区的页码写入缓冲区数据

        //100,1储存图象处理的数�?
        
        
        if(flash_check(100, 1)){flash_erase_page(100, 1);}
        flash_buffer_clear();

        flash_union_buffer[0].float_type=S_PID.p;
        flash_union_buffer[1].float_type=S_PID.i;    
        flash_union_buffer[2].float_type=S_PID.d;
        flash_union_buffer[3].float_type=S_PID.outputmax;
        flash_union_buffer[4].float_type=S_PID.outputmin;
        
        flash_erase_page(100,1);
        flash_write_page_from_buffer(100,1);        // 向指�? Flash 扇区的页码写入缓冲区数据

        if(flash_check(100, 2)){flash_erase_page(100, 2);}
        flash_buffer_clear();

        flash_union_buffer[0].int32_type=speed;
        flash_union_buffer[1].int32_type=forwardsight;
        
        flash_erase_page(100,2);
        flash_write_page_from_buffer(100,2);        // 向指�? Flash 扇区的页码写入缓冲区数据

        save_flag=false;
    }
}

int main (void)
{
    all_init();
    stop_flag1=false;
    while(1)
    { 
        Key_Scan();             //按键�?�?
        Menu_control();         //菜单控制
        flash_save();           //flash�?�?
		BUZZ_cycle();           //蜂鸣器循�?
        if(mt9v03x_finish_flag)
        { 
            image_threshold=my_adapt_threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);//图像获取阈�?
             set_b_imagine(image_threshold);
            image_boundary_process2();
            if(current_state==1)
            {
                
                 ips200_show_gray_image(0,120,(const uint8 *)dis_image,MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H,0);       //图像处理�?注释�?
                element_check();
                show_line(); 
            }                                                                   
			if( encodercounter1>7000)
			{	
				banmaxian_check();//�����߼��
			}
            black_protect_check();  //��ɫ�������
            if(stop_flag1)
            {
            pit_disable(TIM6_PIT);  
            motor_run(0,0 );

            }
            mt9v03x_finish_flag = 0;
            
        } 

    }
        
}





















// **************************** 代码区域 ****************************

// *************************** 例程常�?�问题�?�明 ***************************
// 遇到�?题时请按照以下问题�?�查列表�?��?
// 
// �?�?1：屏幕不显示
//      如果使用主板测试，主板必须�?�用电池供电 检查屏幕供电引脚电�?
//      检查屏幕是不是插错位置�? 检查引脚�?�应关系
//      如果对应引脚都�?�确 检查一下是否有引脚波形不�?? 需要有示波�?
//      无法完成波形测试则�?�制一个GPIO例程将屏幕所有IO初�?�化为GPIO翻转电平 看看�?否受�?
