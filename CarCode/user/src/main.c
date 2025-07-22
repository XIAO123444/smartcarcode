/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// *************************** 例程硬件连接说明 ***************************
//      单排排针 SPI 两寸屏 硬件引脚
//      SCL                 查看 zf_device_ips200.h 中 IPS200_SCL_PIN_SPI  宏定义 A5
//      SDA                 查看 zf_device_ips200.h 中 IPS200_SDA_PIN_SPI  宏定义 A7
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_SPI  宏定义 A6
//      DC                  查看 zf_device_ips200.h 中 IPS200_DC_PIN_SPI   宏定义 D0
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_SPI   宏定义 A4
//      BL                  查看 zf_device_ips200.h 中 IPS200_BLk_PIN_SPI  宏定义 D1
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源



// *************************** 例程测试说明 ***************************
// 1.核心板烧录本例程 插在主板上 2寸IPS 显示模块插在主板的屏幕接口排座上 请注意引脚对应 不要插错
// 
// 2.电池供电 上电后 2寸IPS 屏幕亮起 显示字符数字浮点数和波形图
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


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
bool stop_flag1;                            //停止标志符
extern bool start_flag;                     //发车标识符

extern uint8 leftline_num;//左线点数量
extern uint8 rightline_num;//右线点数量
extern struct pid_v PID_V;                  //pid_V
extern struct steer_pid S_PID;
extern int current_state;
extern int speed; 
extern int forwardsight;
extern int encodercounter1;

void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);//必须最先开启时钟
    debug_init();
    Menu_Screen_Init();             //屏幕显示初始化
	system_delay_ms(300);
    flash_init();    
    Key_init();                     //按键初始化
    BUZZ_init();
    motor_init();
    S_PID_CAL_init();
    while(1)//摄像头... 
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            pit_ms_init(TIM7_PIT, 5);                                                   //硬件定时，周期5ms
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
        
        //100,0储存pid_v的数据
        flash_union_buffer[0].float_type=PID_V.p;
        flash_union_buffer[1].float_type=PID_V.i;    
        flash_union_buffer[2].float_type=PID_V.d;
        flash_union_buffer[3].float_type=PID_V.i_max;
        flash_union_buffer[4].float_type=PID_V.d_max;    
        flash_union_buffer[5].float_type=PID_V.output_max;
        
        flash_erase_page(100,0);
        flash_write_page_from_buffer(100,0);        // 向指定 Flash 扇区的页码写入缓冲区数据

        //100,1储存图象处理的数据
        
        
        if(flash_check(100, 1)){flash_erase_page(100, 1);}
        flash_buffer_clear();

        flash_union_buffer[0].float_type=S_PID.p;
        flash_union_buffer[1].float_type=S_PID.i;    
        flash_union_buffer[2].float_type=S_PID.d;
        flash_union_buffer[3].float_type=S_PID.outputmax;
        flash_union_buffer[4].float_type=S_PID.outputmin;
        
        flash_erase_page(100,1);
        flash_write_page_from_buffer(100,1);        // 向指定 Flash 扇区的页码写入缓冲区数据

        if(flash_check(100, 2)){flash_erase_page(100, 2);}
        flash_buffer_clear();

        flash_union_buffer[0].int32_type=speed;
        flash_union_buffer[1].int32_type=forwardsight;
        
        flash_erase_page(100,2);
        flash_write_page_from_buffer(100,2);        // 向指定 Flash 扇区的页码写入缓冲区数据

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
        flash_save();           //flash闪存
		BUZZ_cycle();           //蜂鸣器循环
        

    }
        
}





















// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
