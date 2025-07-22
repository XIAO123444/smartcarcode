#include "flash.h"
#include "pid_v.h"
#include "steer_pid.h"
#include "menu.h"

extern struct pid_v PID_V;
extern struct steer_pid
 S_PID;
extern int speed;
extern int forwardsight;

void flash_init(void)
{
    flash_buffer_clear();
////����дҪ���������
    flash_read_page_to_buffer(100, 0);                            // �����ݴ� flash ��ȡ��������
    PID_V.p                =flash_union_buffer[0].float_type;
    PID_V.i                =flash_union_buffer[1].float_type;
    PID_V.d                =flash_union_buffer[2].float_type;
    PID_V.i_max            =flash_union_buffer[3].float_type;
    PID_V.d_max            =flash_union_buffer[4].float_type;
    PID_V.output_max       =flash_union_buffer[5].float_type;
    flash_buffer_clear();
    flash_read_page_to_buffer(100, 1);                            // �����ݴ� flash ��ȡ��������
    S_PID.p                =flash_union_buffer[0].float_type;
    S_PID.i                =flash_union_buffer[1].float_type;
    S_PID.d                =flash_union_buffer[2].float_type;
    S_PID.outputmax        =flash_union_buffer[3].float_type;
    S_PID.outputmin        =flash_union_buffer[4].float_type;
    flash_buffer_clear();
    flash_read_page_to_buffer(100, 2);                            // �����ݴ� flash ��ȡ��������
    speed                  =flash_union_buffer[0].int32_type;
    forwardsight                  =flash_union_buffer[1].int32_type;

}
