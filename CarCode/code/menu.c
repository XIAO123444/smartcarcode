 #include "menu.h"
#include "pid_v.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"



#define ips200_x_max 240
#define ips200_y_max 320
bool start_flag=false;
int current_state=1;
int p=0;//��¼��ǰָ��
int p_nearby=0;//��¼������ָ��
int input;
extern int status;
extern uint16 w_step,h_step,K,limit;
extern bool save_flag;
int32 speed;
int32 forwardsight;


typedef struct 
{
    unsigned char priority;             //ҳ�����ȼ�
    char str[20];                       //����
    uint16 x;                           //��ʾ������
    uint16 y;                           //��ʾ������
    float value_f;                      //��������
    int value_i;                        //��������
    char int_float;                     //�������ͱ�־λ
    void (*Operate_DOWN)();             //������
    void (*Operate_UP)();               //������
    void (*Operate_default)();          //Ĭ��ִ�к���

}MENU;

void nfunc(void){
    ips200_show_string(0,180,"nofunc");

}
void start_car(void)
{
    start_flag=true;
}
void addspeed()
{
    speed+=5;
}
void subspeed()
{
    speed-=10;
}
void addforwardsight()
{
    forwardsight+=1;
}
void subforwardsight()
{
    forwardsight-=1;
    if(forwardsight<0)
    {
        forwardsight=0;
    }
}
void car_init(void)
{
    speed=0;
    forwardsight=50;
}

MENU menu[]={
    {1,"pid_param",0,20,0,0,0,nfunc,nfunc,nfunc},
        {2,"p",      ips200_x_max-10 * 7, 20,  0,0,1,  pid_sub_p,           pid_add_p,          nfunc},  
        {2,"i",      ips200_x_max-10 * 7, 40,  0,0,1,  pid_sub_i,           pid_add_i,          nfunc},  
        {2,"d",      ips200_x_max-10 * 7, 60,  0,0,1,  pid_sub_d,           pid_add_d,          nfunc},  
//        {2,"i_max",  ips200_x_max-10 * 7, 80,  0,0,1,  pid_sub_i_max,       pid_add_i_max,      nfunc},  
//        {2,"d_max",  ips200_x_max-10 * 7, 100, 0,0,1,  pid_sub_d_max,       pid_add_d_max,      nfunc},  
//        {2,"output", ips200_x_max-10 * 7, 120, 0,0,1,  pid_sub_output_max,  pid_add_output_max, nfunc},

        {2,"reset",  ips200_x_max-10 * 7, 140, 0,0,1,  pid_vparam_init, nfunc , nfunc},
        {2,"encoder_right"  ,ips200_x_max-10*7,160,0,0,0,                nfunc,nfunc,nfunc},
        {2,"encoder_left"   ,ips200_x_max-10*7,180,0,0,0,                nfunc,nfunc,nfunc},
    {1,"pid_s_param",0,40,0,0,0,nfunc,nfunc,nfunc},
        {2,"p_S",         ips200_x_max-10 * 7, 20,  0,0,1,  S_PIDsub_p,           S_PIDadd_p,          nfunc},  
        {2,"i_S",         ips200_x_max-10 * 7, 40,  0,0,1,  S_PIDsub_i,           S_PIDadd_i,          nfunc},  
        {2,"d_S",         ips200_x_max-10 * 7, 60,  0,0,1,  S_PIDsub_d,           S_PIDadd_d,          nfunc},  
        {2,"outputmax", ips200_x_max-10 * 7, 80,  0,0,0,    S_PIDsub_outputmax,  S_PIDadd_outputmax,  nfunc},
        {2,"outputmin", ips200_x_max-10 * 7, 100, 0,0,0,    nfunc             ,  nfunc             ,  nfunc},
        {2,"reset_S",     ips200_x_max-10 * 7, 120, 0,0,1,  PID_init, nfunc , nfunc},
    {1,"carstatue",0,60,0,0,0,nfunc,nfunc,nfunc},
        {2,"v_left"         ,ips200_x_max-10*7,20,0,0,1,                nfunc,nfunc,nfunc},
        {2,"v_right"        ,ips200_x_max -10*7,40,0,0,1,                nfunc,nfunc,nfunc},
        {2,"encoder_right"  ,ips200_x_max-10*7,60,0,0,0,                nfunc,nfunc,nfunc},
        {2,"encoder_left"   ,ips200_x_max-10*7,80,0,0,0,                start_car,nfunc,nfunc},
        {2,"speed",          ips200_x_max-10 * 7 ,100 ,0,0,0, subspeed,           addspeed,          nfunc },
        {2,"forwardsight",   ips200_x_max-10 * 7 ,120 ,0,0,0, subforwardsight,           addforwardsight,          nfunc },
        {2,"reset_C",     ips200_x_max-10 * 7, 140, 0,0,1,  car_init, nfunc , nfunc},


    {1,"START_THECAR",0,80,0,0,0,Encoder_Init,nfunc,nfunc},


    {1,"end",0,0,0,0,0}//����ɾȥ
};

enum condition{
    NOACTION,
    L,
    R,
    UP,
    DOWN,
    CONFIRM,
    BACK

}condition;  
    




/*
------------------------------------------------------------------------------------------------------------------
�������    ��ʼ����Ļ 
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_Screen_Init(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ�׵׺���
    ips200_init(IPS200_TYPE_SPI);    //����ͨ��ģʽΪSPIͨ��
}
/*
------------------------------------------------------------------------------------------------------------------
�������     �������ݣ���pid����������
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     �Ѽ������ұ����������ݣ�ͼ���������
-------------------------------------------------------------------------------------------------------------------
*/
void update(void)
{
    for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(strcmp(menu[i].str, "encoder_right")==0)
            {
                menu[i].value_i=Encoder_GetInfo_R();
            }
            if(strcmp(menu[i].str, "encoder_left")==0)
            {
                menu[i].value_i=Encoder_GetInfo_L();
            }
            struct pid_v *pid_ptr = PID_vget_param();
            struct steer_pid *pid_ptr1 = SPID_vget_param();

            
            //PID����
            if(strcmp(menu[i].str, "p")==0)
            {
                menu[i].value_f=pid_ptr->p;
            }
            if(strcmp(menu[i].str, "i")==0)
            {
                menu[i].value_f=pid_ptr->i;
            }
            if(strcmp(menu[i].str, "d")==0)
            {
                menu[i].value_f=pid_ptr->d;
            }           
            if(strcmp(menu[i].str, "i_max")==0)
            {
                menu[i].value_f=pid_ptr->i_max;
            }
            if(strcmp(menu[i].str, "d_max")==0)
            {
                menu[i].value_f=pid_ptr->d_max;
            }
            if(strcmp(menu[i].str, "output")==0)
            {
                menu[i].value_f=pid_ptr->output_max;
            }
            if(strcmp(menu[i].str, "p_S")==0)
            {
                menu[i].value_f=pid_ptr1->p;
            }
            if(strcmp(menu[i].str, "i_S")==0)
            {
                menu[i].value_f=pid_ptr1->i;
            }
            if(strcmp(menu[i].str, "d_S")==0)
            {
                menu[i].value_f=pid_ptr1->d;
            }           
            if(strcmp(menu[i].str, "outputmax")==0)
            {
                menu[i].value_i=pid_ptr1->outputmax;

            }
            if(strcmp(menu[i].str, "outputmin")==0)
            {
                menu[i].value_i=-pid_ptr1->outputmax;

            }
            
            if(strcmp(menu[i].str, "speed")==0)
            {
                menu[i].value_i=speed;

            }
           if(strcmp(menu[i].str, "forwardsight")==0)
            {
                menu[i].value_i=forwardsight;

            }

            
            //ͼ����

            //ͼ����ʾ
        }
}


/*
------------------------------------------------------------------------------------------------------------------
�������     ��Ļ��ʾ
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void output(void) 
{
    update();
    int target_priority=current_state-1;
    if (target_priority==0)
    {
    ips200_show_string(0,0,"menu");//��������ַ�
        int count=1;
        for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(menu[i].priority==1)
            {
                if(i==p)
                {
                    ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
                count++;
            }
        }
    }
    else if(target_priority!=0)
    {
     ips200_show_string(0,0,menu[p_nearby].str);//����ϼ��ַ�
        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].int_float)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,menu[i].value_f,3,3);
                    }
                    else
                    {
                       ips200_show_int(menu[i].x,menu[i].y,menu[i].value_i,5);
                    }
                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].int_float)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,menu[i].value_f,3,3);
                    }
                    else
                    {
                        ips200_show_int(menu[i].x,menu[i].y,menu[i].value_i,5);
                    }
                }
            } 
        }
    }

}

/*
------------------------------------------------------------------------------------------------------------------
�������     �˵�����
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_control(void)
{
        output();
//      scanf("%d", &input);�ȴ��޸�
        status=0;
        condition = (enum condition)input; 
        if(input)
        {
            ips200_clear();
        }
        switch (condition)
        {
            case NOACTION:
                break;
            
        case R:
            if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                p++;
                while(menu[p].priority!=temp){p++;} 
            }
            else
            {
                ips200_show_string(0,180,"error");
            }
            break;
        case L:
            if(p!=0&&menu[p-1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                p--;
                while (menu[p].priority!=temp){p--;}
            }
            else
            {
                ips200_show_string(0,180,"error");
            }
             

            break;
        case UP:
            menu[p].Operate_UP();
            break;
        case DOWN:
            menu[p].Operate_DOWN();
            break;
        case CONFIRM:
            if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0)
            {
                current_state++;
                p_nearby=p;
                p++;
            }
            else
            {
                ips200_show_string(0,180,"error");
            }
         

            break;
        case BACK:
        if(menu[p].priority!=1)
        {
            save_flag=true;
            current_state--;
            p=p_nearby;
            while (menu[p_nearby].priority!=current_state-1)
            {
                p_nearby--;
            }
        }
        else
        {
            ips200_show_string(0,180,"error");
        }

        default:
            break;
        }
        input=0;
        
}