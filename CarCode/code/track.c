/*
 * track.c
 * ����ʶ���복��״̬����ģ��
 * �������ڣ�2023��10��24��
 * ���ߣ�lychee
 */
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
//��������
int16 bailie_lock_crossroad=MT9V03X_W/2;
int16 bailieright_lock_round=3*MT9V03X_W/4;
// �ⲿ��������
extern int32 forwardsight;
extern int16 centerline[MT9V03X_H];      // ���������飨ͼ��߶�ά�ȣ�
extern int16 leftline[MT9V03X_H];       // ��߽������� 
extern int16 rightline[MT9V03X_H];      // �ұ߽�������
extern int16 rightfollowline[MT9V03X_H]; // �ұ߽������
extern int16 leftfollowline[MT9V03X_H];  // ��߽������
extern uint8 pix_per_meter;             // ����/�ױ���ϵ��
int16 centerline2[MT9V03X_H];           // ���μ����������

extern int16 leftlostpoint[2];   //�����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 rightlostpoint[2];  //�Ҷ����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 bothlostpoint[2];   //ͬʱ�����������ߵ�0Ϊ��������1Ϊ����

// �߽������
extern int16 Right_Down_Find;  // ���±߽���к�
extern int16 Left_Down_Find;   // ���±߽���к�
extern int16 Right_Up_Find;    // ���ϱ߽���к�
extern int16 Left_Up_Find;     // ���ϱ߽���к�

// Բ����־
extern int16 right_budandiao;
float right_dxbudandiao;

extern uint8 leftline_num;         //���ߵ�����
extern uint8 rightline_num;        //���ߵ�����
int16 output_middle2(void) {
    return centerline2[forwardsight  ];
} 
 
int32 encodercounter=0;


enum mark {
    straight,    // ֱ����ʻ
    crossroad,   // ʮ��·��
    round_2,   // �뻷��ֱ��
    round_3,   // Բ����б�ߣ�δʹ�ã�
    round_4,   // �뻷��ʻ
    round_5,   // ��յ㲹б��
    round_6    // ������ֱ��
};
enum mark carstatus_now = straight;  // ��ǰ����״̬


void centerline2_change(void) {
    for(int16 i=MT9V03X_H-5; i>1; i--) {
//        if(rightfollowline[i]==MT9V03X_W-1&&leftfollowline[i]==0)
//        {
//               centerline2[i] =(centerline2[i+1]+centerline2[i+2]+centerline2[i+3]+centerline2[i+4])/4;
//        }
//        else
//        {
//            centerline2[i] = (rightfollowline[i] + leftfollowline[i]) / 2;
//        }        
                centerline2[i] = (rightfollowline[i] + leftfollowline[i]) / 2;

    }
}


void element_check(void) {    
    ips200_show_int(0,280,carstatus_now,1);
    // �������Ҹ����� 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
//	printf("leftpoint%d",leftline_num);
//printf("rightpoint%d",rightline_num);
printf("leftpointlast%d",leftlostpoint[0]);
printf("rightpointlast%d",rightlostpoint[0]);
printf("bothpointlast%d",bothlostpoint[0]);  

    printf("carstatus%d\n",carstatus_now);
    /*---------- ֱ��״̬��� ----------*/
    if(carstatus_now == straight) {
        Find_Up_Point(MT9V03X_H-15, 30);             //���Ϲյ�
        Find_Down_Point(MT9V03X_H-15, 20);             //���Ϲյ�
		right_budandiao=montonicity_right(Right_Down_Find,5);
		//Բ����������������
		//Բ����������������
		//Բ����������������
		//Բ����������������
        if(continuity_left(5, MT9V03X_H-5)==0 &&continuity_right(5, MT9V03X_H-5)
            && Right_Down_Find!=0&&right_budandiao>10
            &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
        &&rightlostpoint[0]<70)
        {
        carstatus_now = round_2;
        BUZZ_START();
        return;
        }

		//ʮ��·�ڡ���������������
		//ʮ��·�ڡ���������������		
		//ʮ��·�ڡ���������������
        // ʮ��·�ڼ�����������ұ߽��������
        if(continuity_left(30, MT9V03X_H-5)  && 
           continuity_right(30, MT9V03X_H-5) > 0 ) 
        {
            
            // ���ұ߽�ͻ��㣨��������ɨ�裩
            
            // δ�ҵ�ͻ������˳�
            if(Left_Up_Find == 0 && Right_Up_Find == 0) return;
            
            // ͬʱ��⵽����ͻ������ж�Ϊʮ��·��
            if(Left_Up_Find != 0 && Right_Up_Find != 0) {
                carstatus_now = crossroad;
                BUZZ_START();
                return;
            }
        }
        

    }

    /*---------- ʮ��·��״̬���� ----------*/
    if(carstatus_now == crossroad) {
        int start_down_point=5;
//        // ����ɨ��߽�ͻ��㣨�������ϣ�
        Find_Up_Point(5, MT9V03X_H-5);
//        
//        // ȷ��ɨ����㣨ȡ����ͻ���Ľϸ�λ�ã�
        start_down_point = (Right_Up_Find < Left_Up_Find) ? Right_Up_Find : Left_Up_Find;
//        
//        // �����°�α߽��
        Find_Down_Point(MT9V03X_H-6, start_down_point);
//        
//        // У���µ�λ����Ч�ԣ���������ϵ�
//        printf("Right_Up_Find%d,Left_Up_Find%d,Right_Down_Find%d,Right_Down_Find%d\n",Right_Up_Find,Left_Up_Find,Right_Down_Find,Right_Down_Find);
        if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
        if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

        /* �߽�����ϲ��� */
        if(Left_Down_Find != 0 && Right_Down_Find != 0) {
            // ���1�������µ����Ч �� ˫�߽�ֱ�����
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                       Right_Up_Find, rightline[Right_Up_Find]);
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                       Left_Up_Find, leftline[Left_Up_Find]);
        }
        else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
            // ���2�������µ���Ч �� �ұ߽����+��߽��ӳ�
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                       Right_Up_Find, rightline[Right_Up_Find]);
            lenthen_Left_bondarise(Left_Up_Find);
        }
        else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
            // ���3�������µ���Ч �� ��߽����+�ұ߽��ӳ�
            lenthen_Right_bondarise(Right_Up_Find);
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                       Left_Up_Find, leftline[Left_Up_Find]);
        }
        else {
            // ���4������Ч�µ� �� ˫�߽��ӳ�
            lenthen_Right_bondarise(Right_Up_Find);
            lenthen_Left_bondarise(Left_Up_Find);
        }

        // �쳣����ͻ���ʧЧʱ�ָ�ԭʼ�߽�
        if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
        if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
        centerline2_change();

        // ͻ���ȫ��ʧЧʱ����ֱ��״̬
        if((Right_Up_Find == 0 && Left_Up_Find == 0)||(rightline_num>50||leftline_num>50)||(rightline_num<10||leftline_num<10)) {
            carstatus_now = straight;
            return;
        }
        
    }

    /*---------- Բ��Ԥʶ��״̬���� ----------*/
    if(carstatus_now == round_2) //��ֱ��
    {
		Find_Down_Point(MT9V03X_H-5, 10);
		Find_Up_Point(Right_Down_Find,5);
		right_budandiao=montonicity_right(Right_Down_Find,5);
        //��������
        if(Right_Down_Find>=75&&right_budandiao&&Right_Up_Find)//���½ǵ���ʧ���Ҳ���������ڣ����ϵ����
		{
			carstatus_now=round_3;
			return;
		}
		if(Right_Down_Find&right_budandiao)
		{
			add_Rline_k(rightline[Right_Down_Find],Right_Down_Find,right_budandiao,rightline[right_budandiao]);
		}
		centerline2_change();
    }
	if(carstatus_now==round_3)//���½ǵ���ʧ
	{
		Find_Up_Point(MT9V03X_H-5, 5);
		right_budandiao=montonicity_right(MT9V03X_H-5,5);
		
		printf("\n rightupfind%d",Right_Up_Find);
		printf("\n right_budandiao%d",right_budandiao);
        		if(right_budandiao==0)//����������ʧ��
		{
			carstatus_now=round_4;
            return;
		}
		if(Right_Up_Find&&right_budandiao)//���϶���Ͳ������㲹б��
		{
			add_Rline_k(rightline[right_budandiao],right_budandiao,MT9V03X_H-5,MT9V03X_W-1);
		}
		centerline2_change();

		
	} 
	if(carstatus_now==round_4)
	{
		Find_Up_Point(MT9V03X_H-5, 5);
        printf("LEFT_up_find%d",Left_Up_Find);
        printf("RIghT_up_find%d\n",Right_Up_Find);
        if(Left_Up_Find)
        {
            
        }
        centerline2_change();


	}


    
} 


void choose_tracktype(void) 
{
    // ��ʵ�ֹ��ܣ�������������ѡ����ٲ���
    // track_type = TRACK_LEFT;
}