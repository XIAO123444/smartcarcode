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

extern int16 search_stop; // ��ֹ��

// �߽������
extern int16 Right_Down_Find;  // ���±߽���к�
extern int16 Left_Down_Find;   // ���±߽���к�
extern int16 Right_Up_Find;    // ���ϱ߽���к�
extern int16 Left_Up_Find;     // ���ϱ߽���к�

extern int16 right_down_guai;   // ���¹յ�
extern int16 right_up_guai;     // ���Ϲյ� 
extern int16 left_down_guai;    // ���¹յ�
extern int16 left_up_guai;      // ���Ϲյ�

// Բ����־
extern int16 right_budandiao;       // �Ҳ�������
float right_dxbudandiao;            // �Ҳ�������б��

extern uint8 leftline_num;         //���ߵ�����
extern uint8 rightline_num;        //���ߵ�����
int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight)            //�����ֹ��Զ�� ǰ�Ӿ���
    {
        result=centerline2[forwardsight];   
        return result;                      
        
    }

    result=centerline2[search_stop];
    return centerline2[search_stop  ];
} 
 
int32 encodercounter=0;


enum mark {
    straight,    // ֱ����ʻ
    crossroad,   // ʮ��·��
    round_1,
    round_2,   // �뻷��ֱ��a
    round_3,   // Բ����б�ߣ�δʹ�ã�
    round_4,   // �뻷��ʻ
    round_5,   // ��յ㲹б��
};
enum mark carstatus_now = straight;  // ��ǰ����״̬


void centerline2_change(void) {
    for(int16 i=MT9V03X_H-1; i>search_stop; i--) {
                centerline2[i] = (rightfollowline[i] + leftfollowline[i]) / 2;

    }
}


void element_check(void) {    
    // �������Ҹ����� 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
    printf("carstatus,%d",carstatus_now);
    printf("search_stop:%d\n", search_stop);
    Find_Down_Point(MT9V03X_H-1, search_stop); // �����°�α߽��
    Find_Up_Point(search_stop, MT9V03X_H-1);   // �����ϰ�α߽��
    printf("rightup%d,leftup%d\n", Right_Up_Find, Left_Up_Find);
    printf("rightdown%d,leftdown%d\n", Right_Down_Find, Left_Down_Find);


//    /*---------- ֱ��״̬��� ----------*/
    if(carstatus_now == straight) {
		//Բ����������������
		//Բ���������������� 
		//Բ����������������
		//Բ����������������
       if(continuity_left(10, MT9V03X_H-10)==0 &&continuity_right(10, MT9V03X_H-10)
           && Right_Down_Find!=0&&right_budandiao>10
           &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
       &&rightlostpoint[0]<70)  
       //�������ԣ����������жϣ����½ǵ��ҵ����Ҳ��������ҵ������ߵ�������70��ͬʱ������С��10���Ҷ��ߵ�������30�Ҷ��ߵ���С��70���ɲ���ɾȥ����������
       {
           carstatus_now=round_1;
           return;
       }


        if(Left_Up_Find >= 10 && Right_Up_Find >= 10&&bothlostpoint[0]>20) {
        carstatus_now = crossroad;
        BUZZ_START();
            return; 
        }
        

   }

//    /*---------- ʮ��·��״̬���� ----------*/
    if(carstatus_now == crossroad) {
        int start_down_point=5;
//        // ����ɨ��߽�ͻ��㣨�������ϣ�
        Find_Up_Point(5, MT9V03X_H-5);
        printf("rightup%d,leftup%d\n",Right_Up_Find,Left_Up_Find);
        
        Find_Down_Point(MT9V03X_H-4, start_down_point);
         printf("rightdown%d,leftdown%d\n",Right_Down_Find,Left_Down_Find);
        //        // ȷ���°�α߽�㣨ȡ�����µ�
        if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
        if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

        /* �߽�����ϲ��� */
        if(Left_Down_Find != 0 && Right_Down_Find != 0) {
            // ���1�������µ����Ч �� ˫�߽�ֱ�����
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                       Right_Up_Find, rightline[Right_Up_Find]);        // �ұ߽����
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
                       Left_Up_Find, leftline[Left_Up_Find]);           // ��߽����
            printf("cross1");
        }
        else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
            // ���2�������µ���Ч �� �ұ߽����+��߽��ӳ�
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // �ұ߽����
                       Right_Up_Find, rightline[Right_Up_Find]);
            lenthen_Left_bondarise(Left_Up_Find);                       //
            printf("cross2");
        }
        else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
            // ���3�������µ���Ч �� ��߽����+�ұ߽��ӳ�
            lenthen_Right_bondarise(Right_Up_Find);
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                       Left_Up_Find, leftline[Left_Up_Find]);
            printf("cross3");
        }
        else {
            // ���4������Ч�µ� �� ˫�߽��ӳ�
            lenthen_Right_bondarise(Right_Up_Find);
            lenthen_Left_bondarise(Left_Up_Find);
            printf("cross4");
        }

        // �쳣����ͻ���ʧЧʱ�ָ�ԭʼ�߽�
        if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
        if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
        centerline2_change();

        // ͻ���ȫ��ʧЧʱ����ֱ��״̬
        if(Right_Up_Find <= 5 || Left_Up_Find <= 5)//ͨ����λ����� 
            {
            carstatus_now = straight;
            return;
        }
        
    }

//    /*---------- Բ��Ԥʶ��״̬���� ----------*/
    if(carstatus_now == round_1) {
        // Բ��Ԥʶ�𣺼�����¹յ���Ҳ�������
        right_budandiao=montonicity_right(10, MT9V03X_H-10);
        ips200_show_int(0,300, right_budandiao, 3);
        if(Right_Down_Find != 0 && right_budandiao > 10) {
            right_dxbudandiao = (float)(rightline[right_budandiao] - rightline[right_down_guai]) / (right_budandiao - right_down_guai);
            draw_Rline_k(rightline[Right_Down_Find], Right_Down_Find, right_budandiao, right_dxbudandiao);
        }
//        if(Right_Down_Find==0&&Right_Up_Find>5&&right_budandiao>10) {
//            // ���¹յ�δ�ҵ������Ϲյ���Ч��ֱ�ӽ���Բ��״̬
//            carstatus_now = round_2;
//            return;
//        }

    }
    if(carstatus_now == round_2) {

    }
    
}


void choose_tracktype(void) 
{
    // ��ʵ�ֹ��ܣ�������������ѡ����ٲ���
    // track_type = TRACK_LEFT;
}