/*
 * track.c
 * ����ʶ���복��״̬����ģ��
 * �������ڣ�2023��10��24��
 * ���ߣ�lychee
 */
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
#include "speed.h"
//��������
int16 bailie_lock_crossroad=MT9V03X_W/2;
int16 bailieright_lock_round=3*MT9V03X_W/4;
// �ⲿ��������
uint8 cross_flag=0;

uint8 island_flag = 0; // �����־
uint8 Island_State=0;
uint8 right_island_flag = 0; // �ҵ����־
uint8 left_island_flag = 0; // �����־
extern int32 forwardsight;
extern int16 centerline[MT9V03X_H];      // ���������飨ͼ��߶�ά�ȣ�
extern int16 leftline[MT9V03X_H];       // ��߽������� 
extern int16 rightline[MT9V03X_H];      // �ұ߽�������
extern int16 rightfollowline[MT9V03X_H]; // �ұ߽������
extern int16 leftfollowline[MT9V03X_H];  // ��߽������
extern uint8 pix_per_meter;             // ����/�ױ���ϵ��
int16 centerline2[MT9V03X_H];           // ���μ����������

extern int16 boundry_start_left; //��߽���ʼ��
extern int16 boundry_start_right; //�ұ߽���ʼ��

extern int16 leftlostpoint[2];   //�����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 rightlostpoint[2];  //�Ҷ����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 bothlostpoint[2];   //ͬʱ�����������ߵ�0Ϊ��������1Ϊ����

extern int16 search_stop; // ��ֹ��

// �߽������
extern int16 Right_Down_Find;  // ���±߽���к�
extern int16 Left_Down_Find;   // ���±߽���к�
extern int16 Right_Up_Find;    // ���ϱ߽���к�
extern int16 Left_Up_Find;     // ���ϱ߽���к�

extern int16 right_down_guai[2];   // ���¹յ�
extern int16 right_up_guai[2];     // ���Ϲյ� 
extern int16 left_down_guai[2];    // ���¹յ�
extern int16 left_up_guai[2];      // ���Ϲյ�

// Բ����־
extern int16 right_budandiao;       // �Ҳ�������
float right_dxbudandiao;            // �Ҳ�������б��

extern uint8 leftline_num;         //���ߵ�����
extern uint8 rightline_num;        //���ߵ�����

extern int32 forwardsight_stragety;

uint8 crossconfirm=0;

int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight_stragety)            //�����ֹ��Զ�� ǰ�Ӿ���
    {
        result=centerline2[forwardsight_stragety];   
        return result;                      
        
    }

    result=centerline2[search_stop];
    return centerline2[search_stop];
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
void island_check(void)
{   
    ips200_show_int(0,300,Island_State,2);
    static float k=0;//3��5״̬��k
    static int island_state_5_down[2]={0};//״̬5ʱ�����뿪���������ұ߽����͵㣬[0]��y����ĳ�У�{1}��x����ĳ��
    static int island_state_3_up[2]={0};//״̬3ʱ�������뻷���ã���������ǵ�[0]��y����ĳ�У�{1}��x����ĳ��
    static int left_down_guai[2]={0};//�ĸ��յ������洢��[0]��y����ĳ�У�{1}��x����ĳ��
    static int right_down_guai[2]={0};//�ĸ��յ������洢��[0]��y����ĳ�У�{1}��x����ĳ��
    int monotonicity_change_line[2];//�����Ըı�����꣬[0]��ĳ�У�[1]��ĳ��
    int monotonicity_change_left_flag=0;//��ת����0
    int monotonicity_change_right_flag=0;//��ת����0
    int continuity_change_right_flag=0; //������0
    int continuity_change_left_flag=0;  //������0

    continuity_change_left_flag=continuity_left(MT9V03X_H-1-5,10);//�������ж�
    continuity_change_right_flag=continuity_right(MT9V03X_H-1-5,10);
    monotonicity_change_right_flag=montonicity_right(MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=montonicity_left(MT9V03X_H-1-10,10);
    if(island_flag==0&&carstatus_now==straight)//&&Ramp_Flag==0
    {
        continuity_change_left_flag=continuity_left(MT9V03X_H-1-5,10);//�������ж�
        continuity_change_right_flag=continuity_right(MT9V03X_H-1-5,10);
 

            if(monotonicity_change_left_flag==0&&
               continuity_change_left_flag==0&& //�һ��������������
               continuity_change_right_flag!=1&& //�ұ��ǲ�������
               rightlostpoint[0]>=10&&           //�Ҷ��߶�
               rightlostpoint[0]<=50&&           //�Ҷ��߲���̫��
               leftlostpoint[0]<=10&&            //������
               search_stop>=MT9V03X_H*0.95&& //������ֹ�п�����Զ
            //    Boundry_Start_Left>=MT9V03X_H-20&&Boundry_Start_Right>=MT9V03X_H-20&& //�߽���ʼ�㿿��
               bothlostpoint[0]<=10)
            {
                right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);//���µ�
                if(right_down_guai[0]>=30)//����1���ɣ���������йյ㣬λ�ò��ԣ��������У�����
                {
                    carstatus_now==straight;
                    Island_State=1;
                    right_island_flag=1;
                }
                else
                {
                    carstatus_now==straight;
                    Island_State=0;
                    right_island_flag=0;
                }
            }
        
    }
    if(right_island_flag==1)
    {
        if(Island_State==1)//1״̬�¹յ㻹�ڣ�û����
        {
            monotonicity_change_line[0]=montonicity_right(30,5);//�����Ըı�        ��ֵ�ɸı�
            monotonicity_change_line[1]=rightline[monotonicity_change_line[0]];
            add_Rline_k((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[0],monotonicity_change_line[1]);
            if(boundry_start_right<=30)//���½��ȶ���       ��ֵ�ɸ�
            {
                Island_State=2;
            }
        }
        else if(Island_State==2)//2״̬�·����ߣ��Ϸ��������ִ���
        {
            monotonicity_change_line[0]=montonicity_right(70,5);//�����Ըı�            
            monotonicity_change_line[1]=rightline[monotonicity_change_line[0]];
            add_Rline_k((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[0],monotonicity_change_line[1]);
//            if(Island_State==2&&(Boundry_Start_Right>=MT9V03X_H-10))//���½��ٲ����߽�3
            if(Island_State==2&&(boundry_start_right>=MT9V03X_H-5||monotonicity_change_line[0]>50))//���½��ٲ����߽�3          /��ֵ�ɸ�
            {
                Island_State=3;//�·����ߣ�˵�������Ѿ�������
            }
            centerline2_change();

        }
        else if(Island_State==3)//�����Ѿ����ִ��ߣ����Ϸ����ֽǵ�
        {
            continuity_change_left_flag=continuity_left(5,MT9V03X_H-5);
            continuity_change_right_flag=continuity_right(5,MT9V03X_H-5);
            left_up_guai[0]= Find_Left_Up_Point(MT9V03X_H-5,5);         //�����Ͻǵ�
            left_up_guai[1]=leftline[left_up_guai[0]];                  
            right_up_guai[0]=Find_Right_Up_Point(MT9V03X_H-5,5);        //�����Ͻǵ�
            right_up_guai[1]=rightline[right_up_guai[0]];
            if(continuity_change_left_flag==0&&continuity_change_right_flag!=0&&right_up_guai[0]>10)        //������������Ҳ����������ҹյ��ҵ��������ҹյ㲹����
            {
                add_Lline_k(right_up_guai[1],right_up_guai[0],MT9V03X_H-10,leftline[MT9V03X_H-10]);

            }
            else if(continuity_change_left_flag!=0&&left_up_guai[0])                            //���������������յ��ҵ�������յ㲹����
            {
                add_Lline_k(left_up_guai[1],left_up_guai[0],MT9V03X_H-10,leftline[MT9V03X_H-10]);
            }
            else if(left_up_guai[0]==0&&right_up_guai[0]==0)//������ҹյ㶼���ˣ��Ǿ��ǽ��뻷���� 
            {
                Island_State=4;
            }
            centerline2_change();
        }
        else if(Island_State==4)
        {
            centerline2_change();

        }
            
    }


}
void cross_check(void)
{

    Find_Down_Point(MT9V03X_H-1, search_stop); // �����°�α߽��
    Find_Up_Point(search_stop, MT9V03X_H-1);   // �����ϰ�α߽��
    if(Left_Up_Find >= 10 && Right_Up_Find >= 10&&bothlostpoint[0]>10&&abs(Left_Up_Find-Right_Up_Find)<50&&
    leftline[Left_Up_Find]>MT9V03X_W/2+10&&
    leftline[Right_Up_Find<MT9V03X_W/2-10])       //������ϵ�����ϵ㶼��Ч��ͬʱ���ߵ����20
      {
           crossconfirm++;
           if(crossconfirm>3) {
               carstatus_now = crossroad; // ����ʮ��·��״̬
               crossconfirm = 0; // ����ȷ�ϼ���
               printf("crossroad\n");
           }
      }
      if(carstatus_now == crossroad) {
       int start_down_point=5;
       int16 temp1_L=0;//��¼��һ����յ�
       int16 temp1_R=0;//��¼��һ���ҹյ� 
//        // ����ɨ��߽�ͻ��㣨�������ϣ�
       Find_Up_Point(10, MT9V03X_H-5);
       temp1_L = Left_Up_Find+1; // ��¼���ϵ�
       temp1_R = Right_Up_Find+1; // ��¼���ϵ�   

           Left_Up_Find = temp1_L-3; // �ָ����ϵ�
           Right_Up_Find = temp1_R-3; // �ָ����ϵ�
       Find_Down_Point(MT9V03X_H-4, 20);

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
      if(Right_Up_Find >= MT9V03X_H-10 || Left_Up_Find >=MT9V03X_H-10||Right_Up_Find<10||Left_Up_Find<10)//ͨ����λ����� 
          {
          carstatus_now = straight;
          return;
      }
      
  }

}


void element_check(void) {    
    // �������Ҹ����� 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
    island_check();
    cross_check();
//    printf("carstatus,%d",carstatus_now);
//    printf("search_stop:%d\n", search_stop);


////    printf("rightup%d,leftup%d\n", Right_Up_Find, Left_Up_Find);
////    printf("rightdown%d,leftdown%d\n", Right_Down_Find, Left_Down_Find);


//////    /*---------- ֱ��״̬��� ----------*/
//   if(carstatus_now == straight) {
//		//Բ����������������
//		//Բ���������������� 
//		//Բ����������������
//		//Բ����������������
////      if(continuity_left(10, MT9V03X_H-10)==0 &&continuity_right(10, MT9V03X_H-10)
////          && Right_Down_Find!=0&&right_budandiao>10
////          &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
////      &&rightlostpoint[0]<70)  
////      //�������ԣ����������жϣ����½ǵ��ҵ����Ҳ��������ҵ������ߵ�������70��ͬʱ������С��10���Ҷ��ߵ�������30�Ҷ��ߵ���С��70���ɲ���ɾȥ����������
////      {
////          carstatus_now=round_1;
////          return;
////      }


//       
//       

//   }

////    /*---------- ʮ��·��״̬���� ----------*/
//   if(carstatus_now == crossroad) {
//        int start_down_point=5;
//        int16 temp1_L=0;//��¼��һ����յ�
//        int16 temp1_R=0;//��¼��һ���ҹյ� 
////        // ����ɨ��߽�ͻ��㣨�������ϣ�
//        Find_Up_Point(10, MT9V03X_H-5);
//        temp1_L = Left_Up_Find+1; // ��¼���ϵ�
//        temp1_R = Right_Up_Find+1; // ��¼���ϵ�   
////            
////        int16 start_second_start=(temp1_L> temp1_R )? temp1_L : temp1_R; // ȡ���ϵ�����ϵ�����ֵ��Ϊ�ڶ�����ʼ��
////        Find_Up_Point(start_second_start, MT9V03X_H-5);
////        if((Left_Up_Find||Right_Up_Find)&&Left_Up_Find>temp1_L && Right_Up_Find>temp1_R) 
////        {}
////        else
////        {
//            Left_Up_Find = temp1_L-3; // �ָ����ϵ�
//            Right_Up_Find = temp1_R-3; // �ָ����ϵ�
////        }
//        Find_Down_Point(MT9V03X_H-4, 20);

//       //        // ȷ���°�α߽�㣨ȡ�����µ�
//       if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
//       if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

//       /* �߽�����ϲ��� */
//       if(Left_Down_Find != 0 && Right_Down_Find != 0) {
//           // ���1�������µ����Ч �� ˫�߽�ֱ�����
//           add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
//                      Right_Up_Find, rightline[Right_Up_Find]);        // �ұ߽����
//           add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
//                      Left_Up_Find, leftline[Left_Up_Find]);           // ��߽����
//           printf("cross1");
//       }
//       else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
//           // ���2�������µ���Ч �� �ұ߽����+��߽��ӳ�
//           add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // �ұ߽����
//                      Right_Up_Find, rightline[Right_Up_Find]);
//           lenthen_Left_bondarise(Left_Up_Find);                       //
//           printf("cross2");
//       }
//       else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
//           // ���3�������µ���Ч �� ��߽����+�ұ߽��ӳ�
//           lenthen_Right_bondarise(Right_Up_Find);
//           add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
//                      Left_Up_Find, leftline[Left_Up_Find]);
//           printf("cross3");
//       }
//       else {
//           // ���4������Ч�µ� �� ˫�߽��ӳ�
//           lenthen_Right_bondarise(Right_Up_Find);
//           lenthen_Left_bondarise(Left_Up_Find);
//           printf("cross4");
//       }

//       // �쳣����ͻ���ʧЧʱ�ָ�ԭʼ�߽�
//       if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
//       if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
//       centerline2_change();

//       // ͻ���ȫ��ʧЧʱ����ֱ��״̬
//       if(Right_Up_Find >= MT9V03X_H-10 || Left_Up_Find >=MT9V03X_H-10||Right_Up_Find<10||Left_Up_Find<10)//ͨ����λ����� 
//           {
//           carstatus_now = straight;
//           return;
//       }
//       
//   }

////    /*---------- Բ��Ԥʶ��״̬���� ----------*/
//    if(carstatus_now == round_1) {
//        // Բ��Ԥʶ�𣺼�����¹յ���Ҳ�������
//        right_budandiao=montonicity_right(10, MT9V03X_H-10);
//        ips200_show_int(0,300, right_budandiao, 3);
//        if(Right_Down_Find != 0 && right_budandiao > 10) {
//            right_dxbudandiao = (float)(rightline[right_budandiao] - rightline[right_down_guai]) / (right_budandiao - right_down_guai);
//            draw_Rline_k(rightline[Right_Down_Find], Right_Down_Find, right_budandiao, right_dxbudandiao);
//        }
////        if(Right_Down_Find==0&&Right_Up_Find>5&&right_budandiao>10) {
////            // ���¹յ�δ�ҵ������Ϲյ���Ч��ֱ�ӽ���Բ��״̬
////            carstatus_now = round_2;
////            return;
////        }

//    }
//    if(carstatus_now == round_2) {

//    }
    
}


