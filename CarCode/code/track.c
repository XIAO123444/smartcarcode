/*
 * track.c
 * 赛道识别与车辆状态控制模块
 * 创建日期：2023年10月24日
 * 作者：lychee
 */
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
#include "speed.h"
//白列锁定
int16 bailie_lock_crossroad=MT9V03X_W/2;
int16 bailieright_lock_round=3*MT9V03X_W/4;
// 外部变量声明
uint8 cross_flag=0;

uint8 island_flag = 0; // 岛屿标志
uint8 Island_State=0;
uint8 right_island_flag = 0; // 右岛屿标志
uint8 left_island_flag = 0; // 左岛屿标志
extern int32 forwardsight;
extern int16 centerline[MT9V03X_H];      // 中心线数组（图像高度维度）
extern int16 leftline[MT9V03X_H];       // 左边界线数组 
extern int16 rightline[MT9V03X_H];      // 右边界线数组
extern int16 rightfollowline[MT9V03X_H]; // 右边界跟踪线
extern int16 leftfollowline[MT9V03X_H];  // 左边界跟踪线
extern uint8 pix_per_meter;             // 像素/米比例系数
int16 centerline2[MT9V03X_H];           // 二次计算的中心线

extern int16 boundry_start_left; //左边界起始点
extern int16 boundry_start_right; //右边界起始点

extern int16 leftlostpoint[2];   //左丢线数和左丢线点0为丢线数，1为丢线点
extern int16 rightlostpoint[2];  //右丢线数和左丢线点0为丢线数，1为丢线点
extern int16 bothlostpoint[2];   //同时丢线数和左丢线点0为丢线数，1为丢线

extern int16 search_stop; // 终止点

// 边界点检测结果
extern int16 Right_Down_Find;  // 右下边界点行号
extern int16 Left_Down_Find;   // 左下边界点行号
extern int16 Right_Up_Find;    // 右上边界点行号
extern int16 Left_Up_Find;     // 左上边界点行号

extern int16 right_down_guai[2];   // 右下拐点
extern int16 right_up_guai[2];     // 右上拐点 
extern int16 left_down_guai[2];    // 左下拐点
extern int16 left_up_guai[2];      // 左上拐点

// 圆环标志
extern int16 right_budandiao;       // 右不单调点
float right_dxbudandiao;            // 右不单调点斜率

extern uint8 leftline_num;         //左线点数量
extern uint8 rightline_num;        //右线点数量

extern int32 forwardsight_stragety;

uint8 crossconfirm=0;

int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight_stragety)            //如果终止点远于 前视距离
    {
        result=centerline2[forwardsight_stragety];   
        return result;                      
        
    }

    result=centerline2[search_stop];
    return centerline2[search_stop];
} 
 
int32 encodercounter=0;


enum mark {
    straight,    // 直道行驶
    crossroad,   // 十字路口
    round_1,
    round_2,   // 入环补直线a
    round_3,   // 圆环补斜线（未使用）
    round_4,   // 入环行驶
    round_5,   // 左拐点补斜线
};
enum mark carstatus_now = straight;  // 当前车辆状态


void centerline2_change(void) {
    for(int16 i=MT9V03X_H-1; i>search_stop; i--) {
                centerline2[i] = (rightfollowline[i] + leftfollowline[i]) / 2;

    }
}
void island_check(void)
{   
    ips200_show_int(0,300,Island_State,2);
    static float k=0;//3和5状态的k
    static int island_state_5_down[2]={0};//状态5时即将离开环岛，左右边界边最低点，[0]存y，第某行，{1}存x，第某列
    static int island_state_3_up[2]={0};//状态3时即将进入环岛用，左右上面角点[0]存y，第某行，{1}存x，第某列
    static int left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    static int right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    int monotonicity_change_line[2];//单调性改变点坐标，[0]存某行，[1]存某列
    int monotonicity_change_left_flag=0;//不转折是0
    int monotonicity_change_right_flag=0;//不转折是0
    int continuity_change_right_flag=0; //连续是0
    int continuity_change_left_flag=0;  //连续是0

    continuity_change_left_flag=continuity_left(MT9V03X_H-1-5,10);//连续性判断
    continuity_change_right_flag=continuity_right(MT9V03X_H-1-5,10);
    monotonicity_change_right_flag=montonicity_right(MT9V03X_H-1-10,10);
    monotonicity_change_left_flag=montonicity_left(MT9V03X_H-1-10,10);
    if(island_flag==0&&carstatus_now==straight)//&&Ramp_Flag==0
    {
        continuity_change_left_flag=continuity_left(MT9V03X_H-1-5,10);//连续性判断
        continuity_change_right_flag=continuity_right(MT9V03X_H-1-5,10);
 

            if(monotonicity_change_left_flag==0&&
               continuity_change_left_flag==0&& //右环岛左边是连续的
               continuity_change_right_flag!=1&& //右边是不连续的
               rightlostpoint[0]>=10&&           //右丢线多
               rightlostpoint[0]<=50&&           //右丢线不能太多
               leftlostpoint[0]<=10&&            //左丢线少
               search_stop>=MT9V03X_H*0.95&& //搜索截止行看到很远
            //    Boundry_Start_Left>=MT9V03X_H-20&&Boundry_Start_Right>=MT9V03X_H-20&& //边界起始点靠下
               bothlostpoint[0]<=10)
            {
                right_down_guai[0]=Find_Right_Down_Point(MT9V03X_H-1,20);//右下点
                if(right_down_guai[0]>=30)//条件1很松，在这里加判拐点，位置不对，则是误判，跳出
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
        if(Island_State==1)//1状态下拐点还在，没丢线
        {
            monotonicity_change_line[0]=montonicity_right(30,5);//单调性改变        阈值可改变
            monotonicity_change_line[1]=rightline[monotonicity_change_line[0]];
            add_Rline_k((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[0],monotonicity_change_line[1]);
            if(boundry_start_right<=30)//右下角先丢线       阈值可改
            {
                Island_State=2;
            }
        }
        else if(Island_State==2)//2状态下方丢线，上方即将出现大弧线
        {
            monotonicity_change_line[0]=montonicity_right(70,5);//单调性改变            
            monotonicity_change_line[1]=rightline[monotonicity_change_line[0]];
            add_Rline_k((int)(MT9V03X_W-1-(monotonicity_change_line[1]*0.15)),MT9V03X_H-1,monotonicity_change_line[0],monotonicity_change_line[1]);
//            if(Island_State==2&&(Boundry_Start_Right>=MT9V03X_H-10))//右下角再不丢线进3
            if(Island_State==2&&(boundry_start_right>=MT9V03X_H-5||monotonicity_change_line[0]>50))//右下角再不丢线进3          /阈值可改
            {
                Island_State=3;//下方丢线，说明大弧线已经下来了
            }
            centerline2_change();

        }
        else if(Island_State==3)//下面已经出现大弧线，且上方出现角点
        {
            continuity_change_left_flag=continuity_left(5,MT9V03X_H-5);
            continuity_change_right_flag=continuity_right(5,MT9V03X_H-5);
            left_up_guai[0]= Find_Left_Up_Point(MT9V03X_H-5,5);         //找左上角点
            left_up_guai[1]=leftline[left_up_guai[0]];                  
            right_up_guai[0]=Find_Right_Up_Point(MT9V03X_H-5,5);        //找右上角点
            right_up_guai[1]=rightline[right_up_guai[0]];
            if(continuity_change_left_flag==0&&continuity_change_right_flag!=0&&right_up_guai[0]>10)        //如果左连续，右不连续，且右拐点找到，就用右拐点补左线
            {
                add_Lline_k(right_up_guai[1],right_up_guai[0],MT9V03X_H-10,leftline[MT9V03X_H-10]);

            }
            else if(continuity_change_left_flag!=0&&left_up_guai[0])                            //如果左不连续，且左拐点找到就用左拐点补左线
            {
                add_Lline_k(left_up_guai[1],left_up_guai[0],MT9V03X_H-10,leftline[MT9V03X_H-10]);
            }
            else if(left_up_guai[0]==0&&right_up_guai[0]==0)//如果左右拐点都丢了，那就是进入环道了 
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

    Find_Down_Point(MT9V03X_H-1, search_stop); // 查找下半段边界点
    Find_Up_Point(search_stop, MT9V03X_H-1);   // 查找上半段边界点
    if(Left_Up_Find >= 10 && Right_Up_Find >= 10&&bothlostpoint[0]>10&&abs(Left_Up_Find-Right_Up_Find)<50&&
    leftline[Left_Up_Find]>MT9V03X_W/2+10&&
    leftline[Right_Up_Find<MT9V03X_W/2-10])       //如果左上点和右上点都有效且同时丢线点大于20
      {
           crossconfirm++;
           if(crossconfirm>3) {
               carstatus_now = crossroad; // 进入十字路口状态
               crossconfirm = 0; // 重置确认计数
               printf("crossroad\n");
           }
      }
      if(carstatus_now == crossroad) {
       int start_down_point=5;
       int16 temp1_L=0;//记录第一个左拐点
       int16 temp1_R=0;//记录第一个右拐点 
//        // 重新扫描边界突变点（从下往上）
       Find_Up_Point(10, MT9V03X_H-5);
       temp1_L = Left_Up_Find+1; // 记录左上点
       temp1_R = Right_Up_Find+1; // 记录右上点   

           Left_Up_Find = temp1_L-3; // 恢复左上点
           Right_Up_Find = temp1_R-3; // 恢复右上点
       Find_Down_Point(MT9V03X_H-4, 20);

      //        // 确定下半段边界点（取左右下点
      if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
      if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

      /* 边界线拟合策略 */
      if(Left_Down_Find != 0 && Right_Down_Find != 0) {
          // 情况1：左右下点均有效 → 双边界直线拟合
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                     Right_Up_Find, rightline[Right_Up_Find]);        // 右边界拟合
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
                     Left_Up_Find, leftline[Left_Up_Find]);           // 左边界拟合
          printf("cross1");
      }
      else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
          // 情况2：仅右下点有效 → 右边界拟合+左边界延长
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // 右边界拟合
                     Right_Up_Find, rightline[Right_Up_Find]);
          lenthen_Left_bondarise(Left_Up_Find);                       //
          printf("cross2");
      }
      else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
          // 情况3：仅左下点有效 → 左边界拟合+右边界延长
          lenthen_Right_bondarise(Right_Up_Find);
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                     Left_Up_Find, leftline[Left_Up_Find]);
          printf("cross3");
      }
      else {
          // 情况4：无有效下点 → 双边界延长
          lenthen_Right_bondarise(Right_Up_Find);
          lenthen_Left_bondarise(Left_Up_Find);
          printf("cross4");
      }

      // 异常处理：突变点失效时恢复原始边界
      if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
      if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
      centerline2_change();

      // 突变点全部失效时返回直道状态
      if(Right_Up_Find >= MT9V03X_H-10 || Left_Up_Find >=MT9V03X_H-10||Right_Up_Find<10||Left_Up_Find<10)//通过上位机检测 
          {
          carstatus_now = straight;
          return;
      }
      
  }

}


void element_check(void) {    
    // 更新左右跟踪线 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
    island_check();
    cross_check();
//    printf("carstatus,%d",carstatus_now);
//    printf("search_stop:%d\n", search_stop);


////    printf("rightup%d,leftup%d\n", Right_Up_Find, Left_Up_Find);
////    printf("rightdown%d,leftdown%d\n", Right_Down_Find, Left_Down_Find);


//////    /*---------- 直道状态检测 ----------*/
//   if(carstatus_now == straight) {
//		//圆环↓↓↓↓↓↓↓
//		//圆环↓↓↓↓↓↓↓ 
//		//圆环↓↓↓↓↓↓↓
//		//圆环↓↓↓↓↓↓↓
////      if(continuity_left(10, MT9V03X_H-10)==0 &&continuity_right(10, MT9V03X_H-10)
////          && Right_Down_Find!=0&&right_budandiao>10
////          &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
////      &&rightlostpoint[0]<70)  
////      //左连续性，右连续性判断，右下角点找到，右不单调点找到，左线点数大于70，同时丢线数小于10，右丢线点数大于30右丢线点数小于70（可部分删去冗余条件）
////      {
////          carstatus_now=round_1;
////          return;
////      }


//       
//       

//   }

////    /*---------- 十字路口状态处理 ----------*/
//   if(carstatus_now == crossroad) {
//        int start_down_point=5;
//        int16 temp1_L=0;//记录第一个左拐点
//        int16 temp1_R=0;//记录第一个右拐点 
////        // 重新扫描边界突变点（从下往上）
//        Find_Up_Point(10, MT9V03X_H-5);
//        temp1_L = Left_Up_Find+1; // 记录左上点
//        temp1_R = Right_Up_Find+1; // 记录右上点   
////            
////        int16 start_second_start=(temp1_L> temp1_R )? temp1_L : temp1_R; // 取左上点和右上点的最大值作为第二段起始点
////        Find_Up_Point(start_second_start, MT9V03X_H-5);
////        if((Left_Up_Find||Right_Up_Find)&&Left_Up_Find>temp1_L && Right_Up_Find>temp1_R) 
////        {}
////        else
////        {
//            Left_Up_Find = temp1_L-3; // 恢复左上点
//            Right_Up_Find = temp1_R-3; // 恢复右上点
////        }
//        Find_Down_Point(MT9V03X_H-4, 20);

//       //        // 确定下半段边界点（取左右下点
//       if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
//       if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

//       /* 边界线拟合策略 */
//       if(Left_Down_Find != 0 && Right_Down_Find != 0) {
//           // 情况1：左右下点均有效 → 双边界直线拟合
//           add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
//                      Right_Up_Find, rightline[Right_Up_Find]);        // 右边界拟合
//           add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
//                      Left_Up_Find, leftline[Left_Up_Find]);           // 左边界拟合
//           printf("cross1");
//       }
//       else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
//           // 情况2：仅右下点有效 → 右边界拟合+左边界延长
//           add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // 右边界拟合
//                      Right_Up_Find, rightline[Right_Up_Find]);
//           lenthen_Left_bondarise(Left_Up_Find);                       //
//           printf("cross2");
//       }
//       else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
//           // 情况3：仅左下点有效 → 左边界拟合+右边界延长
//           lenthen_Right_bondarise(Right_Up_Find);
//           add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
//                      Left_Up_Find, leftline[Left_Up_Find]);
//           printf("cross3");
//       }
//       else {
//           // 情况4：无有效下点 → 双边界延长
//           lenthen_Right_bondarise(Right_Up_Find);
//           lenthen_Left_bondarise(Left_Up_Find);
//           printf("cross4");
//       }

//       // 异常处理：突变点失效时恢复原始边界
//       if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
//       if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
//       centerline2_change();

//       // 突变点全部失效时返回直道状态
//       if(Right_Up_Find >= MT9V03X_H-10 || Left_Up_Find >=MT9V03X_H-10||Right_Up_Find<10||Left_Up_Find<10)//通过上位机检测 
//           {
//           carstatus_now = straight;
//           return;
//       }
//       
//   }

////    /*---------- 圆环预识别状态处理 ----------*/
//    if(carstatus_now == round_1) {
//        // 圆环预识别：检测右下拐点和右不单调点
//        right_budandiao=montonicity_right(10, MT9V03X_H-10);
//        ips200_show_int(0,300, right_budandiao, 3);
//        if(Right_Down_Find != 0 && right_budandiao > 10) {
//            right_dxbudandiao = (float)(rightline[right_budandiao] - rightline[right_down_guai]) / (right_budandiao - right_down_guai);
//            draw_Rline_k(rightline[Right_Down_Find], Right_Down_Find, right_budandiao, right_dxbudandiao);
//        }
////        if(Right_Down_Find==0&&Right_Up_Find>5&&right_budandiao>10) {
////            // 右下拐点未找到但右上拐点有效，直接进入圆环状态
////            carstatus_now = round_2;
////            return;
////        }

//    }
//    if(carstatus_now == round_2) {

//    }
    
}


