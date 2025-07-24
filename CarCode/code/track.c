/*
 * track.c
 * 赛道识别与车辆状态控制模块
 * 创建日期：2023年10月24日
 * 作者：lychee
 */
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
//白列锁定
int16 bailie_lock_crossroad=MT9V03X_W/2;
int16 bailieright_lock_round=3*MT9V03X_W/4;
// 外部变量声明
extern int32 forwardsight;
extern int16 centerline[MT9V03X_H];      // 中心线数组（图像高度维度）
extern int16 leftline[MT9V03X_H];       // 左边界线数组 
extern int16 rightline[MT9V03X_H];      // 右边界线数组
extern int16 rightfollowline[MT9V03X_H]; // 右边界跟踪线
extern int16 leftfollowline[MT9V03X_H];  // 左边界跟踪线
extern uint8 pix_per_meter;             // 像素/米比例系数
int16 centerline2[MT9V03X_H];           // 二次计算的中心线

extern int16 leftlostpoint[2];   //左丢线数和左丢线点0为丢线数，1为丢线点
extern int16 rightlostpoint[2];  //右丢线数和左丢线点0为丢线数，1为丢线点
extern int16 bothlostpoint[2];   //同时丢线数和左丢线点0为丢线数，1为丢线

extern int16 search_stop; // 终止点

// 边界点检测结果
extern int16 Right_Down_Find;  // 右下边界点行号
extern int16 Left_Down_Find;   // 左下边界点行号
extern int16 Right_Up_Find;    // 右上边界点行号
extern int16 Left_Up_Find;     // 左上边界点行号

extern int16 right_down_guai;   // 右下拐点
extern int16 right_up_guai;     // 右上拐点 
extern int16 left_down_guai;    // 左下拐点
extern int16 left_up_guai;      // 左上拐点

// 圆环标志
extern int16 right_budandiao;       // 右不单调点
float right_dxbudandiao;            // 右不单调点斜率

extern uint8 leftline_num;         //左线点数量
extern uint8 rightline_num;        //右线点数量
int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight)            //如果终止点远于 前视距离
    {
        result=centerline2[forwardsight];   
        return result;                      
        
    }

    result=centerline2[search_stop];
    return centerline2[search_stop  ];
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


void element_check(void) {    
    // 更新左右跟踪线 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
    printf("carstatus,%d",carstatus_now);
    printf("search_stop:%d\n", search_stop);
    Find_Down_Point(MT9V03X_H-1, search_stop); // 查找下半段边界点
    Find_Up_Point(search_stop, MT9V03X_H-1);   // 查找上半段边界点
    printf("rightup%d,leftup%d\n", Right_Up_Find, Left_Up_Find);
    printf("rightdown%d,leftdown%d\n", Right_Down_Find, Left_Down_Find);


//    /*---------- 直道状态检测 ----------*/
    if(carstatus_now == straight) {
		//圆环↓↓↓↓↓↓↓
		//圆环↓↓↓↓↓↓↓ 
		//圆环↓↓↓↓↓↓↓
		//圆环↓↓↓↓↓↓↓
       if(continuity_left(10, MT9V03X_H-10)==0 &&continuity_right(10, MT9V03X_H-10)
           && Right_Down_Find!=0&&right_budandiao>10
           &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
       &&rightlostpoint[0]<70)  
       //左连续性，右连续性判断，右下角点找到，右不单调点找到，左线点数大于70，同时丢线数小于10，右丢线点数大于30右丢线点数小于70（可部分删去冗余条件）
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

//    /*---------- 十字路口状态处理 ----------*/
    if(carstatus_now == crossroad) {
        int start_down_point=5;
//        // 重新扫描边界突变点（从下往上）
        Find_Up_Point(5, MT9V03X_H-5);
        printf("rightup%d,leftup%d\n",Right_Up_Find,Left_Up_Find);
        
        Find_Down_Point(MT9V03X_H-4, start_down_point);
         printf("rightdown%d,leftdown%d\n",Right_Down_Find,Left_Down_Find);
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
        if(Right_Up_Find <= 5 || Left_Up_Find <= 5)//通过上位机检测 
            {
            carstatus_now = straight;
            return;
        }
        
    }

//    /*---------- 圆环预识别状态处理 ----------*/
    if(carstatus_now == round_1) {
        // 圆环预识别：检测右下拐点和右不单调点
        right_budandiao=montonicity_right(10, MT9V03X_H-10);
        ips200_show_int(0,300, right_budandiao, 3);
        if(Right_Down_Find != 0 && right_budandiao > 10) {
            right_dxbudandiao = (float)(rightline[right_budandiao] - rightline[right_down_guai]) / (right_budandiao - right_down_guai);
            draw_Rline_k(rightline[Right_Down_Find], Right_Down_Find, right_budandiao, right_dxbudandiao);
        }
//        if(Right_Down_Find==0&&Right_Up_Find>5&&right_budandiao>10) {
//            // 右下拐点未找到但右上拐点有效，直接进入圆环状态
//            carstatus_now = round_2;
//            return;
//        }

    }
    if(carstatus_now == round_2) {

    }
    
}


void choose_tracktype(void) 
{
    // 待实现功能：根据赛道特征选择跟踪策略
    // track_type = TRACK_LEFT;
}