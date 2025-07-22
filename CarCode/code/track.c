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

// 边界点检测结果
extern int16 Right_Down_Find;  // 右下边界点行号
extern int16 Left_Down_Find;   // 左下边界点行号
extern int16 Right_Up_Find;    // 右上边界点行号
extern int16 Left_Up_Find;     // 左上边界点行号

// 圆环标志
extern int16 right_budandiao;
float right_dxbudandiao;

extern uint8 leftline_num;         //左线点数量
extern uint8 rightline_num;        //右线点数量
int16 output_middle2(void) {
    return centerline2[forwardsight  ];
} 
 
int32 encodercounter=0;


enum mark {
    straight,    // 直道行驶
    crossroad,   // 十字路口
    round_2,   // 入环补直线
    round_3,   // 圆环补斜线（未使用）
    round_4,   // 入环行驶
    round_5,   // 左拐点补斜线
    round_6    // 出环补直线
};
enum mark carstatus_now = straight;  // 当前车辆状态


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
    // 更新左右跟踪线 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));
    centerline2_change();
//	printf("leftpoint%d",leftline_num);
//printf("rightpoint%d",rightline_num);
printf("leftpointlast%d",leftlostpoint[0]);
printf("rightpointlast%d",rightlostpoint[0]);
printf("bothpointlast%d",bothlostpoint[0]);  

    printf("carstatus%d\n",carstatus_now);
    /*---------- 直道状态检测 ----------*/
    if(carstatus_now == straight) {
        Find_Up_Point(MT9V03X_H-15, 30);             //找上拐点
        Find_Down_Point(MT9V03X_H-15, 20);             //找上拐点
		right_budandiao=montonicity_right(Right_Down_Find,5);
		//圆环↓↓↓↓↓↓↓
		//圆环↓↓↓↓↓↓↓
		//圆环↓↓↓↓↓↓↓
		//圆环↓↓↓↓↓↓↓
        if(continuity_left(5, MT9V03X_H-5)==0 &&continuity_right(5, MT9V03X_H-5)
            && Right_Down_Find!=0&&right_budandiao>10
            &&leftline_num>70&&bothlostpoint[0]<10&&rightlostpoint[0]>30
        &&rightlostpoint[0]<70)
        {
        carstatus_now = round_2;
        BUZZ_START();
        return;
        }

		//十字路口↓↓↓↓↓↓↓↓
		//十字路口↓↓↓↓↓↓↓↓		
		//十字路口↓↓↓↓↓↓↓↓
        // 十字路口检测条件：左右边界均不连续
        if(continuity_left(30, MT9V03X_H-5)  && 
           continuity_right(30, MT9V03X_H-5) > 0 ) 
        {
            
            // 查找边界突变点（从上往下扫描）
            
            // 未找到突变点则退出
            if(Left_Up_Find == 0 && Right_Up_Find == 0) return;
            
            // 同时检测到左右突变点则判定为十字路口
            if(Left_Up_Find != 0 && Right_Up_Find != 0) {
                carstatus_now = crossroad;
                BUZZ_START();
                return;
            }
        }
        

    }

    /*---------- 十字路口状态处理 ----------*/
    if(carstatus_now == crossroad) {
        int start_down_point=5;
//        // 重新扫描边界突变点（从下往上）
        Find_Up_Point(5, MT9V03X_H-5);
//        
//        // 确定扫描起点（取左右突变点的较高位置）
        start_down_point = (Right_Up_Find < Left_Up_Find) ? Right_Up_Find : Left_Up_Find;
//        
//        // 查找下半段边界点
        Find_Down_Point(MT9V03X_H-6, start_down_point);
//        
//        // 校验下点位置有效性（必须低于上点
//        printf("Right_Up_Find%d,Left_Up_Find%d,Right_Down_Find%d,Right_Down_Find%d\n",Right_Up_Find,Left_Up_Find,Right_Down_Find,Right_Down_Find);
        if(Left_Down_Find <= Left_Up_Find) Left_Down_Find = 0;
        if(Right_Down_Find <= Right_Up_Find) Right_Down_Find = 0;

        /* 边界线拟合策略 */
        if(Left_Down_Find != 0 && Right_Down_Find != 0) {
            // 情况1：左右下点均有效 → 双边界直线拟合
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                       Right_Up_Find, rightline[Right_Up_Find]);
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                       Left_Up_Find, leftline[Left_Up_Find]);
        }
        else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
            // 情况2：仅右下点有效 → 右边界拟合+左边界延长
            add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                       Right_Up_Find, rightline[Right_Up_Find]);
            lenthen_Left_bondarise(Left_Up_Find);
        }
        else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
            // 情况3：仅左下点有效 → 左边界拟合+右边界延长
            lenthen_Right_bondarise(Right_Up_Find);
            add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                       Left_Up_Find, leftline[Left_Up_Find]);
        }
        else {
            // 情况4：无有效下点 → 双边界延长
            lenthen_Right_bondarise(Right_Up_Find);
            lenthen_Left_bondarise(Left_Up_Find);
        }

        // 异常处理：突变点失效时恢复原始边界
        if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
        if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
        centerline2_change();

        // 突变点全部失效时返回直道状态
        if((Right_Up_Find == 0 && Left_Up_Find == 0)||(rightline_num>50||leftline_num>50)||(rightline_num<10||leftline_num<10)) {
            carstatus_now = straight;
            return;
        }
        
    }

    /*---------- 圆环预识别状态处理 ----------*/
    if(carstatus_now == round_2) //补直线
    {
		Find_Down_Point(MT9V03X_H-5, 10);
		Find_Up_Point(Right_Down_Find,5);
		right_budandiao=montonicity_right(Right_Down_Find,5);
        //编码器法
        if(Right_Down_Find>=75&&right_budandiao&&Right_Up_Find)//右下角点消失，右不单调点存在，右上点存在
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
	if(carstatus_now==round_3)//右下角点消失
	{
		Find_Up_Point(MT9V03X_H-5, 5);
		right_budandiao=montonicity_right(MT9V03X_H-5,5);
		
		printf("\n rightupfind%d",Right_Up_Find);
		printf("\n right_budandiao%d",right_budandiao);
        		if(right_budandiao==0)//不单调点消失了
		{
			carstatus_now=round_4;
            return;
		}
		if(Right_Up_Find&&right_budandiao)//右上顶点和不单调点补斜率
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
    // 待实现功能：根据赛道特征选择跟踪策略
    // track_type = TRACK_LEFT;
}