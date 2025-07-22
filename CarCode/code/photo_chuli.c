/*
 * camera.c
 *
 *  Created on: 2023年10月24日
 *      Author: lychee
 */
#include "photo_chuli.h"
#include "math.h"
#include "track.h"
int16 centerline[MT9V03X_H];
int16 leftline[MT9V03X_H];
int16 rightline[MT9V03X_H];
int16 rightfollowline[MT9V03X_H];
int16 leftfollowline[MT9V03X_H];

int16 leftlostpoint[2]={0,0};   //左丢线数和左丢线点0为丢线数，1为丢线点
int16 rightlostpoint[2]={0,0};  //右丢线数和左丢线点0为丢线数，1为丢线点
int16 bothlostpoint[2]={0,0};   //同时丢线数和左丢线点0为丢线数，1为丢线点

uint16 left_lost_flag[MT9V03X_H];//左丢线数组
uint16 right_lost_flag[MT9V03X_H];//右丢线数组
uint16 both_lost_flag[MT9V03X_H];//同时丢线数组

//十字↓↓↓↓
int16 Right_Down_Find=0;    //右下点
int16 Left_Down_Find=0;     //左下点
int16 Right_Up_Find=0;      //右上点
int16 Left_Up_Find=0;       //左上点
//十字↑↑↑↑
//车状态
enum mark {
    straight,    // 直道行驶
    crossroad,   // 十字路口
    round_2,   // 入环补直线
    round_3,   // 圆环补斜线（未使用）
    round_4,   // 入环行驶
    round_5,   // 左拐点补斜线
    round_6    // 出环补直线
};
extern enum  mark carstatus_now;
//锁列
extern int16 bailie_lock_crossroad;
extern int16 bailieright_lock_round;

//线点与丢线↓↓↓↓
uint8 leftline_num;         //左线点数量
uint8 rightline_num;        //右线点数量


//线点与丢线↑↑↑↑

//圆环↓↓↓↓
int16 right_down_guai   =0;            //右下拐点
int16 right_up_guai     =0;            //右上拐点
int16 left_down_guai    =0;            //左下拐点
int16 left_up_guai      =0;            //左上拐点

int16 left_budandiao       =0;     //左不单调
int16 right_budandiao      =0;     //右不单调
//圆环↑↑↑↑



//差比和↓↓↓↓
int16 sar_thre = 17;//差比和阈值
//差比和↑↑↑↑
uint8 pix_per_meter = 20;//每米的像素数


extern bool stop_flag1;

float dx1[5]={0};
float dx2[5]={0};

int16 right_down_line =0;

//这里都是差比和↓↓↓↓↓↓↓
/*
------------------------------------------------------------------------------------------------------------------
函数简介     差比和寻找边界点
参数说明     无
返回参数     无
使用示例     直接调用
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/
void image_boundary_process(void){
    uint8 row;//行
    //uint8 col = MT9V03X_W/2;//列
    uint8 start_col = MT9V03X_W / 2;//各行起点的列坐标,默认为MT9V03X_W / 2
    //清零之前的计数
    leftline_num = 0;
    rightline_num = 0;

    for(row = MT9V03X_H - 1; row >= 1; row--){
        //选用上一行的中点作为下一行计算起始点，节省速度，同时防止弯道的左右两边均出现与画面一侧
        if(row != MT9V03X_H - 1){
            start_col = (uint8)(0.4 * centerline[row] + 0.3 * start_col + 0.1 * MT9V03X_W);//一阶低通滤波，防止出现噪点影响下一行的起始点


        }
        else if(row == MT9V03X_H - 1){
            start_col = MT9V03X_W / 2;
        }
        if(start_col<MT9V03X_W/2-30){start_col=MT9V03X_W/2-30;}
        if(start_col>MT9V03X_W/2+30){start_col=MT9V03X_W/2+30;}
        //逐行作差比和 
        difsum_left(row,start_col);
        difsum_right(row,start_col); 
        centerline[row] = 0.5 * (rightline[row] + leftline[row]);
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     差比和寻找左侧边界点
参数说明     
返回参数     
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/void difsum_left(uint8 y,uint8 x){
    float sum,dif,sar;//和，差，比
    uint8 col;//列
    uint8 mov = 3;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    leftline[y] = 0;//未找到左边界时输出为0
    for(col = x; col >= mov + 1; col -= mov){
        dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col - mov - 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
        sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col - mov - 1]));
        sar = fabs(dif / sum);//求取差比和
        if(sar > sar_thre){//差比和大于阈值代表深浅色突变
            leftline[y] = (int16)(col - mov);
            leftline_num ++;//左线点计数+
            break;//找到边界后退出
        }
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     差比和寻找右侧边界点
参数说明     
返回参数     
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/void difsum_right(uint8 y,uint8 x){
    float sum,dif,sar;//和，差，比
    uint8 col;//列
    uint8 mov = 3;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    rightline[y] = MT9V03X_W - 1;//未找到右边界时输出为187
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov){
        dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col + mov + 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
        sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col + mov + 1]));
        sar = fabs(dif / sum);//求取差比和
        if(sar > sar_thre){//差比和大于阈值代表深浅色突变
            rightline[y] = (int16)(col + mov);
            rightline_num ++;//右线点计数+
            break;//找到边界后退出
        }
    }
}


//这里都是差比和↑↑↑↑↑↑↑

//大津法↓↓↓↓↓↓↓↓↓↓
uint8 dis_image[MT9V03X_H][MT9V03X_W];
int16 image_threshold = 46;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，实测使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
	if(threshold<170)
	{
		return 170;
	}
	if(threshold>200)
	{
		return 200;
	}
    return threshold;
}
void set_b_imagine(int threshold)
{
		for(int16 i=0;i<MT9V03X_H;i++)
	{
		for(int16 j=0;j<MT9V03X_W;j++)
		{
			dis_image[i][j]=(mt9v03x_image[i][j]>threshold)?255:0;
		}
	}
}

void difsum_left1(uint8 y,uint8 x)
	{
    uint8 col;//列
    uint8 mov = 2;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    leftline[y] = 0;//未找到左边界时输出为0
    for(col = x; col >= mov + 1; col -= mov)
	{
		if(dis_image[y][col] - dis_image[y][col - mov]>0)
		{
			leftline[y] = col;
			leftline_num ++;//左线点计数+
			break;//找到边界后退出
		}
		else
		{
			leftlostpoint[0]++;
			left_lost_flag[y]=1;
			if(leftlostpoint[1]==0)
			{
				leftlostpoint[1]=y;
				
			}
		}

	}
}

void difsum_right1(uint8 y,uint8 x)
{
	uint8 col;//列
    uint8 mov = 2;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    rightline[y] = MT9V03X_W-1;//未找到左边界时输出为0
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov)
	{
		if(dis_image[y][col] - dis_image[y][col + mov]>5)
		{
			rightline[y] = col ;
			rightline_num ++;//右边线点计数+
			break;//找到边界后退出
		}
		else
		{
			rightlostpoint[0]++;
			right_lost_flag[y]=1;

			if(rightlostpoint[1]==0)
			{
				rightlostpoint[1]=y;
			}
		}

	}
}
void image_boundary_process2(void)
	{
    uint8 row;//行
    //uint8 col = MT9V03X_W/2;//列
    uint8 start_col = MT9V03X_W / 2;//各行起点的列坐标,默认为MT9V03X_W / 2
    //清零之前的计数
    leftline_num = 0;
    rightline_num = 0;

    for(row = MT9V03X_H - 1; row >= 1; row--){
        //选用上一行的中点作为下一行计算起始点，节省速度，同时防止弯道的左右两边均出现与画面一侧
        if(row != MT9V03X_H - 1){
            
            
            if(carstatus_now==straight)
            {			
                if(centerline[row+1]==0)
                {
                    start_col=(uint8)(MT9V03X_W / 2);
                }
                else if(rightline[row+1]!=MT9V03X_W-1&&leftline[row+1]!=0)
                {
                    start_col=(rightline[row+1]+leftline[row+1])/2;
                }
                else
                {
                    start_col = centerline[row+1];//一阶低通滤波，防止出现噪点影响下一行的起始点
                }
            }
            else if(carstatus_now==crossroad)
            {
                start_col=bailie_lock_crossroad;
              
            }
            else if(carstatus_now==round_4)
            {
                start_col=90;
            }


            
		}
        else if(row == MT9V03X_H - 1){
            start_col = (uint8)(MT9V03X_W / 2);
        }

        //逐行作差比和 
        difsum_left1(row,start_col);
        difsum_right1(row,start_col); 
		for(int16 i=MT9V03X_H-3;i>3;i--)
		{
			if(right_lost_flag[i]==1&&left_lost_flag[i]==1)
			{
				both_lost_flag[i]=1;
				bothlostpoint[0]++;
				if(bothlostpoint[1]==0)
				{
					bothlostpoint[1]=i;
				}
			}
		}
        centerline[row] = 0.5 * (rightline[row] + leftline[row]);
    }
}


//大津法↑↑↑↑↑↑↑↑↑↑
//斑马线处理和保护在这里
void black_protect_check(void)
{
    int16 sum =0;
    for(int16 i=70;i>20;i--)
    {
        if(mt9v03x_image[ MT9V03X_H - 1][i]<150)
        {
            sum++;
        }

    }
        for(int16 i=70;i<120;i++)
    {
        if(mt9v03x_image[ MT9V03X_H - 1][i]<150)
        {
            sum++; 
        }

    }
            if (sum>100*0.8)
        {
            stop_flag1=true;
        }
}



void banmaxian_check(void)
{
	int16 count=0;
    int16 sum =0;
    bool black=0;
    for(int i=MT9V03X_H-1;i>=MT9V03X_H-3;i--)
            {
                for(int j=0;j<=MT9V03X_W-1-3;j++)
                {
                    if(dis_image[i][j]==255&&dis_image[i][j+1]==0&&dis_image[i][j+2]==0)
                    {
                        count++;
                    }
                }
                if(count>=10)//如果黑色计数大于等于40，认为是斑马线
                {
                    stop_flag1=true;
                }
            }
}


/*
------------------------------------------------------------------------------------------------------------------
函数简介      输出中点位置：这里是从底向上数第四个
参数说明     无
返回参数     int16，中点的横坐标
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16 output_middle(void)
{
    return centerline[MT9V03X_H-15];
}


bool stop_flag(void)
{
    if(leftline_num<0.1*MT9V03X_H   &&rightline_num<0.1*MT9V03X_H)
    {
        return true;
    }
    else return false;
}
//十字判断
//十字判断
//十字判断
//十字判断
//十字判断
//十字判断


/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int16 start,int16 end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int16 start,int16 end)
{
    int16 i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end>=MT9V03X_H-10)
        end=MT9V03X_H-10;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
           abs(leftline[i]-leftline[i+1])<=5&&//角点的阈值可以更改
           abs(leftline[i+1]-leftline[i+2])<=5&&
           abs(leftline[i+2]-leftline[i+3])<=5&&
            ((leftline[i]-leftline[i-2])>=5||leftline[i-2]<=4)&&
            ((leftline[i]-leftline[i-3])>=7||leftline[i-3]<=4)&&
            ((leftline[i]-leftline[i-4])>=7||leftline[i-4]<=4))
        {
            Left_Down_Find=i+2;//获取行数即可
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(rightline[i]-rightline[i+1])<=5&&//角点的阈值可以更改
           abs(rightline[i+1]-rightline[i+2])<=5&&
           abs(rightline[i+2]-rightline[i+3])<=5&&
              ((rightline[i]-rightline[i-2])<=-5||rightline[i-2]>=MT9V03X_W-4)&&
              ((rightline[i]-rightline[i-3])<=-7||rightline[i-3]>=MT9V03X_W-4)&&
              ((rightline[i]-rightline[i-4])<=-7||rightline[i-4]>=MT9V03X_W-4))
        {
            Right_Down_Find=i+2;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }
        if(abs(Right_Down_Find-Left_Down_Find)>=50)//纵向撕裂过大，视为误判
    {
        Right_Down_Find=0;
        Left_Down_Find=0;
    }
//    if(Right_Down_Find==74)
//    {
//            Right_Down_Find=0;

//    }
//        if(Left_Down_Find==74)
//    {
//            Left_Down_Find=0;

//    }

}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int16 start,int16 end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int16 start,int16 end)
{
    int16 i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start>end)
    {
        t=start;
        start=end;
        end=t;
    }
    //start<end由上往下
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start<=5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=5;
    for(i=start;i<=end;i++)
    {
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(leftline[i]-leftline[i-1])<=5&&
           abs(leftline[i-1]-leftline[i-2])<=5&&
           abs(leftline[i-2]-leftline[i-3])<=5&&
              ((leftline[i]-leftline[i+2])>=7||leftline[i+2]<4)&&
              ((leftline[i]-leftline[i+3])>=7||leftline[i+3]<4)&&
              ((leftline[i]-leftline[i+4])>=7||leftline[i+4]<4))
        {
            Left_Up_Find=i-2;//获取行数即可

        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(rightline[i]-rightline[i-1])<=5&&//下面两行位置差不多
           abs(rightline[i-1]-rightline[i-2])<=5&&
           abs(rightline[i-2]-rightline[i-3])<=5&&
              ((rightline[i]-rightline[i+2]<=-7)||rightline[i+2]>MT9V03X_W-4)&&
              ((rightline[i]-rightline[i+3])<=-7||rightline[i+3]>MT9V03X_W-4)&&
              ((rightline[i]-rightline[i+4])<=-7||rightline[i+4]>MT9V03X_W-4))
        {
            Right_Up_Find=i-2;//获取行数即可

        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=50)//纵向撕裂过大，视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
    
}


/*
------------------------------------------------------------------------------------------------------------------
函数简介     滑动平均滤波左
参数说明     无
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void dx1_left_average(float dx)
{
    for(uint8 i=1;i<5;i++)
    {
        dx1[i-1]=dx1[i];
    }
    dx1[4]=dx;
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     滑动平均滤波右
参数说明     无
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void dx2_right_average(float dx)
{
    for(uint8 i=1;i<5;i++)
    {
        dx2[i-1]=dx2[i];
    }
    dx2[4]=dx;
}

//圆环判断
//1.找圆环

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Down_Point(int16 start,int16 end)//找左下角点，返回值是角点所在的行数
 {
    int16 i,t;
    int16 left_down_line=0;
    if(leftline_num<=0.1*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
       return left_down_line;
    if(start<end)//--访问，要保证start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;//另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
    if(end<=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//只找第一个符合条件的点
           abs(leftline[i]-leftline[i+1])<=5&&//角点的阈值可以更改
           abs(leftline[i+1]-leftline[i+2])<=5&&  
           abs(leftline[i+2]-leftline[i+3])<=5&&
              (leftline[i]-leftline[i-2])>=5&&
              (leftline[i]-leftline[i-3])>=10&&
              (leftline[i]-leftline[i-4])>=10)
        {
            left_down_line=i;//获取行数即可
            break;
        }
    }
    return left_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     right_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Right_Down_Point(uint8 start,uint8 end)
{
    right_down_line=0;
    if(start>MT9V03X_H-6)
    {
        start=MT9V03X_H-6;
    }
    if(end<5){end=5;}
    if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    for(int16 i=start;i>=end;i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
           abs(rightline[i]-rightline[i+1])<=5&&//角点的阈值可以更改
           abs(rightline[i+1]-rightline[i+2])<=5&&  
           abs(rightline[i+2]-rightline[i+3])<=5&&
              ((rightline[i]-rightline[i-2])>=5 ||rightline[i-2]==MT9V03X_W-1)&&
              ((rightline[i]-rightline[i-3])>=10||rightline[i-3]==MT9V03X_W-1)&&
              ((rightline[i]-rightline[i-4])>=10||rightline[i-4]==MT9V03X_W-1))
        {
            right_down_line=i;
            break;
        }
    }
//    printf("find_pointright%d,",right_down_line);
    return right_down_line;                                                         //在i处有角点
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     确定连续性，用于处理右侧线。
参数说明     uint8起点和终点
返回参数     中点位置
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16 continuity_right(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start>=MT9V03X_H-2)//数组越界保护
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }
        if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(rightline[i]-rightline[i-1])>=7)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;                                         //在i处不连续了

            break;
       }

    }
//      printf("continuity_right%d,\n",continuity_change_flag);没问题

    return continuity_change_flag;
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     确定连续性，用于处理左侧线。
参数说明     uint8起点和终点
返回参数     中点位置
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/

int16 continuity_left(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start>=MT9V03X_H-2)//数组越界保护
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }
        if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(leftline[i]-leftline[i-1])>=7)//连续性阈值是5，可更改
       {

            continuity_change_flag=i;                                         //在i处不连续了

            break;
       }

    }
        printf("continuity_left%d,\n",continuity_change_flag);
    return continuity_change_flag;
}
//单调性变化s
int16 montonicity_right(uint8 start,uint8 end)
{            

    int16 i;
    int16 result=0;

            if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-5)//数组越界保护
        start=MT9V03X_H-5;
    if(end<=5)
    {
        end=5;
    }
    for(i=start;i>=end;i--)
    {

        if(rightline[i]-rightline[i-1]<=0&&rightline[i-1]-rightline[i-2]<=0&&rightline[i-2]-rightline[i-3]<=0
            &&rightline[i]-rightline[i+1]<=0&&rightline[i+1]-rightline[i+2]<=0&&rightline[i+2]-rightline[i+3]<=0&&
        rightline[i]!=MT9V03X_W-1&&
        rightline[i+1]!=MT9V03X_W-1&&
        rightline[i-1]!=MT9V03X_W-1&&
        rightline[i+2]!=MT9V03X_W-1&&
        rightline[i-2]!=MT9V03X_W-1)
        {
            result=i;
        return result;

        }
    }
    return 0;

}



/*
------------------------------------------------------------------------------------------------------------------
以下都是补线
以下都是补线
以下都是补线
以下都是补线
以下都是补线
-------------------------------------------------------------------------------------------------------------------
*/


/*
------------------------------------------------------------------------------------------------------------------
函数简介     补线左
参数说明     起点x，y终点y，增长率dx
返回参数     无
使用示例     
备注信息     增长率dx等于x1-x2/y1-y2
-------------------------------------------------------------------------------------------------------------------
*/
void draw_Lline_k(int16 startx, int16 starty, int16 endy, float dx) {
    int16 step = (starty < endy) ? 1 : -1;
    if (dx == 0) {

        for (int16 i =starty; i != endy; i += step) {
            leftfollowline[i] = startx;
        }
        return;
    }
    for (int16 i = starty; i != endy; i += step) {

        int16 temp=startx + (int16)((float)(i - starty) * dx );   
            leftfollowline[i] = temp;
        
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     补线右
参数说明     起点x，y终点y，增长率dx
返回参数     无  
使用示例     
备注信息     增长率dx等于x1-x2/y1-y2
-------------------------------------------------------------------------------------------------------------------
*/
void draw_Rline_k(int16 startx, int16 starty, int16 endy, float dx) {


    int16 step = (starty < endy) ? 1 : -1;

    if (dx == 0) 
    {                
        for (int16 i = starty; i != endy; i += step) 
        {
            rightfollowline[i] = startx;
        }
        return;
    }
    for (int16 i = starty; i != endy; i += step) {
        int16 temp=startx + (int16)((float)(i - starty) * dx );      
        rightfollowline[i] = temp;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     右两点间连线
参数说明     起点xy，终点xy
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void add_Rline_k(int16 startx, int16 starty, int16 endy,int16 endx)
{
    if(endy!=starty)
    {
        float dx=(float)(endx-startx)/(float)(endy-starty);
        draw_Rline_k(startx,starty,endy,dx);
    }
    else
    {
        return;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     左两点间连线
参数说明     起点xy，终点xy
返回参数     无
使用示例     
备注信息     能除以0那你真牛逼
-------------------------------------------------------------------------------------------------------------------
*/
void add_Lline_k(int16 startx, int16 starty, int16 endy,int16 endx)
{
    if(endy!=starty)
    {
        float dx=(float)(endx-startx)/(float)(endy-starty);
        draw_Lline_k(startx,starty,endy,dx);
    }
    else{
         return;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     自上而下补左线
参数说明     起点
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void lenthen_Left_bondarise(int16 start)
{
    if(start<7){start=7;}
    if(start>MT9V03X_H-7){start=MT9V03X_H-7;}
    float dx=(float)(leftline[start]-leftline[start-7])/7;
    dx1_left_average(dx);
    float dx_average=(dx1[0]+dx1[1]+dx1[2]+dx1[3]+dx1[4])/5;
    for(int16 i=start;i<MT9V03X_H-1;i++)
    {
        if((float)leftline[start]+(float)(dx_average*(i-start))<0||(float)leftline[start]+dx_average*(float)(i-start)>(float)MT9V03X_W)
        {
            break;
        }
        else
        {
            leftfollowline[i]=(int16)((float)leftline[start]+dx_average*(float)(i-start));
        }
    }
}
 /*
------------------------------------------------------------------------------------------------------------------
函数简介     自上而下补右线
参数说明     起点
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/  
void lenthen_Right_bondarise(int16 start) 
{
    if(start<7){start=7;}
    if(start>MT9V03X_H-7){start=MT9V03X_H-7;}
    float dx=(float)(rightline[start]-rightline[start-7])/7;
    dx2_right_average(dx);
    float dx_average=(dx2[0]+dx2[1 ]+dx2[2]+dx2[3]+dx2[4])/5;
    for(int16 i=start;i<MT9V03X_H-1;i++)
    {
        if((float)rightline[start]+dx_average*(i-start)>MT9V03X_W-5||(float)rightline[start]+dx_average*(float)(i-start)<0)
        {
            break;
        }
        else 
        {
            rightfollowline[i]=(int16)((float)rightline[start]+dx_average*(float)(i-start));
        }
    }
}
   