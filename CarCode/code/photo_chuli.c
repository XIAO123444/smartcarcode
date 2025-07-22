/*
 * camera.c
 *
 *  Created on: 2023��10��24��
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

int16 leftlostpoint[2]={0,0};   //�����������ߵ�0Ϊ��������1Ϊ���ߵ�
int16 rightlostpoint[2]={0,0};  //�Ҷ����������ߵ�0Ϊ��������1Ϊ���ߵ�
int16 bothlostpoint[2]={0,0};   //ͬʱ�����������ߵ�0Ϊ��������1Ϊ���ߵ�

uint16 left_lost_flag[MT9V03X_H];//��������
uint16 right_lost_flag[MT9V03X_H];//�Ҷ�������
uint16 both_lost_flag[MT9V03X_H];//ͬʱ��������

//ʮ�֡�������
int16 Right_Down_Find=0;    //���µ�
int16 Left_Down_Find=0;     //���µ�
int16 Right_Up_Find=0;      //���ϵ�
int16 Left_Up_Find=0;       //���ϵ�
//ʮ�֡�������
//��״̬
enum mark {
    straight,    // ֱ����ʻ
    crossroad,   // ʮ��·��
    round_2,   // �뻷��ֱ��
    round_3,   // Բ����б�ߣ�δʹ�ã�
    round_4,   // �뻷��ʻ
    round_5,   // ��յ㲹б��
    round_6    // ������ֱ��
};
extern enum  mark carstatus_now;
//����
extern int16 bailie_lock_crossroad;
extern int16 bailieright_lock_round;

//�ߵ��붪�ߡ�������
uint8 leftline_num;         //���ߵ�����
uint8 rightline_num;        //���ߵ�����


//�ߵ��붪�ߡ�������

//Բ����������
int16 right_down_guai   =0;            //���¹յ�
int16 right_up_guai     =0;            //���Ϲյ�
int16 left_down_guai    =0;            //���¹յ�
int16 left_up_guai      =0;            //���Ϲյ�

int16 left_budandiao       =0;     //�󲻵���
int16 right_budandiao      =0;     //�Ҳ�����
//Բ����������



//��Ⱥ͡�������
int16 sar_thre = 17;//��Ⱥ���ֵ
//��Ⱥ͡�������
uint8 pix_per_meter = 20;//ÿ�׵�������


extern bool stop_flag1;

float dx1[5]={0};
float dx2[5]={0};

int16 right_down_line =0;

//���ﶼ�ǲ�Ⱥ͡�������������
/*
------------------------------------------------------------------------------------------------------------------
�������     ��Ⱥ�Ѱ�ұ߽��
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void image_boundary_process(void){
    uint8 row;//��
    //uint8 col = MT9V03X_W/2;//��
    uint8 start_col = MT9V03X_W / 2;//��������������,Ĭ��ΪMT9V03X_W / 2
    //����֮ǰ�ļ���
    leftline_num = 0;
    rightline_num = 0;

    for(row = MT9V03X_H - 1; row >= 1; row--){
        //ѡ����һ�е��е���Ϊ��һ�м�����ʼ�㣬��ʡ�ٶȣ�ͬʱ��ֹ������������߾������뻭��һ��
        if(row != MT9V03X_H - 1){
            start_col = (uint8)(0.4 * centerline[row] + 0.3 * start_col + 0.1 * MT9V03X_W);//һ�׵�ͨ�˲�����ֹ�������Ӱ����һ�е���ʼ��


        }
        else if(row == MT9V03X_H - 1){
            start_col = MT9V03X_W / 2;
        }
        if(start_col<MT9V03X_W/2-30){start_col=MT9V03X_W/2-30;}
        if(start_col>MT9V03X_W/2+30){start_col=MT9V03X_W/2+30;}
        //��������Ⱥ� 
        difsum_left(row,start_col);
        difsum_right(row,start_col); 
        centerline[row] = 0.5 * (rightline[row] + leftline[row]);
    }
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ��Ⱥ�Ѱ�����߽��
����˵��     
���ز���     
ʹ��ʾ��     
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/void difsum_left(uint8 y,uint8 x){
    float sum,dif,sar;//�ͣ����
    uint8 col;//��
    uint8 mov = 3;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    leftline[y] = 0;//δ�ҵ���߽�ʱ���Ϊ0
    for(col = x; col >= mov + 1; col -= mov){
        dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col - mov - 1])<<8);//����8λ����256���ɱ��⸡�����ˣ��ӿ��ٶ�
        sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col - mov - 1]));
        sar = fabs(dif / sum);//��ȡ��Ⱥ�
        if(sar > sar_thre){//��Ⱥʹ�����ֵ������ǳɫͻ��
            leftline[y] = (int16)(col - mov);
            leftline_num ++;//���ߵ����+
            break;//�ҵ��߽���˳�
        }
    }
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ��Ⱥ�Ѱ���Ҳ�߽��
����˵��     
���ز���     
ʹ��ʾ��     
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/void difsum_right(uint8 y,uint8 x){
    float sum,dif,sar;//�ͣ����
    uint8 col;//��
    uint8 mov = 3;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    rightline[y] = MT9V03X_W - 1;//δ�ҵ��ұ߽�ʱ���Ϊ187
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov){
        dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col + mov + 1])<<8);//����8λ����256���ɱ��⸡�����ˣ��ӿ��ٶ�
        sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col + mov + 1]));
        sar = fabs(dif / sum);//��ȡ��Ⱥ�
        if(sar > sar_thre){//��Ⱥʹ�����ֵ������ǳɫͻ��
            rightline[y] = (int16)(col + mov);
            rightline_num ++;//���ߵ����+
            break;//�ҵ��߽���˳�
        }
    }
}


//���ﶼ�ǲ�Ⱥ͡�������������

//��򷨡�������������������
uint8 dis_image[MT9V03X_H][MT9V03X_W];
int16 image_threshold = 46;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���ٴ������ֵ������ɽ��
  @param     image       ͼ������
             col         �� �����
             row         �У�����
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//ɽ�����ٴ��
  @note      ��˵�ȴ�ͳ��򷨿�һ�㣬ʵ��ʹ��Ч�����
-------------------------------------------------------------------------------------------------------------------*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
    //����ÿ������ֵ�ĵ�������ͼ���еı���
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
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
    uint8 col;//��
    uint8 mov = 2;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    leftline[y] = 0;//δ�ҵ���߽�ʱ���Ϊ0
    for(col = x; col >= mov + 1; col -= mov)
	{
		if(dis_image[y][col] - dis_image[y][col - mov]>0)
		{
			leftline[y] = col;
			leftline_num ++;//���ߵ����+
			break;//�ҵ��߽���˳�
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
	uint8 col;//��
    uint8 mov = 2;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    rightline[y] = MT9V03X_W-1;//δ�ҵ���߽�ʱ���Ϊ0
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov)
	{
		if(dis_image[y][col] - dis_image[y][col + mov]>5)
		{
			rightline[y] = col ;
			rightline_num ++;//�ұ��ߵ����+
			break;//�ҵ��߽���˳�
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
    uint8 row;//��
    //uint8 col = MT9V03X_W/2;//��
    uint8 start_col = MT9V03X_W / 2;//��������������,Ĭ��ΪMT9V03X_W / 2
    //����֮ǰ�ļ���
    leftline_num = 0;
    rightline_num = 0;

    for(row = MT9V03X_H - 1; row >= 1; row--){
        //ѡ����һ�е��е���Ϊ��һ�м�����ʼ�㣬��ʡ�ٶȣ�ͬʱ��ֹ������������߾������뻭��һ��
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
                    start_col = centerline[row+1];//һ�׵�ͨ�˲�����ֹ�������Ӱ����һ�е���ʼ��
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

        //��������Ⱥ� 
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


//��򷨡�������������������
//�����ߴ���ͱ���������
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
                if(count>=10)//�����ɫ�������ڵ���40����Ϊ�ǰ�����
                {
                    stop_flag1=true;
                }
            }
}


/*
------------------------------------------------------------------------------------------------------------------
�������      ����е�λ�ã������Ǵӵ����������ĸ�
����˵��     ��
���ز���     int16���е�ĺ�����
ʹ��ʾ��     
��ע��Ϣ     
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
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�


/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int16 start,int16 end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
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
    if(start>=MT9V03X_H-1-5)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=MT9V03X_H-1-5;
    if(end>=MT9V03X_H-10)
        end=MT9V03X_H-10;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(leftline[i]-leftline[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(leftline[i+1]-leftline[i+2])<=5&&
           abs(leftline[i+2]-leftline[i+3])<=5&&
            ((leftline[i]-leftline[i-2])>=5||leftline[i-2]<=4)&&
            ((leftline[i]-leftline[i-3])>=7||leftline[i-3]<=4)&&
            ((leftline[i]-leftline[i-4])>=7||leftline[i-4]<=4))
        {
            Left_Down_Find=i+2;//��ȡ��������
        }
        if(Right_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(rightline[i]-rightline[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(rightline[i+1]-rightline[i+2])<=5&&
           abs(rightline[i+2]-rightline[i+3])<=5&&
              ((rightline[i]-rightline[i-2])<=-5||rightline[i-2]>=MT9V03X_W-4)&&
              ((rightline[i]-rightline[i-3])<=-7||rightline[i-3]>=MT9V03X_W-4)&&
              ((rightline[i]-rightline[i-4])<=-7||rightline[i-4]>=MT9V03X_W-4))
        {
            Right_Down_Find=i+2;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//�����ҵ����˳�
        {
            break;
        }
    }
        if(abs(Right_Down_Find-Left_Down_Find)>=50)//����˺�ѹ�����Ϊ����
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
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int16 start,int16 end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
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
    //start<end��������
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��
        end=5;
    if(start<=5)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=5;
    for(i=start;i<=end;i++)
    {
        if(Left_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(leftline[i]-leftline[i-1])<=5&&
           abs(leftline[i-1]-leftline[i-2])<=5&&
           abs(leftline[i-2]-leftline[i-3])<=5&&
              ((leftline[i]-leftline[i+2])>=7||leftline[i+2]<4)&&
              ((leftline[i]-leftline[i+3])>=7||leftline[i+3]<4)&&
              ((leftline[i]-leftline[i+4])>=7||leftline[i+4]<4))
        {
            Left_Up_Find=i-2;//��ȡ��������

        }
        if(Right_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(rightline[i]-rightline[i-1])<=5&&//��������λ�ò��
           abs(rightline[i-1]-rightline[i-2])<=5&&
           abs(rightline[i-2]-rightline[i-3])<=5&&
              ((rightline[i]-rightline[i+2]<=-7)||rightline[i+2]>MT9V03X_W-4)&&
              ((rightline[i]-rightline[i+3])<=-7||rightline[i+3]>MT9V03X_W-4)&&
              ((rightline[i]-rightline[i+4])<=-7||rightline[i+4]>MT9V03X_W-4))
        {
            Right_Up_Find=i-2;//��ȡ��������

        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//���������ҵ��ͳ�ȥ
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=50)//����˺�ѹ�����Ϊ����
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
    
}


/*
------------------------------------------------------------------------------------------------------------------
�������     ����ƽ���˲���
����˵��     ��
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     ����ƽ���˲���
����˵��     ��
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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

//Բ���ж�
//1.��Բ��

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���½ǵ���
  @param     ��ʼ�У���ֹ��
  @return    ���ؽǵ����ڵ��������Ҳ�������0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      �ǵ�����ֵ�ɸ���ʵ��ֵ����
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Down_Point(int16 start,int16 end)//�����½ǵ㣬����ֵ�ǽǵ����ڵ�����
 {
    int16 i,t;
    int16 left_down_line=0;
    if(leftline_num<=0.1*MT9V03X_H)//�󲿷ֶ����ߣ�û�йյ��жϵ�����
       return left_down_line;
    if(start<end)//--���ʣ�Ҫ��֤start>end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=MT9V03X_H-1-5;//��һ���棬���жϵ�i��ʱ������ʵ�i+3��i-4�У���ֹԽ��
    if(end<=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(left_down_line==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(leftline[i]-leftline[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(leftline[i+1]-leftline[i+2])<=5&&  
           abs(leftline[i+2]-leftline[i+3])<=5&&
              (leftline[i]-leftline[i-2])>=5&&
              (leftline[i]-leftline[i-3])>=10&&
              (leftline[i]-leftline[i-4])>=10)
        {
            left_down_line=i;//��ȡ��������
            break;
        }
    }
    return left_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���½ǵ���
  @param     ��ʼ�У���ֹ��
  @return    ���ؽǵ����ڵ��������Ҳ�������0
  Sample     right_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      �ǵ�����ֵ�ɸ���ʵ��ֵ����
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
        if(right_down_line==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(rightline[i]-rightline[i+1])<=5&&//�ǵ����ֵ���Ը���
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
    return right_down_line;                                                         //��i���нǵ�
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ȷ�������ԣ����ڴ����Ҳ��ߡ�
����˵��     uint8�����յ�
���ز���     �е�λ��
ʹ��ʾ��     
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/
int16 continuity_right(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start>=MT9V03X_H-2)//����Խ�籣��
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
        if(abs(rightline[i]-rightline[i-1])>=7)//��������ֵ��5���ɸ���
       {
            continuity_change_flag=i;                                         //��i����������

            break;
       }

    }
//      printf("continuity_right%d,\n",continuity_change_flag);û����

    return continuity_change_flag;
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ȷ�������ԣ����ڴ�������ߡ�
����˵��     uint8�����յ�
���ز���     �е�λ��
ʹ��ʾ��     
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/

int16 continuity_left(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start>=MT9V03X_H-2)//����Խ�籣��
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
        if(abs(leftline[i]-leftline[i-1])>=7)//��������ֵ��5���ɸ���
       {

            continuity_change_flag=i;                                         //��i����������

            break;
       }

    }
        printf("continuity_left%d,\n",continuity_change_flag);
    return continuity_change_flag;
}
//�����Ա仯s
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
    if(start>=MT9V03X_H-5)//����Խ�籣��
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
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
-------------------------------------------------------------------------------------------------------------------
*/


/*
------------------------------------------------------------------------------------------------------------------
�������     ������
����˵��     ���x��y�յ�y��������dx
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     ������dx����x1-x2/y1-y2
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
�������     ������
����˵��     ���x��y�յ�y��������dx
���ز���     ��  
ʹ��ʾ��     
��ע��Ϣ     ������dx����x1-x2/y1-y2
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
�������     �����������
����˵��     ���xy���յ�xy
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     �����������
����˵��     ���xy���յ�xy
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     �ܳ���0������ţ��
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
�������     ���϶��²�����
����˵��     ���
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     ���϶��²�����
����˵��     ���
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
   