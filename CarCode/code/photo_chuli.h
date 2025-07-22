#ifndef PHOTO_CHULI_H__
#define PHOTO_CHULI_H__

#include "zf_common_headfile.h"
void difsum_left(uint8 y,uint8 x);
void difsum_right(uint8 y,uint8 x);
void image_boundary_process(void);

void set_b_imagine(int threshold);
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row);
void image_boundary_process2(void);

bool stop_flag(void);

int16 Find_Left_Down_Point(int16 start,int16 end);
int16 Find_Right_Down_Point(uint8 start,uint8 end);
int16 continuity_left(uint8 start,uint8 end);
int16 montonicity_right(uint8 start,uint8 end);
int16 continuity_right(uint8 start,uint8 end);

//十字
void Find_Down_Point(int16 start,int16 end);                    //寻找下点
void Find_Up_Point(int16 start,int16 end);                      //寻找上点  

void draw_Lline_k(int16 startx, int16 starty, int16 endy, float dx);        //画左边界线
void draw_Rline_k(int16 startx, int16 starty, int16 endy, float dx);        //画右边界线

void add_Rline_k(int16 startx, int16 starty, int16 endy,int16 endx);        //添加右边界线
void add_Lline_k(int16 startx, int16 starty, int16 endy,int16 endx);        //添加左边界线

void lenthen_Left_bondarise(int16 start);                   //延长左边界线
void lenthen_Right_bondarise(int16 start);                  //延长右边界线

void banmaxian_check(void);                         //斑马线保护
void black_protect_check(void);                         //黑线保护

int16 output_middle(void);                          //输出中线

#endif