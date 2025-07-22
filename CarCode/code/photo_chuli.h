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

//ʮ��
void Find_Down_Point(int16 start,int16 end);
void Find_Up_Point(int16 start,int16 end);

void draw_Lline_k(int16 startx, int16 starty, int16 endy, float dx);
void draw_Rline_k(int16 startx, int16 starty, int16 endy, float dx);

void add_Rline_k(int16 startx, int16 starty, int16 endy,int16 endx);
void add_Lline_k(int16 startx, int16 starty, int16 endy,int16 endx);

void lenthen_Left_bondarise(int16 start);
void lenthen_Right_bondarise(int16 start);

void banmaxian_check(void);
void black_protect_check(void);

int16 output_middle(void);

#endif