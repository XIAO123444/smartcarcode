 /*
 * screen.c
 *
 *  Created on: 2023年10月24日
 *      Author: lychee
 */
#include "screen.h"
#include "photo_chuli.h"
extern int16 centerline[MT9V03X_H];
extern int16 leftline[MT9V03X_H];
extern int16 rightline[MT9V03X_H];
extern int16 rightfollowline[MT9V03X_H];
extern int16 leftfollowline[MT9V03X_H];
extern int16 centerline2[MT9V03X_H];
void show_line(void){

    for(int16 i = 0; i < MT9V03X_H-1; i ++){
//        ips200_draw_point((uint16)leftline[i], i+120, RGB565_RED);//红色左线
//        ips200_draw_point((uint16)rightline[i], i+120, RGB565_BLUE);//蓝色右线
//        ips200_draw_point((uint16)centerline[i], i+120, RGB565_PURPLE);//紫色中线
        ips200_draw_point((uint16)leftfollowline[i], i+120, RGB565_RED);//红色左线
        ips200_draw_point((uint16)rightfollowline[i], i+120, RGB565_BLUE);//蓝色右线
        ips200_draw_point((uint16)centerline2[i], i+120, RGB565_PURPLE);//紫色中线
    }
}
