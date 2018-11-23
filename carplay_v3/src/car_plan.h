//
// Created by peng on 18-11-12.
//

#ifndef CARPLAY_V3_CAR_PLAN_H
#define CARPLAY_V3_CAR_PLAN_H


#include <math.h>
#include <stdio.h>
#include <vector>
#include <string.h>

#include <fstream>


#define T_CAR_GRID_DISTANCE_V_CM 1000 //车体栅格地图垂直(vertical)距离，车前方1000,后方0,单位cm
#define T_CAR_GRID_DISTANCE_H_CM 200  //车体栅格地图水平(Horizontal)距离，车左右各100,单位cm
#define T_CAR_GRID_RESOLUTION_V_CM 10 //车体栅格地图垂直分辨率，单位cm
#define T_CAR_GRID_RESOLUTION_H_CM 10 //车体栅格地图水平分辨率，单位cm
#define T_CAR_GRID_V_NUM 100          //车体栅格地图垂直方向栅格数
#define T_CAR_GRID_H_NUM 20           //车体栅格地图水平方向栅格数



/*
 * 现在有的数据：gridCell[2000]，1为障碍，注意：是10Hz实时刷新的！要根据此做随机避障规划：环境想象成楼道，
 * 1、看栅格结构，
 * 2、转向，根据能够前进的位置与中心点的距离设置反比速度，为零时退出while(1)循环进入行进模式，
 * 3、行进，设置安全行驶距离，设置与障碍物距离反比的速度，
 */

/* gridCell[i]所在的行：i/20；列：i%20;
 * 判断障碍物：if(girdCell[i]=1,计算(gC[i-1]+gC[i]+gC[i+1]+gC[i-20]+gC[i+20])>2是否成立
 * 计算距离：得到gC[i]的i，int x=i%20, int y =i/20, dis = sqrt(x*x+ y*y)
 * 速度变化0-127,v=127*()
 * 规划：从左往右，左边只要一个栅格有障碍物，检查右边两格（或者右边六格）加起来是否>=1，若成立继续往右扫
 *
 *
 * 先恒定低速跑，结合紧急刹车，设定速度再说
 *
 * 紧急刹车：通过距离判断，小于阈值刹车
 *
 * 增加功能：设置一个状态变量，用于控制自主和非自主。发送指令前检查该变量是否为真，不为真取消自主控制。
 *         非自主控制时直接发数字控制指令（前后左右停）
 */


//前方障碍物判断函数，i_meters米内是否有障碍物

bool obsInFront(boost::array<unsigned char, 2000ul> gridCell, int i_meters);

//前方1米内是否有障碍，有的话紧急刹车
bool isEmergency(boost::array<unsigned char, 2000ul> gridCell);


//判断障碍物：if(girdCell[i]=1,计算(gC[i-1]+gC[i]+gC[i+1]+gC[i-20]+gC[i+20])>2是否成立,注意边角处不能加减行列的特殊情况
//在main函数中重新给它赋传入的消息的值
bool isObs(boost::array<unsigned char, 2000ul> gridCell,int i);


//计算距离函数
//计算距离：得到gC[i]的i，int x=i%20, int y =i/20, dis = sqrt(x*x+ y*y)

int getDistance(boost::array<unsigned char, 2000ul> gridCell, int i);

//输出十六进制：cout << "35的16进制:" << std::hex << 35 << endl;
int getCarSpeed(boost::array<unsigned char, 2000ul> gridCell, int i);//set speed depend with distance,





/***********************************************************************************************************
 **                     1118跟点走部分
 **                  全部转到大地坐标下进行
 ***********************************************************************************************************/


//订阅nav_mc的消息直接得到下面数据
//typedef struct tagT_MC_TO_FU
//{
//    int navID;
//    int syntime;
//    double Longitude_degree; //纬度　单位:度
//    double Latitude_degree;  //经度　单位:度
//    double Altitude_m;       //海拔　单位:米
//    double EarthRefCoord[2]; // 地面平面坐标系 单位:米  [0] 北向 +X     [1] 东向 +Y
//
//    //横摆角　俯仰角　航向角
//    float Roll_rad;  //单位:弧度
//    float Pitch_rad; //单位:弧度
//    float Yaw_rad;   //单位:弧度
//} T_MC_TO_FU;


//处理惯导信息，暂时只用偏航角
//如果有障碍物，得到经过障碍物的边缘的点的坐标，返回下一个跟的点的栅格的下标

int getObsCoor(boost::array<unsigned char, 2000ul> gridCell);

vector<double > gridToEarthXY(int gridSub, float theta,vector<double > carEarthXY);

//bool arrDest(vector<float > startXY(2), vector<float > destXY(2), float theta)

vector<double> gaussBLtoXY(double longitude,double latitude);


#endif




