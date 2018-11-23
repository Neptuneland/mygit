//
// Created by peng on 18-11-13.
//

#ifndef CARPLAY_V3_T_FU_DATA_H
#define CARPLAY_V3_T_FU_DATA_H

//
// Created by peng on 18-11-12.
//

////////////////////////////////////////////////////////////////
//
//  智能小车项目，南京理工大学计算机科学与工程学院
//  FileName:  T_FU_DATA.h
//  Author: 王东生
//  Date:   2018.7.3
//  Description: 融合模块的数据定义
//  Ｎote:仅先考虑静态障碍物,生成栅格地图并附上障碍物占据的栅格列表
//
////////////////////////////////////////////////////////////////

#ifndef _T_FU_DATA_H
#define _T_FU_DATA_H

#include <deque>
using namespace std;

#define T_3D16_OBS_MAX_POINT_NUM 4    //每个障碍物最多用T_OBS_MAX_POINT个点描述
#define T_3D16_OBS_MAX_GRID_NUM 100   //每个障碍物最多占据多少个栅格
#define T_3D16_OBS_MAX_NUM 10         //每帧最大障碍物数量
#define T_CAR_GRID_DISTANCE_V_CM 1000 //车体栅格地图垂直(vertical)距离，车前方1000,后方0,单位cm
#define T_CAR_GRID_DISTANCE_H_CM 200  //车体栅格地图水平(Horizontal)距离，车左右各100,单位cm
#define T_CAR_GRID_RESOLUTION_V_CM 10 //车体栅格地图垂直分辨率，单位cm
#define T_CAR_GRID_RESOLUTION_H_CM 10 //车体栅格地图水平分辨率，单位cm
#define T_CAR_GRID_V_NUM 100          //车体栅格地图垂直方向栅格数
#define T_CAR_GRID_H_NUM 20           //车体栅格地图水平方向栅格数


/*3D16障碍物*/
//2D点描述
struct T_POINT_2D
{
    int x_cm; //车前进方向右转90度为ｘ正方向，单位cm
    int y_cm; //车前进方向为ｙ正方向,单位cm
};

//(静态)障碍物结构体，（凸障碍凹障碍水坑石头等障碍都用四点描述）
typedef struct tag_T_3D16_OBS_DATA
{
    int OBS_ID; //障碍物ID
    T_POINT_2D pPoint[T_3D16_OBS_MAX_POINT_NUM];
    /* 四个点描述一个障碍物
     *pPoint[0]:该障碍物最左边点的坐标
     *pPoint[1]:该障碍物最右边点的坐标
     *pPoint[2]:横坐标为[0]和[1]的中点，纵坐标为直线x=(pPoint[0].x_cm+pPoint[1].x_cm)/2上最靠近车头的障碍点的纵坐标,距车头近，可称为“前方的点”
     *pPoint[3]:可称为"后方的点"，距车头远，坐标大致在远处即可，车不会到达该点
    */
    int nPoint; //描述障碍物的实际有效点数
} T_3D16_OBS_DATA;

//3D给融合的一帧(静态)障碍物数据
typedef struct tagT_3D16_OBS_TO_FU
{
    int frameID;                              //帧ID(从0开始)
    int syntime;                              //时间戳
    int navID;                                //与图像获取时间最接近的导航数据编号(从0开始)
    T_3D16_OBS_DATA pObs[T_3D16_OBS_MAX_NUM]; //障碍物数组
    int nObs;                                 //障碍物有效数量
} T_3D16_OBS_TO_FU;

/*栅格地图,给融合的和融合发出去的有何区别*/
//每一帧栅格数据
typedef struct tagT_3D16_GRID_TO_FU
{
    int frameID;                                                //帧号
    int syntime;                                                //时间戳
    int navID;                                                  //最近的惯导号
    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //栅格数组,值0-255，0表示无障碍，非0表示有障碍
    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //数字数组，每行表示该障碍物占据的栅格标号，以-1结束
    int nObs;                                                   //障碍物有效数量
} T_3D16_GRID_TO_FU;

typedef struct tagT_FU_TO_PL
{
    int frameID;                                                //帧号
    int syntime;                                                //时间戳
    int navID;                                                  //最近的惯导号
    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //栅格数组,值0-255，0表示无障碍，非0表示有障碍
    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //数字数组，每行表示该障碍物占据的栅格标号，以-1结束
    int nObs;                                                   //障碍物有效数量
} T_FU_TO_PL;

/*T_MC.h*/
typedef struct tagT_MC_TO_FU
{
    int navID;
    int syntime;
    double Longitude_degree; //纬度　单位:度
    double Latitude_degree;  //经度　单位:度
    double Altitude_m;       //海拔　单位:米
    double EarthRefCoord[2]; // 地面平面坐标系 单位:米  [0] 北向 +X     [1] 东向 +Y

    //横摆角　俯仰角　航向角
    float Roll_rad;  //单位:弧度
    float Pitch_rad; //单位:弧度
    float Yaw_rad;   //单位:弧度
} T_MC_TO_FU;

#endif

/* T_MC.h
 * 结构体中包含帧号、位姿、速度、GPS坐标等信息；
 * 保存一定数量的历史帧信息，如50帧，可用std::deque;
 * 根据惯导号从历史信息中查找出相应帧信息的函数；
 *
 * T_ESR.h
 * 为3D16提供位置参考，与FU无直接通信，自行定义。
 *
 * T_2D.h　（单目双目）
 * 相机FU暂时用不到，自行定义。
 *  * 其他：
 * 时间戳如何获取？发送的每一帧数据都要有时间戳信息
*/


#endif //CARPLAY_V3_T_FU_DATA_H
