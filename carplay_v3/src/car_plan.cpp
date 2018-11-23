//
// Created by peng on 18-11-12.
//

#include "car_plan.h"
#include "serialport.h"


#include "carplay_v3/T_Msg_FU_TO_PL.h"



/************************************************************************************************************
 *
 * 随机走、键盘控制部分
 *
 ************************************************************************************************************/



//判断前方i_meters米(30格)内是否有障碍物
bool obsInFront(boost::array<uint8_t, 2000> gridCell,int i_meters)
{
    bool isObstacle = false;//默认前方无障碍
    for(int i = 0;i<T_CAR_GRID_H_NUM*i_meters*10;i++)
    {
        if((T_CAR_GRID_H_NUM/2-6)<i%T_CAR_GRID_H_NUM && i%T_CAR_GRID_H_NUM<(T_CAR_GRID_H_NUM/2+6))
        {
            if(isObs(gridCell,i))
            {
                isObstacle = true;
                break;
            }
        }
    }
    return isObstacle;
}

//前方0.8米内是否有障碍物
bool isEmergency(boost::array<uint8_t, 2000> gridCell)
{
    bool isObstacleE = false;
    for(int i = 0;i<T_CAR_GRID_H_NUM*10;i++)
    {
        if((T_CAR_GRID_H_NUM/2-6)<i%T_CAR_GRID_H_NUM && i%T_CAR_GRID_H_NUM<(T_CAR_GRID_H_NUM/2+6))
        {
            if(isObs(gridCell,i))
            {
                isObstacleE = true;//这里可以直接写成return true，会直接退出循环
                break;//退出for循环
            }
        }
    }
    return isObstacleE;
}


//因为只看中间小车左右六格，前方30格，所以只用考虑第一行的特殊情况
bool isObs(boost::array<uint8_t, 2000> gridCell,int i)
{

    if(gridCell[i]==1) {
        if (i < T_CAR_GRID_H_NUM)
        {
            if ((gridCell[i] + gridCell[i - 1] + gridCell[i + 1] +
            gridCell[i+19] + gridCell[i+20] + gridCell[i + 21]) > 2)
                return true;
        }
        else
            if((gridCell[i] + gridCell[i - 1] + gridCell[i + 1] + gridCell[i - 20] + gridCell[i + 20]
            +gridCell[i+19] + gridCell[i+21] + gridCell[i-19] + gridCell[i-21]) > 2)
                return true;
    }
    else
        return false;

}

//计算距离，以格为单位
int getDistance(boost::array<uint8_t, 2000> gridCell, int i)
{
    int gridX = i/T_CAR_GRID_H_NUM;
    int gridY = i%T_CAR_GRID_H_NUM;
    double gridDis = sqrt((gridX-T_CAR_GRID_H_NUM/2)*(gridX-T_CAR_GRID_H_NUM/2)+gridY*gridY);
    return gridDis;
}



/***********************************************************************************************************
 **                     1118跟点走部分
 **                  全部转到大地坐标下进行
 ***********************************************************************************************************/


//前方5米内如果有障碍物的话，得到障碍物旁边经过的点的栅格下标，转换，返回大地坐标系坐标
//思路：得到左右两边1的数量，1的栅格坐标的1_LEFT_MAX和 1_RIGHT_MAX
//应该是每一帧都返回一个障碍旁边的跟的点
//设置反比速度
int getObsCoor(boost::array<unsigned char, 2000ul> gridCell)
{
    int l_count = 0,r_count = 0,midValue = T_CAR_GRID_H_NUM/2,iLeftMax = midValue,
        iRightMax = midValue,gridSub_l,gridSub_r;//左右的障碍点数统计，车往少的那边绕
    if(obsInFront(gridCell,5))
    {
        for(int i = 0;i<=T_CAR_GRID_H_NUM*10*5;i++)
        {
            if((T_CAR_GRID_H_NUM/2-6)<i%T_CAR_GRID_H_NUM && i%T_CAR_GRID_H_NUM<(T_CAR_GRID_H_NUM/2+6))
            {
                if (gridCell[i] = 1)
                {
                    if (i % T_CAR_GRID_H_NUM < T_CAR_GRID_H_NUM / 2)
                        l_count++;
                    else
                        r_count++;
                    if (i % T_CAR_GRID_H_NUM < iLeftMax)
                    {
                        iLeftMax = i % T_CAR_GRID_H_NUM;
                        gridSub_l = i;
                    }
                    if (i % T_CAR_GRID_H_NUM >= iRightMax)
                    {
                        iRightMax = i % T_CAR_GRID_H_NUM;
                        gridSub_r = i;
                    }
                }
            }
        }
        if(r_count>=l_count)
            return gridSub_r+4;
        else
            return gridSub_l-4;

        //return gridSub;//！！如果gridSub为0或者19,要考虑前方是墙的情况，这时候要右转到一般情况再执行这个函数，返回gridSub
    }
}


/* 栅格下标换算成大地坐标
 * 已知：偏航角theta，车体的大地坐标，栅格下标。算：栅格的大地坐标
 * 假设车体地球坐标是（0,0），不为零时加上偏移量就行。
 */
vector<double > gridToEarthXY(int gridSub, float theta,vector<double > carEarthXY)
{
    int gridX,gridY;
    vector<double > xy(2);
    gridX = gridSub%T_CAR_GRID_H_NUM*10;//乘10把单位换算成米
    gridY = gridSub/T_CAR_GRID_H_NUM*10;
    float gama = atan(gridY/gridX)-theta;//地球坐标系下的极坐标角度
    xy[0]=(sqrt(gridX*gridX+gridY*gridY))*(sin(gama))+carEarthXY[0];
    xy[1]=(sqrt(gridX*gridX+gridY*gridY))*(cos(gama))+carEarthXY[1];
    return xy;
}

// 控制行走函数
// 已知：惯导偏航角，起始点、目标点的earthXY
// 目标：走到目标点，到达返回true


//下面函数需一直循环调用，偏航角theta实时更新
//不可用，仅作思想，需改写进main函数
/**********************************************************************************
bool arrDest(vector<float > startXY(2), vector<float > destXY(2), float theta)
{
    float beta;
    beta = atan((destXY[1]-startXY[1])/(destXY[0]-startXY[0]));
    while ((theta-beta)>0.05)//使航向角与转向目标的角度重合误差在2度左右
        port1.sendWheelSpd(GORIGHT);
    while(sqrt((destXY[1]-startXY[1])*(destXY[1]-startXY[1])+
    (destXY[0]-startXY[0])*(destXY[0]-startXY[0]))>1)//距离目标点一米左右算到达目标
        port1.sendWheelSpd(GOAHEAD);
}
***********************************************************************************/

//一个问题，用师妹的坐标换算函数来换算出来的车的地球坐标与惯导直接传过来的是一样的吗，测试，不一样的话减去偏移量

vector<double> gaussBLtoXY(double longitude,double latitude) {
    int ProjNo = 0;

    // 带宽
    int ZoneWide = 6;

    double longitude1, latitude1, longitude0, X0, Y0, xval, yval;
    double a, f, e2, ee, NN, T, C, A, M, iPI;

    // 3.1415926535898/180.0;
    iPI = 0.0174532925199433;

    // 54年北京坐标系参数
    // a = 6378245.0;
    // f = 1.0 / 298.3;

    // 80年西安坐标系参数
    // a=6378140.0;
    // f=1/298.257;

    // WGS84
    a = 6378137.0;
    f = 1 / 298.257223565;

    ProjNo = (int) (longitude / ZoneWide);
    longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
    longitude0 = longitude0 * iPI;

    // 经度转换为弧度
    longitude1 = longitude * iPI;

    // 纬度转换为弧度
    latitude1 = latitude * iPI;

    e2 = 2 * f - f * f;
    ee = e2 * (1.0 - e2);
    NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
    T = tan(latitude1) * tan(latitude1);
    C = ee * cos(latitude1) * cos(latitude1);
    A = (longitude1 - longitude0) * cos(latitude1);
    M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 -
             (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * latitude1) +
             (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) -
             (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));
    xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
    yval = M + NN * tan(latitude1) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                                      (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
    X0 = 1000000 * (ProjNo + 1) + 500000;
    Y0 = 0;
    xval = xval + X0;
    yval = yval + Y0;

    vector<double> xy(2); //单位m
    xy[0] = xval;
    xy[1] = yval;
    return xy;
}


//读txt文件函数
//输入：txt文件目录，要读入的行





