//
// Created by peng on 18-11-12.
//

#include "car_plan.h"
#include "serialport.h"


#include "carplay_v3/T_Msg_FU_TO_PL.h"



/************************************************************************************************************
 *
 * ����ߡ����̿��Ʋ���
 *
 ************************************************************************************************************/



//�ж�ǰ��i_meters��(30��)���Ƿ����ϰ���
bool obsInFront(boost::array<uint8_t, 2000> gridCell,int i_meters)
{
    bool isObstacle = false;//Ĭ��ǰ�����ϰ�
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

//ǰ��0.8�����Ƿ����ϰ���
bool isEmergency(boost::array<uint8_t, 2000> gridCell)
{
    bool isObstacleE = false;
    for(int i = 0;i<T_CAR_GRID_H_NUM*10;i++)
    {
        if((T_CAR_GRID_H_NUM/2-6)<i%T_CAR_GRID_H_NUM && i%T_CAR_GRID_H_NUM<(T_CAR_GRID_H_NUM/2+6))
        {
            if(isObs(gridCell,i))
            {
                isObstacleE = true;//�������ֱ��д��return true����ֱ���˳�ѭ��
                break;//�˳�forѭ��
            }
        }
    }
    return isObstacleE;
}


//��Ϊֻ���м�С����������ǰ��30������ֻ�ÿ��ǵ�һ�е��������
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

//������룬�Ը�Ϊ��λ
int getDistance(boost::array<uint8_t, 2000> gridCell, int i)
{
    int gridX = i/T_CAR_GRID_H_NUM;
    int gridY = i%T_CAR_GRID_H_NUM;
    double gridDis = sqrt((gridX-T_CAR_GRID_H_NUM/2)*(gridX-T_CAR_GRID_H_NUM/2)+gridY*gridY);
    return gridDis;
}



/***********************************************************************************************************
 **                     1118�����߲���
 **                  ȫ��ת����������½���
 ***********************************************************************************************************/


//ǰ��5����������ϰ���Ļ����õ��ϰ����Ա߾����ĵ��դ���±꣬ת�������ش������ϵ����
//˼·���õ���������1��������1��դ�������1_LEFT_MAX�� 1_RIGHT_MAX
//Ӧ����ÿһ֡������һ���ϰ��Աߵĸ��ĵ�
//���÷����ٶ�
int getObsCoor(boost::array<unsigned char, 2000ul> gridCell)
{
    int l_count = 0,r_count = 0,midValue = T_CAR_GRID_H_NUM/2,iLeftMax = midValue,
        iRightMax = midValue,gridSub_l,gridSub_r;//���ҵ��ϰ�����ͳ�ƣ������ٵ��Ǳ���
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

        //return gridSub;//�������gridSubΪ0����19,Ҫ����ǰ����ǽ���������ʱ��Ҫ��ת��һ�������ִ���������������gridSub
    }
}


/* դ���±껻��ɴ������
 * ��֪��ƫ����theta������Ĵ�����꣬դ���±ꡣ�㣺դ��Ĵ������
 * ���賵����������ǣ�0,0������Ϊ��ʱ����ƫ�������С�
 */
vector<double > gridToEarthXY(int gridSub, float theta,vector<double > carEarthXY)
{
    int gridX,gridY;
    vector<double > xy(2);
    gridX = gridSub%T_CAR_GRID_H_NUM*10;//��10�ѵ�λ�������
    gridY = gridSub/T_CAR_GRID_H_NUM*10;
    float gama = atan(gridY/gridX)-theta;//��������ϵ�µļ�����Ƕ�
    xy[0]=(sqrt(gridX*gridX+gridY*gridY))*(sin(gama))+carEarthXY[0];
    xy[1]=(sqrt(gridX*gridX+gridY*gridY))*(cos(gama))+carEarthXY[1];
    return xy;
}

// �������ߺ���
// ��֪���ߵ�ƫ���ǣ���ʼ�㡢Ŀ����earthXY
// Ŀ�꣺�ߵ�Ŀ��㣬���ﷵ��true


//���溯����һֱѭ�����ã�ƫ����thetaʵʱ����
//�����ã�����˼�룬���д��main����
/**********************************************************************************
bool arrDest(vector<float > startXY(2), vector<float > destXY(2), float theta)
{
    float beta;
    beta = atan((destXY[1]-startXY[1])/(destXY[0]-startXY[0]));
    while ((theta-beta)>0.05)//ʹ�������ת��Ŀ��ĽǶ��غ������2������
        port1.sendWheelSpd(GORIGHT);
    while(sqrt((destXY[1]-startXY[1])*(destXY[1]-startXY[1])+
    (destXY[0]-startXY[0])*(destXY[0]-startXY[0]))>1)//����Ŀ���һ�������㵽��Ŀ��
        port1.sendWheelSpd(GOAHEAD);
}
***********************************************************************************/

//һ�����⣬��ʦ�õ����껻�㺯������������ĳ��ĵ���������ߵ�ֱ�Ӵ���������һ�����𣬲��ԣ���һ���Ļ���ȥƫ����

vector<double> gaussBLtoXY(double longitude,double latitude) {
    int ProjNo = 0;

    // ����
    int ZoneWide = 6;

    double longitude1, latitude1, longitude0, X0, Y0, xval, yval;
    double a, f, e2, ee, NN, T, C, A, M, iPI;

    // 3.1415926535898/180.0;
    iPI = 0.0174532925199433;

    // 54�걱������ϵ����
    // a = 6378245.0;
    // f = 1.0 / 298.3;

    // 80����������ϵ����
    // a=6378140.0;
    // f=1/298.257;

    // WGS84
    a = 6378137.0;
    f = 1 / 298.257223565;

    ProjNo = (int) (longitude / ZoneWide);
    longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
    longitude0 = longitude0 * iPI;

    // ����ת��Ϊ����
    longitude1 = longitude * iPI;

    // γ��ת��Ϊ����
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

    vector<double> xy(2); //��λm
    xy[0] = xval;
    xy[1] = yval;
    return xy;
}


//��txt�ļ�����
//���룺txt�ļ�Ŀ¼��Ҫ�������





