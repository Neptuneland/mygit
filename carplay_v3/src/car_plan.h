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


#define T_CAR_GRID_DISTANCE_V_CM 1000 //����դ���ͼ��ֱ(vertical)���룬��ǰ��1000,��0,��λcm
#define T_CAR_GRID_DISTANCE_H_CM 200  //����դ���ͼˮƽ(Horizontal)���룬�����Ҹ�100,��λcm
#define T_CAR_GRID_RESOLUTION_V_CM 10 //����դ���ͼ��ֱ�ֱ��ʣ���λcm
#define T_CAR_GRID_RESOLUTION_H_CM 10 //����դ���ͼˮƽ�ֱ��ʣ���λcm
#define T_CAR_GRID_V_NUM 100          //����դ���ͼ��ֱ����դ����
#define T_CAR_GRID_H_NUM 20           //����դ���ͼˮƽ����դ����



/*
 * �����е����ݣ�gridCell[2000]��1Ϊ�ϰ���ע�⣺��10Hzʵʱˢ�µģ�Ҫ���ݴ���������Ϲ滮�����������¥����
 * 1����դ��ṹ��
 * 2��ת�򣬸����ܹ�ǰ����λ�������ĵ�ľ������÷����ٶȣ�Ϊ��ʱ�˳�while(1)ѭ�������н�ģʽ��
 * 3���н������ð�ȫ��ʻ���룬�������ϰ�����뷴�ȵ��ٶȣ�
 */

/* gridCell[i]���ڵ��У�i/20���У�i%20;
 * �ж��ϰ��if(girdCell[i]=1,����(gC[i-1]+gC[i]+gC[i+1]+gC[i-20]+gC[i+20])>2�Ƿ����
 * ������룺�õ�gC[i]��i��int x=i%20, int y =i/20, dis = sqrt(x*x+ y*y)
 * �ٶȱ仯0-127,v=127*()
 * �滮���������ң����ֻҪһ��դ�����ϰ������ұ����񣨻����ұ����񣩼������Ƿ�>=1����������������ɨ
 *
 *
 * �Ⱥ㶨�����ܣ���Ͻ���ɲ�����趨�ٶ���˵
 *
 * ����ɲ����ͨ�������жϣ�С����ֵɲ��
 *
 * ���ӹ��ܣ�����һ��״̬���������ڿ��������ͷ�����������ָ��ǰ���ñ����Ƿ�Ϊ�棬��Ϊ��ȡ���������ơ�
 *         ����������ʱֱ�ӷ����ֿ���ָ�ǰ������ͣ��
 */


//ǰ���ϰ����жϺ�����i_meters�����Ƿ����ϰ���

bool obsInFront(boost::array<unsigned char, 2000ul> gridCell, int i_meters);

//ǰ��1�����Ƿ����ϰ����еĻ�����ɲ��
bool isEmergency(boost::array<unsigned char, 2000ul> gridCell);


//�ж��ϰ��if(girdCell[i]=1,����(gC[i-1]+gC[i]+gC[i+1]+gC[i-20]+gC[i+20])>2�Ƿ����,ע��߽Ǵ����ܼӼ����е��������
//��main���������¸������������Ϣ��ֵ
bool isObs(boost::array<unsigned char, 2000ul> gridCell,int i);


//������뺯��
//������룺�õ�gC[i]��i��int x=i%20, int y =i/20, dis = sqrt(x*x+ y*y)

int getDistance(boost::array<unsigned char, 2000ul> gridCell, int i);

//���ʮ�����ƣ�cout << "35��16����:" << std::hex << 35 << endl;
int getCarSpeed(boost::array<unsigned char, 2000ul> gridCell, int i);//set speed depend with distance,





/***********************************************************************************************************
 **                     1118�����߲���
 **                  ȫ��ת����������½���
 ***********************************************************************************************************/


//����nav_mc����Ϣֱ�ӵõ���������
//typedef struct tagT_MC_TO_FU
//{
//    int navID;
//    int syntime;
//    double Longitude_degree; //γ�ȡ���λ:��
//    double Latitude_degree;  //���ȡ���λ:��
//    double Altitude_m;       //���Ρ���λ:��
//    double EarthRefCoord[2]; // ����ƽ������ϵ ��λ:��  [0] ���� +X     [1] ���� +Y
//
//    //��ڽǡ������ǡ������
//    float Roll_rad;  //��λ:����
//    float Pitch_rad; //��λ:����
//    float Yaw_rad;   //��λ:����
//} T_MC_TO_FU;


//����ߵ���Ϣ����ʱֻ��ƫ����
//������ϰ���õ������ϰ���ı�Ե�ĵ�����꣬������һ�����ĵ��դ����±�

int getObsCoor(boost::array<unsigned char, 2000ul> gridCell);

vector<double > gridToEarthXY(int gridSub, float theta,vector<double > carEarthXY);

//bool arrDest(vector<float > startXY(2), vector<float > destXY(2), float theta)

vector<double> gaussBLtoXY(double longitude,double latitude);


#endif




