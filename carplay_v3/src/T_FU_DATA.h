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
//  ����С����Ŀ���Ͼ�����ѧ�������ѧ�빤��ѧԺ
//  FileName:  T_FU_DATA.h
//  Author: ������
//  Date:   2018.7.3
//  Description: �ں�ģ������ݶ���
//  ��ote:���ȿ��Ǿ�̬�ϰ���,����դ���ͼ�������ϰ���ռ�ݵ�դ���б�
//
////////////////////////////////////////////////////////////////

#ifndef _T_FU_DATA_H
#define _T_FU_DATA_H

#include <deque>
using namespace std;

#define T_3D16_OBS_MAX_POINT_NUM 4    //ÿ���ϰ��������T_OBS_MAX_POINT��������
#define T_3D16_OBS_MAX_GRID_NUM 100   //ÿ���ϰ������ռ�ݶ��ٸ�դ��
#define T_3D16_OBS_MAX_NUM 10         //ÿ֡����ϰ�������
#define T_CAR_GRID_DISTANCE_V_CM 1000 //����դ���ͼ��ֱ(vertical)���룬��ǰ��1000,��0,��λcm
#define T_CAR_GRID_DISTANCE_H_CM 200  //����դ���ͼˮƽ(Horizontal)���룬�����Ҹ�100,��λcm
#define T_CAR_GRID_RESOLUTION_V_CM 10 //����դ���ͼ��ֱ�ֱ��ʣ���λcm
#define T_CAR_GRID_RESOLUTION_H_CM 10 //����դ���ͼˮƽ�ֱ��ʣ���λcm
#define T_CAR_GRID_V_NUM 100          //����դ���ͼ��ֱ����դ����
#define T_CAR_GRID_H_NUM 20           //����դ���ͼˮƽ����դ����


/*3D16�ϰ���*/
//2D������
struct T_POINT_2D
{
    int x_cm; //��ǰ��������ת90��Ϊ�������򣬵�λcm
    int y_cm; //��ǰ������Ϊ��������,��λcm
};

//(��̬)�ϰ���ṹ�壬��͹�ϰ����ϰ�ˮ��ʯͷ���ϰ������ĵ�������
typedef struct tag_T_3D16_OBS_DATA
{
    int OBS_ID; //�ϰ���ID
    T_POINT_2D pPoint[T_3D16_OBS_MAX_POINT_NUM];
    /* �ĸ�������һ���ϰ���
     *pPoint[0]:���ϰ�������ߵ������
     *pPoint[1]:���ϰ������ұߵ������
     *pPoint[2]:������Ϊ[0]��[1]���е㣬������Ϊֱ��x=(pPoint[0].x_cm+pPoint[1].x_cm)/2�������ͷ���ϰ����������,�೵ͷ�����ɳ�Ϊ��ǰ���ĵ㡱
     *pPoint[3]:�ɳ�Ϊ"�󷽵ĵ�"���೵ͷԶ�����������Զ�����ɣ������ᵽ��õ�
    */
    int nPoint; //�����ϰ����ʵ����Ч����
} T_3D16_OBS_DATA;

//3D���ںϵ�һ֡(��̬)�ϰ�������
typedef struct tagT_3D16_OBS_TO_FU
{
    int frameID;                              //֡ID(��0��ʼ)
    int syntime;                              //ʱ���
    int navID;                                //��ͼ���ȡʱ����ӽ��ĵ������ݱ��(��0��ʼ)
    T_3D16_OBS_DATA pObs[T_3D16_OBS_MAX_NUM]; //�ϰ�������
    int nObs;                                 //�ϰ�����Ч����
} T_3D16_OBS_TO_FU;

/*դ���ͼ,���ںϵĺ��ںϷ���ȥ���к�����*/
//ÿһ֡դ������
typedef struct tagT_3D16_GRID_TO_FU
{
    int frameID;                                                //֡��
    int syntime;                                                //ʱ���
    int navID;                                                  //����Ĺߵ���
    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //դ������,ֵ0-255��0��ʾ���ϰ�����0��ʾ���ϰ�
    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //�������飬ÿ�б�ʾ���ϰ���ռ�ݵ�դ���ţ���-1����
    int nObs;                                                   //�ϰ�����Ч����
} T_3D16_GRID_TO_FU;

typedef struct tagT_FU_TO_PL
{
    int frameID;                                                //֡��
    int syntime;                                                //ʱ���
    int navID;                                                  //����Ĺߵ���
    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //դ������,ֵ0-255��0��ʾ���ϰ�����0��ʾ���ϰ�
    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //�������飬ÿ�б�ʾ���ϰ���ռ�ݵ�դ���ţ���-1����
    int nObs;                                                   //�ϰ�����Ч����
} T_FU_TO_PL;

/*T_MC.h*/
typedef struct tagT_MC_TO_FU
{
    int navID;
    int syntime;
    double Longitude_degree; //γ�ȡ���λ:��
    double Latitude_degree;  //���ȡ���λ:��
    double Altitude_m;       //���Ρ���λ:��
    double EarthRefCoord[2]; // ����ƽ������ϵ ��λ:��  [0] ���� +X     [1] ���� +Y

    //��ڽǡ������ǡ������
    float Roll_rad;  //��λ:����
    float Pitch_rad; //��λ:����
    float Yaw_rad;   //��λ:����
} T_MC_TO_FU;

#endif

/* T_MC.h
 * �ṹ���а���֡�š�λ�ˡ��ٶȡ�GPS�������Ϣ��
 * ����һ����������ʷ֡��Ϣ����50֡������std::deque;
 * ���ݹߵ��Ŵ���ʷ��Ϣ�в��ҳ���Ӧ֡��Ϣ�ĺ�����
 *
 * T_ESR.h
 * Ϊ3D16�ṩλ�òο�����FU��ֱ��ͨ�ţ����ж��塣
 *
 * T_2D.h������Ŀ˫Ŀ��
 * ���FU��ʱ�ò��������ж��塣
 *  * ������
 * ʱ�����λ�ȡ�����͵�ÿһ֡���ݶ�Ҫ��ʱ�����Ϣ
*/


#endif //CARPLAY_V3_T_FU_DATA_H
