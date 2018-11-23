//
// Created by peng on 18-11-12.
//

#include <iostream>
#include "serialport.h"
#include "serialport.cpp"
#include "ros/ros.h"
#include "car_plan.h"
#include "car_plan.cpp"
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "carplay_v3/T_Msg_FU_TO_PL.h"
#include "carplay_v3/T_Msg_MC_TO_PL.h"

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

//定义键盘消息的ASCII值
#define KEY_FRONT 119//w
#define KEY_BACK 115//s
#define KEY_LEFT 97//a
#define KEY_RIGHT 100//d
#define KEY_STOP 32//space
#define KEY_MODECHANGE 99//c
#define KEY_RACE 114//r

#define pi 3.14159265

//返回键盘ASCII码值
//！！把这个函数换掉，换成非阻塞式监听（有输入接收输入，没输入pass，执行后面代码）
int scanKeyboardN()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0; tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    in = getchar();
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}

//非阻塞监听
int scanKeyboard()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)// Ctrl + C
    {
        ungetc(ch, stdin);
        return ch;
    }
    return 0;//没有按键的话一直返回0
}


//状态变量，选择是否是自主模式
bool isAutoModeON = false;


typedef struct tagT_FU_TO_PL
{
    int frameID;                                                //帧号
    int syntime;                                                //时间戳
    int navID;                                                  //最近的惯导号
//    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //栅格数组,值0-255，0表示无障碍，非0表示有障碍
    boost::array<uint8_t, 2000> gridMsk;
//    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //数字数组，每行表示该障碍物占据的栅格标号，以-1结束
    boost::array<int8_t, 1000> pObs;
    int nObs;                                                   //障碍物有效数量
} T_FU_TO_PL;

T_FU_TO_PL Sub_Msg;


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
} T_MC_TO_PL;


T_MC_TO_PL Sub_Msg_Nav;

//订阅地图的回调函数
void T_FU_Callback(const carplay_v3::T_Msg_FU_TO_PL::ConstPtr& msg)
{
    Sub_Msg.frameID = msg->frameID;
    Sub_Msg.syntime = msg->syntime;
    Sub_Msg.navID = msg->navID;
    Sub_Msg.gridMsk = msg->gridMsk;
    Sub_Msg.pObs = msg->pObs;
    Sub_Msg.nObs = msg->nObs;
//    memcpy(&Sub_Msg,&msg, sizeof(T_FU_TO_PL));
    std::cout << "I heard: FU "  << Sub_Msg.frameID << std::endl; 
    ROS_INFO("I heard: FU [%d]", msg->frameID);
}


//订阅惯导的回调函数

void T_MC_Callback(const carplay_v3::T_Msg_MC_TO_PL::ConstPtr& msg)
{
    Sub_Msg_Nav.navID = msg->navID;
    Sub_Msg_Nav.syntime = msg->syntime;
    Sub_Msg_Nav.Longitude_degree = msg->coor[0];
    Sub_Msg_Nav.Latitude_degree = msg->coor[1];
    Sub_Msg_Nav.Altitude_m = msg->coor[2];
    Sub_Msg_Nav.EarthRefCoord[0] = msg->coor[3];//测试，两个地球坐标不一致的话在这里加上偏移量
    Sub_Msg_Nav.EarthRefCoord[1] = msg->coor[4];
    Sub_Msg_Nav.Roll_rad = msg->angle[0];
    Sub_Msg_Nav.Pitch_rad = msg->angle[1];
    Sub_Msg_Nav.Yaw_rad = msg->angle[2];
    ROS_INFO("I heard: MC [%d]", msg->navID);

//    T_MC_TO_PL tempMC;
//    memcpy(&g_MC, &(msg->navID), sizeof(T_MC_TO_FU));
//    g_MC.push_front(tempMC);
//    if (g_MC.size() > 300)
//        g_MC.pop_back();
//    ROS_INFO("i heard you MC!Current navID:%d Current size:%d", g_MC.front().navID, g_MC.size());
}




/***********************************************************************************************************
 **                     1118跟点走部分
 **                  全部转到大地坐标下进行
 ***********************************************************************************************************/

//计算直角坐标系两点之间距离函数

double getDistXY(double point1[2], double point2[2])
{
    return (sqrt((point2[1]-point1[1])*(point2[1]-point1[1])+(point2[0]-point1[0])*(point2[0]-point1[0])));
}




/************************************************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "carplay");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("T_FU_TO_PL", 1000, T_FU_Callback);
    ros::Subscriber PL_MC_sub = n.subscribe("T_MC", 1000, T_MC_Callback);//PL_MC_sub名字要不要改





    //打开串口并初始化，可以发送指令
    SerialPort port1;
    if(port1.Open(0,9600) == UART_FALSE)
    {
        cout<<"ERROR:can not open serialpoint!!"<<endl;
        return 0;
    }

/******************************************************************************************************
 * 随机避障算法
 *
 * 先直行，直到遇到障碍物，然后跳出直行的循环，然后进入转弯循环，
 * 转弯循环break条件：前方可行
 * 紧急停车条件：前方0.8米突然出现障碍物，取消紧急状态条件：0.8米内障碍物消失.
 *
 * 上面三块都写在while（1）的大循环里
 *
 * 在每次发送指令前加上状态判断，为真则继续自主，否则取消发指令，退出循环，进入键盘控制函数
 *
 * 在本文件中加上接收按键消息的代码，不用再写节点
 *
 * wsad->上下左右，space->刹车，Enter->切换模式
 * ASCII码：w 119, s 115, a 97, d 100, space 32, enter 10, c 99, v 118
 *
 * 一个情况：键盘接收函数会阻塞  已解决
 *********************************************************************************************************/

//写一个while，里面并列。只用一次scanKeyboard，不然会导致输多次切换键才会切换，用continue跳过剩余部分，switch & continue
    while(1)
    {
        ros::spinOnce();
        std::cout << Sub_Msg.frameID << std::endl;

        /********************************************************************************
         * 键盘控制部分
         *********************************************************************************/

        if (!isAutoModeON) {
            //ros::spinOnce();//循环，接收最新的消息帧
            int keyValue = scanKeyboardN(),len = 0;//这里还是会阻塞键盘控制时需要阻塞，自主模式时在自己的小循环里就行。
            switch (keyValue) {
                case KEY_FRONT:
                    len = port1.sendWheelSpd(GOAHEAD);
                    continue;
                case KEY_RACE:
                    len = port1.sendWheelSpd(RACE);
                    continue;
                case KEY_BACK:
                    len = port1.sendWheelSpd(GOBACK);
                    continue;
                case KEY_LEFT:
                    len = port1.sendWheelSpd(GOLEFT);
                    continue;
                case KEY_RIGHT:
                    len = port1.sendWheelSpd(GORIGHT);
                    continue;
                case KEY_STOP:
                    len = port1.sendWheelSpd(EMERGENCYSTOP);
                    continue;
                case KEY_MODECHANGE:
                    isAutoModeON = true;
                    continue;
                default:
                    continue;
            }
        }





        if (isAutoModeON) {

            /************************  读txt把经纬度存入二维数组  **********************/

            ifstream fin;
            fin.open("/home/peng/catkin_ws/src/carplay_v3/src/GPSPoints.txt");//换机器这里要修改
            double gpsTar[1000][2];
            memset(gpsTar,0,sizeof(double)*2*1000);
            int gpsTar_i=0;
            while(!fin.eof())
            {
                fin >> gpsTar[gpsTar_i][0];
                fin >> gpsTar[gpsTar_i][1];
                gpsTar_i++;
            }
            fin.close();
            printf("Reading GPSPoints.txt completed! gpsTar[3][0] is %.7f\n", gpsTar[3][0]);


            /****************************  开始跟点走规划  *************************/

            //目标点gps到地球，之后只用地球坐标XYTar就行
            double XYTar[1000][2];
            memset(XYTar,0,sizeof(double)*2*1000);
            for(int p =0;p<1000;p++)
            {
                XYTar[p][0]=(gaussBLtoXY(gpsTar[p][0],gpsTar[p][1]))[0];//试一试可不可以这样写 可以
                XYTar[p][1]=(gaussBLtoXY(gpsTar[p][0],gpsTar[p][1]))[1];
            }
            printf("Transform GPS coor to earth coor completed! XYTar[3][0] is %.7f\n", XYTar[3][0]);


            int destId = 0;
            double destPoint[2]= {0,0};
            while(1)//循环完成整个跟点走过程，只有按下切换键或者到达终点，才会break，重新进入键盘控制
            {
                printf("Have reached the %dth point!\n",destId-1);
                //读入第destID个目标点,作为下一个目标点 //dest[i][0]=X, dest[i][1]=Y
                bool reachDest = false;//初始未到达
                destPoint[0]=XYTar[destId][0];
                destPoint[1]=XYTar[destId][1];
                //到达下一个目标点destPoint的过程，分为四部分，紧急刹车、转向对准、检查障碍，直行，
                // 有障碍的话经过障碍旁边点然后continue跳回开始的转弯对准部分
                while(1)
                {
                    ros::spinOnce();
                    //0、检查是否需要紧急刹车
                    if (isEmergency(Sub_Msg.gridMsk)) {
                        while (1) {
                            ros::spinOnce();
                            int len = port1.sendWheelSpd(EMERGENCYSTOP);
                            if (len == 4) {
                                printf("In Emergency!\r\n");
                            } else
                                printf("send error\r\n");

                            usleep(200000);

                            if (!isEmergency(Sub_Msg.gridMsk))
                                break;
                            if (scanKeyboard() == KEY_RACE) {
                                isAutoModeON = false;
                                break;
                            }
                        }
                        if (!isAutoModeON)
                            break;
                    }

                    //1、转弯，对准目标点
                    while(1)
                    {
                        //首先检测车体是否已经到达目标点，到达就break，读下一个目标点
                        if(getDistXY(destPoint,Sub_Msg_Nav.EarthRefCoord)<0.5)
                        {
                            reachDest = true;
                            break;
                        }
                        ros::spinOnce();
                        //测试惯导传回的大地坐标与函数换算的大地坐标是否一致
                        //ros::spinOnce();
                        vector<double> calcuEarthCoor(2);
                        calcuEarthCoor[0]= (gaussBLtoXY(Sub_Msg_Nav.Longitude_degree,Sub_Msg_Nav.Latitude_degree))[0];
                        calcuEarthCoor[1]= (gaussBLtoXY(Sub_Msg_Nav.Longitude_degree,Sub_Msg_Nav.Latitude_degree))[1];

                        printf("calculate earthX is %.7f\n",calcuEarthCoor[0]);
                        printf("calculate earthY is %.7f\n",calcuEarthCoor[1]);
                        printf("IMU's earthX is %.7f\n",Sub_Msg_Nav.EarthRefCoord[0]);
                        printf("IMU's earthY is %.7f\n",Sub_Msg_Nav.EarthRefCoord[1]);

                        //计算起始点与坐标点之间夹角
                        float angleXY =(pi/2 - atan((destPoint[1]-Sub_Msg_Nav.EarthRefCoord[1])
                                /(destPoint[0]-Sub_Msg_Nav.EarthRefCoord[0])));
                        if((angleXY-Sub_Msg_Nav.Yaw_rad)<0.1 && (angleXY-Sub_Msg_Nav.Yaw_rad)>-0.1)//0.2弧度大概是角度10度左右
                            break;
                        if(angleXY-Sub_Msg_Nav.Yaw_rad<-0.1)
                        {
                            int len = port1.sendWheelSpd(GORIGHT);
                            if (len == 4) {
                                printf("Initial Searching Target!\r\n");
                            } else
                                printf("send error\r\n");
                            usleep(200000);
                        }
                        if(angleXY-Sub_Msg_Nav.Yaw_rad>0.1)
                        {
                            int len = port1.sendWheelSpd(GOLEFT);
                            if (len == 4) {
                                printf("Initial Searching Target!\r\n");
                            } else
                                printf("send error\r\n");
                            usleep(200000);
                        }
                        if (scanKeyboard() == KEY_RACE) {
                            isAutoModeON = false;
                            break;
                        }
                    }
                    if (!isAutoModeON)
                        break;


                    //2、检查前方8m障碍物，有障碍物的话走到临时目标点，然后continue
                    if(obsInFront(Sub_Msg.gridMsk,8))
                    {
                        int gridSub = getObsCoor(Sub_Msg.gridMsk);
                        bool reachObsPoint = false;
                        bool angleError = false;
                        //墙型障碍
                        if(gridSub%T_CAR_GRID_H_NUM == 19 || gridSub%T_CAR_GRID_H_NUM == 0)
                        {
                            //转弯，直到变成石头型障碍，然后调函数找到经过点;
                            while(1)
                            {
                                ros::spinOnce();
                                int len = port1.sendWheelSpd(GORIGHT);
                                if (len == 4) {
                                    printf("Avoiding the Wall-like obstacle!\r\n");
                                } else
                                    printf("send error\r\n");
                                if(getObsCoor(Sub_Msg.gridMsk)<17)
                                {
                                    gridSub = getObsCoor(Sub_Msg.gridMsk);
                                    break;
                                }
                                usleep(200000);
                                if (scanKeyboard() == KEY_RACE) {
                                    isAutoModeON = false;
                                    break;
                                }
                            }
                            if (!isAutoModeON)
                                break;
                        }

                        //得到障碍物旁边经过点的地球坐标
                        vector<double > carEarthCoor(begin(Sub_Msg_Nav.EarthRefCoord),
                                end(Sub_Msg_Nav.EarthRefCoord));
                        vector<double> gridEarthCoor(2);
                        gridEarthCoor = gridToEarthXY(gridSub,Sub_Msg_Nav.Yaw_rad,carEarthCoor);
                        double obsPoint[2]={gridEarthCoor[0],gridEarthCoor[1]};


                        //下面是不考虑障碍版的跟点走算法，包括转弯对准目标点和直行直到目标点两部分
                        //目的是走到障碍旁边点
                        while(1)
                        {
                            while(1)
                            {
                                ros::spinOnce();
                                float angleXYObs = (pi/2 - atan((obsPoint[1]-Sub_Msg_Nav.EarthRefCoord[1])
                                                     /(obsPoint[0]-Sub_Msg_Nav.EarthRefCoord[0])));
                                if(angleXYObs-Sub_Msg_Nav.Yaw_rad>-0.1 && angleXYObs-Sub_Msg_Nav.Yaw_rad<0.1)
                                    break;
                                if(angleXYObs-Sub_Msg_Nav.Yaw_rad<-0.1)
                                {
                                    int len = port1.sendWheelSpd(GORIGHT);
                                    if (len == 4) {
                                        printf("Searching Target!\r\n");
                                    } else
                                        printf("send error\r\n");
                                    usleep(200000);
                                }
                                if(angleXYObs-Sub_Msg_Nav.Yaw_rad>0.1)
                                {
                                    int len = port1.sendWheelSpd(GOLEFT);
                                    if (len == 4) {
                                        printf("Searching Target!\r\n");
                                    } else
                                        printf("send error\r\n");
                                    usleep(200000);
                                }
                                if (scanKeyboard() == KEY_RACE) {
                                    isAutoModeON = false;
                                    break;
                                }
                                if (!isAutoModeON)
                                    break;
                            }
                            if (!isAutoModeON)
                                break;
                            while(1)
                            {
                                //前进过程中注意检测角度是不是对准，如果没对准，continue
                                ros::spinOnce();
                                double XYObsDist = getDistXY(Sub_Msg_Nav.EarthRefCoord, obsPoint);
                                if(XYObsDist<0.5)//距目标点0.5米的范围算作到达目标点
                                {
                                    reachObsPoint = true;
                                    break;
                                }
                                int len = port1.sendWheelSpd(GOAHEAD);
                                if (len == 4) {
                                    printf("Going to point beside the Obstacle!\r\n");
                                } else
                                    printf("send error\r\n");
                                usleep(200000);
                                float angleXYObs = (pi/2 - atan((obsPoint[1]-Sub_Msg_Nav.EarthRefCoord[1])
                                                        /(obsPoint[0]-Sub_Msg_Nav.EarthRefCoord[0])));
                                if(angleXYObs - Sub_Msg_Nav.Yaw_rad>0.1 || angleXYObs - Sub_Msg_Nav.Yaw_rad<-0.1)
                                {
                                    //angleError = true;//前进过程停止，进入转弯循环，对正了再直行
                                    break;
                                }
                                if (scanKeyboard() == KEY_RACE) {
                                    isAutoModeON = false;
                                    break;
                                }
                            }
                            if (!isAutoModeON)
                                break;
                            if(reachObsPoint)
                                break;
                        }
                        if (!isAutoModeON)
                            break;
                        if(reachObsPoint)
                            continue;

                    }
                    if (!isAutoModeON)
                        break;

                    //3、直行,并每次执行动作前都要检测角度是否对准
                    while(1)
                    {
                        ros::spinOnce();
                        int len = port1.sendWheelSpd(GOAHEAD);
                        if (len == 4) {
                            printf("Going to next point!\r\n");
                        } else
                            printf("send error\r\n");
                        usleep(200000);
                        if(obsInFront(Sub_Msg.gridMsk, 5))
                        {
                            break;
                        }
                        float angleXY = (pi/2 - atan((destPoint[1]-Sub_Msg_Nav.EarthRefCoord[1])
                                             /(destPoint[0]-Sub_Msg_Nav.EarthRefCoord[0])));
                        if(angleXY-Sub_Msg_Nav.Yaw_rad>0.1 || angleXY-Sub_Msg_Nav.Yaw_rad<-0.1)
                            break;

                        if(getDistXY(destPoint,Sub_Msg_Nav.EarthRefCoord)<0.5)
                        {
                            reachDest = true;
                            break;
                        }
                    }

                    if(reachDest)//跳出次大循环唯一条件，到达读入的第i个目标点
                    {
                        //destId++;
                        break;
                    }
                }
                if (!isAutoModeON)
                    break;    //跳回键盘控制

                destId+=20;//设置gps点之间距离大概4米
                if(destId == 1000)//跟点走200米，停，跳回键盘控制
                    break;
            }
            printf("Successful Arrival!");




            /***********************************************************************************************
             *
             ****** 随机避障部分 *****************



            while (1) {
                if (isEmergency(Sub_Msg.gridMsk)) {
                    while (1) {
                        ros::spinOnce();
                        int len = port1.sendWheelSpd(EMERGENCYSTOP);
                        if (len == 4) {
                            printf("In Emergency!\r\n");
                        } else
                            printf("send error\r\n");

                        usleep(200000);

                        if (!isEmergency(Sub_Msg.gridMsk))
                            break;
                        if (scanKeyboard() == KEY_RACE) {
                            isAutoModeON = false;
                            break;
                         }
                    }
                    if (!isAutoModeON)//不进入上面的紧急情况判断，这里肯定不会break
                        break;
                }
                while (1) {
                    ros::spinOnce();
                    int len = port1.sendWheelSpd(GOAHEAD);
                    usleep(200000);
                    if (len == 4) {
                        printf("Auto Going Ahead!\r\n");
                    }
                    else
                        printf("send ahead cmd error\r\n");

                    if (obsInFront(Sub_Msg.gridMsk,3))
                        break;


                    if (scanKeyboard() == KEY_RACE) {
                        isAutoModeON = false;
                        break;
                    }
                }
                if (!isAutoModeON)
                    break;

                while (1) {
                    ros::spinOnce();
                    int len = port1.sendWheelSpd(GORIGHT);
                    usleep(200000);
                    if (len == 4) {
                        printf("Auto Going Right!\r\n");
                    } else
                        printf("send right cmd error\r\n");
                    if (!obsInFront(Sub_Msg.gridMsk,3))
                        break;
                    if (scanKeyboard() == KEY_RACE) {
                        isAutoModeON = false;
                        break;
                    }
                }

                if (!isAutoModeON)
                    break;
            }

            *****************************************************************************************/
        }

    }
    //串口不关
    //port1.Close();
    return 0;

}
