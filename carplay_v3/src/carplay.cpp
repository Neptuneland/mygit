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


#define T_3D16_OBS_MAX_POINT_NUM 4    //ÿ���ϰ��������T_OBS_MAX_POINT��������
#define T_3D16_OBS_MAX_GRID_NUM 100   //ÿ���ϰ������ռ�ݶ��ٸ�դ��
#define T_3D16_OBS_MAX_NUM 10         //ÿ֡����ϰ�������
#define T_CAR_GRID_DISTANCE_V_CM 1000 //����դ���ͼ��ֱ(vertical)���룬��ǰ��1000,��0,��λcm
#define T_CAR_GRID_DISTANCE_H_CM 200  //����դ���ͼˮƽ(Horizontal)���룬�����Ҹ�100,��λcm
#define T_CAR_GRID_RESOLUTION_V_CM 10 //����դ���ͼ��ֱ�ֱ��ʣ���λcm
#define T_CAR_GRID_RESOLUTION_H_CM 10 //����դ���ͼˮƽ�ֱ��ʣ���λcm
#define T_CAR_GRID_V_NUM 100          //����դ���ͼ��ֱ����դ����
#define T_CAR_GRID_H_NUM 20           //����դ���ͼˮƽ����դ����

//���������Ϣ��ASCIIֵ
#define KEY_FRONT 119//w
#define KEY_BACK 115//s
#define KEY_LEFT 97//a
#define KEY_RIGHT 100//d
#define KEY_STOP 32//space
#define KEY_MODECHANGE 99//c
#define KEY_RACE 114//r

#define pi 3.14159265

//���ؼ���ASCII��ֵ
//����������������������ɷ�����ʽ������������������룬û����pass��ִ�к�����룩
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

//����������
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
    return 0;//û�а����Ļ�һֱ����0
}


//״̬������ѡ���Ƿ�������ģʽ
bool isAutoModeON = false;


typedef struct tagT_FU_TO_PL
{
    int frameID;                                                //֡��
    int syntime;                                                //ʱ���
    int navID;                                                  //����Ĺߵ���
//    unsigned char gridMsk[T_CAR_GRID_V_NUM * T_CAR_GRID_H_NUM]; //դ������,ֵ0-255��0��ʾ���ϰ�����0��ʾ���ϰ�
    boost::array<uint8_t, 2000> gridMsk;
//    int pObs[T_3D16_OBS_MAX_NUM][T_3D16_OBS_MAX_GRID_NUM];      //�������飬ÿ�б�ʾ���ϰ���ռ�ݵ�դ���ţ���-1����
    boost::array<int8_t, 1000> pObs;
    int nObs;                                                   //�ϰ�����Ч����
} T_FU_TO_PL;

T_FU_TO_PL Sub_Msg;


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
} T_MC_TO_PL;


T_MC_TO_PL Sub_Msg_Nav;

//���ĵ�ͼ�Ļص�����
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


//���Ĺߵ��Ļص�����

void T_MC_Callback(const carplay_v3::T_Msg_MC_TO_PL::ConstPtr& msg)
{
    Sub_Msg_Nav.navID = msg->navID;
    Sub_Msg_Nav.syntime = msg->syntime;
    Sub_Msg_Nav.Longitude_degree = msg->coor[0];
    Sub_Msg_Nav.Latitude_degree = msg->coor[1];
    Sub_Msg_Nav.Altitude_m = msg->coor[2];
    Sub_Msg_Nav.EarthRefCoord[0] = msg->coor[3];//���ԣ������������겻һ�µĻ����������ƫ����
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
 **                     1118�����߲���
 **                  ȫ��ת����������½���
 ***********************************************************************************************************/

//����ֱ������ϵ����֮����뺯��

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
    ros::Subscriber PL_MC_sub = n.subscribe("T_MC", 1000, T_MC_Callback);//PL_MC_sub����Ҫ��Ҫ��





    //�򿪴��ڲ���ʼ�������Է���ָ��
    SerialPort port1;
    if(port1.Open(0,9600) == UART_FALSE)
    {
        cout<<"ERROR:can not open serialpoint!!"<<endl;
        return 0;
    }

/******************************************************************************************************
 * ��������㷨
 *
 * ��ֱ�У�ֱ�������ϰ��Ȼ������ֱ�е�ѭ����Ȼ�����ת��ѭ����
 * ת��ѭ��break������ǰ������
 * ����ͣ��������ǰ��0.8��ͻȻ�����ϰ��ȡ������״̬������0.8�����ϰ�����ʧ.
 *
 * �������鶼д��while��1���Ĵ�ѭ����
 *
 * ��ÿ�η���ָ��ǰ����״̬�жϣ�Ϊ�����������������ȡ����ָ��˳�ѭ����������̿��ƺ���
 *
 * �ڱ��ļ��м��Ͻ��հ�����Ϣ�Ĵ��룬������д�ڵ�
 *
 * wsad->�������ң�space->ɲ����Enter->�л�ģʽ
 * ASCII�룺w 119, s 115, a 97, d 100, space 32, enter 10, c 99, v 118
 *
 * һ����������̽��պ���������  �ѽ��
 *********************************************************************************************************/

//дһ��while�����沢�С�ֻ��һ��scanKeyboard����Ȼ�ᵼ�������л����Ż��л�����continue����ʣ�ಿ�֣�switch & continue
    while(1)
    {
        ros::spinOnce();
        std::cout << Sub_Msg.frameID << std::endl;

        /********************************************************************************
         * ���̿��Ʋ���
         *********************************************************************************/

        if (!isAutoModeON) {
            //ros::spinOnce();//ѭ�����������µ���Ϣ֡
            int keyValue = scanKeyboardN(),len = 0;//���ﻹ�ǻ��������̿���ʱ��Ҫ����������ģʽʱ���Լ���Сѭ������С�
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

            /************************  ��txt�Ѿ�γ�ȴ����ά����  **********************/

            ifstream fin;
            fin.open("/home/peng/catkin_ws/src/carplay_v3/src/GPSPoints.txt");//����������Ҫ�޸�
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


            /****************************  ��ʼ�����߹滮  *************************/

            //Ŀ���gps������֮��ֻ�õ�������XYTar����
            double XYTar[1000][2];
            memset(XYTar,0,sizeof(double)*2*1000);
            for(int p =0;p<1000;p++)
            {
                XYTar[p][0]=(gaussBLtoXY(gpsTar[p][0],gpsTar[p][1]))[0];//��һ�Կɲ���������д ����
                XYTar[p][1]=(gaussBLtoXY(gpsTar[p][0],gpsTar[p][1]))[1];
            }
            printf("Transform GPS coor to earth coor completed! XYTar[3][0] is %.7f\n", XYTar[3][0]);


            int destId = 0;
            double destPoint[2]= {0,0};
            while(1)//ѭ��������������߹��̣�ֻ�а����л������ߵ����յ㣬�Ż�break�����½�����̿���
            {
                printf("Have reached the %dth point!\n",destId-1);
                //�����destID��Ŀ���,��Ϊ��һ��Ŀ��� //dest[i][0]=X, dest[i][1]=Y
                bool reachDest = false;//��ʼδ����
                destPoint[0]=XYTar[destId][0];
                destPoint[1]=XYTar[destId][1];
                //������һ��Ŀ���destPoint�Ĺ��̣���Ϊ�Ĳ��֣�����ɲ����ת���׼������ϰ���ֱ�У�
                // ���ϰ��Ļ������ϰ��Աߵ�Ȼ��continue���ؿ�ʼ��ת���׼����
                while(1)
                {
                    ros::spinOnce();
                    //0������Ƿ���Ҫ����ɲ��
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

                    //1��ת�䣬��׼Ŀ���
                    while(1)
                    {
                        //���ȼ�⳵���Ƿ��Ѿ�����Ŀ��㣬�����break������һ��Ŀ���
                        if(getDistXY(destPoint,Sub_Msg_Nav.EarthRefCoord)<0.5)
                        {
                            reachDest = true;
                            break;
                        }
                        ros::spinOnce();
                        //���Թߵ����صĴ�������뺯������Ĵ�������Ƿ�һ��
                        //ros::spinOnce();
                        vector<double> calcuEarthCoor(2);
                        calcuEarthCoor[0]= (gaussBLtoXY(Sub_Msg_Nav.Longitude_degree,Sub_Msg_Nav.Latitude_degree))[0];
                        calcuEarthCoor[1]= (gaussBLtoXY(Sub_Msg_Nav.Longitude_degree,Sub_Msg_Nav.Latitude_degree))[1];

                        printf("calculate earthX is %.7f\n",calcuEarthCoor[0]);
                        printf("calculate earthY is %.7f\n",calcuEarthCoor[1]);
                        printf("IMU's earthX is %.7f\n",Sub_Msg_Nav.EarthRefCoord[0]);
                        printf("IMU's earthY is %.7f\n",Sub_Msg_Nav.EarthRefCoord[1]);

                        //������ʼ���������֮��н�
                        float angleXY =(pi/2 - atan((destPoint[1]-Sub_Msg_Nav.EarthRefCoord[1])
                                /(destPoint[0]-Sub_Msg_Nav.EarthRefCoord[0])));
                        if((angleXY-Sub_Msg_Nav.Yaw_rad)<0.1 && (angleXY-Sub_Msg_Nav.Yaw_rad)>-0.1)//0.2���ȴ���ǽǶ�10������
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


                    //2�����ǰ��8m�ϰ�����ϰ���Ļ��ߵ���ʱĿ��㣬Ȼ��continue
                    if(obsInFront(Sub_Msg.gridMsk,8))
                    {
                        int gridSub = getObsCoor(Sub_Msg.gridMsk);
                        bool reachObsPoint = false;
                        bool angleError = false;
                        //ǽ���ϰ�
                        if(gridSub%T_CAR_GRID_H_NUM == 19 || gridSub%T_CAR_GRID_H_NUM == 0)
                        {
                            //ת�䣬ֱ�����ʯͷ���ϰ���Ȼ��������ҵ�������;
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

                        //�õ��ϰ����Ա߾�����ĵ�������
                        vector<double > carEarthCoor(begin(Sub_Msg_Nav.EarthRefCoord),
                                end(Sub_Msg_Nav.EarthRefCoord));
                        vector<double> gridEarthCoor(2);
                        gridEarthCoor = gridToEarthXY(gridSub,Sub_Msg_Nav.Yaw_rad,carEarthCoor);
                        double obsPoint[2]={gridEarthCoor[0],gridEarthCoor[1]};


                        //�����ǲ������ϰ���ĸ������㷨������ת���׼Ŀ����ֱ��ֱ��Ŀ���������
                        //Ŀ�����ߵ��ϰ��Աߵ�
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
                                //ǰ��������ע����Ƕ��ǲ��Ƕ�׼�����û��׼��continue
                                ros::spinOnce();
                                double XYObsDist = getDistXY(Sub_Msg_Nav.EarthRefCoord, obsPoint);
                                if(XYObsDist<0.5)//��Ŀ���0.5�׵ķ�Χ��������Ŀ���
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
                                    //angleError = true;//ǰ������ֹͣ������ת��ѭ������������ֱ��
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

                    //3��ֱ��,��ÿ��ִ�ж���ǰ��Ҫ���Ƕ��Ƿ��׼
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

                    if(reachDest)//�����δ�ѭ��Ψһ�������������ĵ�i��Ŀ���
                    {
                        //destId++;
                        break;
                    }
                }
                if (!isAutoModeON)
                    break;    //���ؼ��̿���

                destId+=20;//����gps��֮�������4��
                if(destId == 1000)//������200�ף�ͣ�����ؼ��̿���
                    break;
            }
            printf("Successful Arrival!");




            /***********************************************************************************************
             *
             ****** ������ϲ��� *****************



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
                    if (!isAutoModeON)//����������Ľ�������жϣ�����϶�����break
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
    //���ڲ���
    //port1.Close();
    return 0;

}
