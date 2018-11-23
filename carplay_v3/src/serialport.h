//
// Created by peng on 18-11-12.
//

#ifndef CARPLAY_V3_SERIALPORT_H
#define CARPLAY_V3_SERIALPORT_H
//������ص�ͷ�ļ�
#include<stdio.h>      /*��׼�����������*/
#include<stdlib.h>     /*��׼�����ⶨ��*/
#include<unistd.h>     /*Unix ��׼��������*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*�ļ����ƶ���*/
#include<termios.h>    /*PPSIX �ն˿��ƶ���*/
#include<errno.h>      /*����Ŷ���*/
#include<string.h>
#include<string>

using namespace std;

#define UART_FALSE  -1
#define UART_TRUE   0

#define GNR_COM  1
#define USB_COM  2
#define COM_TYPE USB_COM

enum GoState
{
    GOAHEAD,
    GOBACK,
    GOLEFT,
    GORIGHT,
    EMERGENCYSTOP,
    RACE,
};


class SerialPort
{
public:
    int fd;              								     //�ļ�������
    int isnormal;
    SerialPort(); 	      		           					     //���캯��
    ~SerialPort(void );			           					     //��������
    int Open(int id,int speed);                	   					     //�򿪴���
    void Close(void );                    	   					     //�رմ���
    int Set(int speed,int flow_ctrl,int databits,int stopbits,char parity);      //�������ò���
    int readBuffer(char *rcv_buf,int data_len);                           //���ڽ���
    int writeBuffer(unsigned char *send_buf,int data_len);       	     //���ڷ���
    int sendWheelSpd(GoState goMode);         //���������ٶ�,//zou�ĳ��Լ�����ӵĲ���
protected:
private:
    //ָʾ�����Ƿ�����
};
unsigned char XorCode(unsigned char *buf,int len);
unsigned char XorCode(std::string buf,int len);


#endif //CARPLAY_V3_SERIALPORT_H
