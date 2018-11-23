//
// Created by peng on 18-11-12.
//

#include "serialport.h"

#include "serialport.h"

/*******************************************************************
* ���ƣ�    SerialPort()
* ���ܣ�    ���캯��
* ��ڲ���: ��
* ���ڲ���: ��
*******************************************************************/
SerialPort::SerialPort()
{
    isnormal = UART_FALSE;
    fd = UART_FALSE;
}

SerialPort::~SerialPort()
{
    Close();
}
/*******************************************************************
* ���ƣ�       Open
* ���ܣ�       �򿪴��ڲ����ش����豸�ļ�����
* ��ڲ�����   id     :���ں�
	       speed :������
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int SerialPort::Open(int id,int speed)
{
    const char *dev_t[] ={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2"}; //�˴�����Ҫ�޸�Ϊ/dev/ttyS0��1��2��
    // ����ʹ�õ���USB/RS232ת��������ֱ�ӵĴ�����
    fd = open( dev_t[id], O_RDWR|O_NOCTTY|O_NDELAY);

    printf("\nfdfdfdf = %d\n",fd);

    if (UART_FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(UART_FALSE);
    }

    //�ָ�����Ϊ����״̬ �ȴ����ݶ���
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(UART_FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }

    //�����Ƿ�Ϊ�ն��豸  ȷ�ϴ����Ƿ��
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(UART_FALSE);
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);

    //���ô�����Ϣ
    isnormal = Set(speed,0,8,1,'N');
    if(isnormal != UART_FALSE)
    {
        printf("serialPort setup successful!!!\n");
    }

    return fd;
}

/*******************************************************************
* ���ƣ�                UART0_Close
* ���ܣ�                �رմ��ڲ����ش����豸�ļ�����
* ��ڲ�����        fd    :�ļ�������     port :���ں�(ttyS0,ttyS1,ttyS2)
* ���ڲ�����        void
*******************************************************************/

void SerialPort::Close( )
{
    close(fd);
}

/*******************************************************************
* ���ƣ�                UART0_Set
* ���ܣ�                ���ô�������λ��ֹͣλ��Ч��λ
* ��ڲ�����        fd        �����ļ�������
*                              speed     �����ٶ�
*                              flow_ctrl   ����������
*                           databits   ����λ   ȡֵΪ 7 ����8
*                           stopbits   ֹͣλ   ȡֵΪ 1 ����2
*                           parity     Ч������ ȡֵΪN,E,O,,S
*���ڲ�����          ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int SerialPort::Set(int speed,int flow_ctrl,int databits,int stopbits,char parity)
{

    int   i;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)�õ���fdָ��������ز�����
     * �������Ǳ�����options,�ú��������Բ��������Ƿ���ȷ���ô����Ƿ���õȡ�
     * �����óɹ�����������ֵΪ0��������ʧ�ܣ���������ֵΪ1.
    */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(UART_FALSE);
    }

    //���ô������벨���ʺ����������
    for ( i= 0;  i < 7;  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���
    options.c_cflag |= CLOCAL;
    //�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ����ж�ȡ��������
    options.c_cflag |= CREAD;

    //��������������
    switch(flow_ctrl)
    {

        case 0 ://��ʹ��������
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://ʹ��Ӳ��������
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://ʹ�����������
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //��������λ
    //����������־λ
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
            options.c_cflag |= CS5;
            break;
        case 6    :
            options.c_cflag |= CS6;
            break;
        case 7    :
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return (UART_FALSE);
    }
    //����У��λ
    switch (parity)
    {
        case 'n':
        case 'N': //����żУ��λ��
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://����Ϊ��У��
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://����ΪżУ��
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //����Ϊ�ո�
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (UART_FALSE);
    }
    // ����ֹͣλ
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return (UART_FALSE);
    }

    //�޸����ģʽ��ԭʼ�������
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//�Ҽӵ�
//options.c_lflag &= ~(ISIG | ICANON);

    //���õȴ�ʱ�����С�����ַ�
    options.c_cc[VTIME] = 1; /* ��ȡһ���ַ��ȴ�1*(1/10)s */
    options.c_cc[VMIN] = 1; /* ��ȡ�ַ������ٸ���Ϊ1 */

    //�����������������������ݣ����ǲ��ٶ�ȡ ˢ���յ������ݵ��ǲ���
    tcflush(fd,TCIFLUSH);

    //�������� (���޸ĺ��termios�������õ������У�
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (UART_FALSE);
    }
    printf("usart set normal\r\n");
    return (UART_TRUE);
}

/*******************************************************************
* ���ƣ�                  UART0_Recv
* ���ܣ�                ���մ�������
* ��ڲ�����        fd                  :�ļ�������
*                              rcv_buf     :���մ��������ݴ���rcv_buf��������
*                              data_len    :һ֡���ݵĳ���
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int SerialPort::readBuffer(char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    if(isnormal == UART_FALSE)return UART_FALSE;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 0;
    time.tv_usec = 1000;  //1ms

    //ʹ��selectʵ�ִ��ڵĶ�·ͨ��
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        //  printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
        //   printf("Sorry,I have not received!\n");
        return UART_FALSE;
    }
}

/********************************************************************
* ���ƣ�                  UART0_Send
* ���ܣ�                ��������
* ��ڲ�����        fd                  :�ļ�������
*                              send_buf    :��Ŵ��ڷ�������
*                              data_len    :һ֡���ݵĸ���
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
int SerialPort::writeBuffer(unsigned char *send_buf,int data_len)
{
    int len = 0;
    if(isnormal == UART_FALSE)return UART_FALSE;

    // printf("hehe1\n");
    len = write(fd,(char *)send_buf,data_len);
    //  printf("%d \n",fd);
    //   printf("haha2");
    if (len == data_len )
    {
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return UART_FALSE;
    }
}

/********************************************************************
* ���ƣ�       XorCode�� У��
* ���ܣ�       ��������
* ��ڲ�����   *buf:�����ܵ�����
*              len: ���������ݳ���
*
* ���ڲ�����    ���ܹ����ֵ
*******************************************************************/

//zou��У�飬ǰ��λ֮����0x7f�󲢵õ�У��ֵ
unsigned char XorCode(unsigned char *buf,int len)
{
    unsigned  char ans = buf[0];
    for(int i = 1; i<len;i++)
        ans = ans+buf[i];
    return ans & 0X7F;  //������ô��
}

unsigned char XorCode(string buf,int len)
{
    int ans = buf[0];
    for(int i = 1; i< len;i++)
        ans ^= buf[i];
    return ans&0x7f;
}


//������Ҫ�޸ģ��˶��ٶ�Ӧ������ϰ�����뷴�ȱ仯
int SerialPort::sendWheelSpd(GoState goMode)
{
    unsigned char buff[20];

    switch (goMode)
    {
        case GOAHEAD:
            buff[0]=0x80;
            buff[1]=0X04;
            buff[2]=0x30;//����48
            buff[3]=XorCode(buff,3);
            break;
        case RACE:
            buff[0]=0x80;
            buff[1]=0X04;
            buff[2]=0x40;//����64
            buff[3]=XorCode(buff,3);
            break;
        case GOBACK:
            buff[0]=0x80;
            buff[1]=0X05;
            buff[2]=0x24;//����36
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X12,0x22,XorCode(buff,3)};//����34
            break;
        case GOLEFT:
            buff[0]=0x80;
            buff[1]=0X01;
            buff[2]=0x30;//��ת48
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X13,0x22,XorCode(buff,3)};//��ת34
            break;
        case GORIGHT:
            buff[0]=0x80;
            buff[1]=0X00;
            buff[2]=0x30;//��ת48
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X13,0x5E,XorCode(buff,3)};//��ת94
            break;
        case EMERGENCYSTOP:
            buff[0]=0x80;
            buff[1]=0X06;
            buff[2]=0x40;//����0
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X12,0x22,XorCode(buff,3)};//��ʱ�Ե�����������ƶ�
            break;
    }
    int len = writeBuffer(buff,4);
    return len;
}
