//
// Created by peng on 18-11-12.
//

#include "serialport.h"

#include "serialport.h"

/*******************************************************************
* 名称：    SerialPort()
* 功能：    构造函数
* 入口参数: 无
* 出口参数: 无
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
* 名称：       Open
* 功能：       打开串口并返回串口设备文件描述
* 入口参数：   id     :串口号
	       speed :波特率
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int SerialPort::Open(int id,int speed)
{
    const char *dev_t[] ={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2"}; //此处可能要修改为/dev/ttyS0、1、2，
    // 根据使用的是USB/RS232转换器还是直接的串口线
    fd = open( dev_t[id], O_RDWR|O_NOCTTY|O_NDELAY);

    printf("\nfdfdfdf = %d\n",fd);

    if (UART_FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(UART_FALSE);
    }

    //恢复串口为阻塞状态 等待数据读入
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(UART_FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }

    //测试是否为终端设备  确认串口是否打开
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

    //设置串口信息
    isnormal = Set(speed,0,8,1,'N');
    if(isnormal != UART_FALSE)
    {
        printf("serialPort setup successful!!!\n");
    }

    return fd;
}

/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/

void SerialPort::Close( )
{
    close(fd);
}

/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int SerialPort::Set(int speed,int flow_ctrl,int databits,int stopbits,char parity)
{

    int   i;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，
     * 并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。
     * 若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(UART_FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < 7;  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
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
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return (UART_FALSE);
    }
    // 设置停止位
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

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
//options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (UART_FALSE);
    }
    printf("usart set normal\r\n");
    return (UART_TRUE);
}

/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
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

    //使用select实现串口的多路通信
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
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
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
* 名称：       XorCode， 校验
* 功能：       加密数据
* 入口参数：   *buf:待加密的数据
*              len: 带加密数据长度
*
* 出口参数：    加密过后的值
*******************************************************************/

//zou，校验，前三位之和与0x7f求并得到校验值
unsigned char XorCode(unsigned char *buf,int len)
{
    unsigned  char ans = buf[0];
    for(int i = 1; i<len;i++)
        ans = ans+buf[i];
    return ans & 0X7F;  //有问题么？
}

unsigned char XorCode(string buf,int len)
{
    int ans = buf[0];
    for(int i = 1; i< len;i++)
        ans ^= buf[i];
    return ans&0x7f;
}


//下面需要修改，运动速度应随距离障碍物距离反比变化
int SerialPort::sendWheelSpd(GoState goMode)
{
    unsigned char buff[20];

    switch (goMode)
    {
        case GOAHEAD:
            buff[0]=0x80;
            buff[1]=0X04;
            buff[2]=0x30;//车速48
            buff[3]=XorCode(buff,3);
            break;
        case RACE:
            buff[0]=0x80;
            buff[1]=0X04;
            buff[2]=0x40;//车速64
            buff[3]=XorCode(buff,3);
            break;
        case GOBACK:
            buff[0]=0x80;
            buff[1]=0X05;
            buff[2]=0x24;//车速36
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X12,0x22,XorCode(buff,3)};//车速34
            break;
        case GOLEFT:
            buff[0]=0x80;
            buff[1]=0X01;
            buff[2]=0x30;//左转48
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X13,0x22,XorCode(buff,3)};//左转34
            break;
        case GORIGHT:
            buff[0]=0x80;
            buff[1]=0X00;
            buff[2]=0x30;//右转48
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X13,0x5E,XorCode(buff,3)};//右转94
            break;
        case EMERGENCYSTOP:
            buff[0]=0x80;
            buff[1]=0X06;
            buff[2]=0x40;//车速0
            buff[3]=XorCode(buff,3);
            //buff[20]={0xA0,0X12,0x22,XorCode(buff,3)};//暂时以倒车代替紧急制动
            break;
    }
    int len = writeBuffer(buff,4);
    return len;
}
