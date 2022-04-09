#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

using namespace std;

const int FALSE = -1;
const int TURE = 0;

unsigned char rcv_buf[8] = {0};
unsigned char send_buf[5]={0x40,0x01,0x01,0x40,0x0D};

class USART
{
    public:
            int fd;
            USART(char* port);  //串口号
            int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
            int UART0_Send(int fd,unsigned char *send_buf,int data_len);
            int UART0_Recv(int fd,unsigned char *rcv_buf,int data_len);
            float GET_TEMP(void);
            int temp_move(int size);
            int temp_distance(int mode);
            ~USART();
};

class Listener
{
    public:
            int x, count = 0;
    public:
        void callback(const std_msgs::Int32::ConstPtr& data);
        int print_datax(){
            return x;
        }
};

void Listener::callback(const std_msgs::Int32::ConstPtr& data)
{
    x = data->data;
    print_datax();
    ++count;
}


USART::USART(char* port)      //打开串口
{
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == FALSE) cout<<"can't open Serial Port"<<endl;
}


int USART::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    unsigned int   i;
    //int   status;
    int speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
    struct termios options;
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if(tcgetattr(fd, &options) != 0){
        cout<<"SetupSerial 1"<<endl;
        return(FALSE);
    }
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++){
        if  (speed == name_arr[i]){
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    //设置数据流控制
    switch(flow_ctrl){
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
    switch (databits){
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
                 cout<<"Unsupported data size"<<endl;
                 return (FALSE);
    }
    //设置校验位
    switch (parity){
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
                 cout<<"Unsupported parity"<<endl;;
                 return (FALSE);
    }
    // 设置停止位
    switch (stopbits){
        case 1:
                 options.c_cflag &= ~CSTOPB; break;
        case 2:
                 options.c_cflag |= CSTOPB; break;
        default:
                       cout<<"Unsupported stop bits"<<endl;
                       return (FALSE);
    }

 	/*******修改输出模式，原始数据输出(重点问题) **********/ 
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);  
    options.c_oflag &= ~OPOST;  
    options.c_cflag |= CLOCAL | CREAD;  
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0){
        cout<<"com set error!"<<endl;
        return FALSE;
    }
    return TURE;
}

int USART::UART0_Send(int fd,unsigned char *send_buf,int data_len)
{
    int len = 0;
    len = write(fd,send_buf,data_len);
    if (len == data_len ){
        return len;
    }
    else{
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}

int USART::UART0_Recv(int fd,unsigned char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec = 10;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel){
        len = read(fd,rcv_buf,data_len);
        return len;
    }
    else{
        cout<<"Sorry,I am wrong!"<<endl;
        return FALSE;
    }
    return TURE;
}

int USART::temp_move(int size)
{
    unsigned char cmd[6]={0x40,0x02,0x06,0x72,0x36,0x0D};      //0x72 = 114 向上偏移1.4
    unsigned char ack[6]={0};
    if(size){
        cmd[3] = size;
        cmd[4] = cmd[0]^cmd[1]^cmd[2]^cmd[3];
        UART0_Send(fd,cmd,6);
    }
    else UART0_Send(fd,cmd,6);
    UART0_Recv(fd,ack,6);
    return ack[3];  //  0 为成功
}

int USART::temp_distance(int mode)
{
    unsigned char cmd[6]={0x40,0x02,0x05,0x3C,0x7B,0x0D};      //默认为远距离测温（额头） 0x3C = 60cm
    unsigned char ack[6]={0};
    if(mode)  UART0_Send(fd,cmd,6);            // 1 测额头
    else {
        cmd[3] = 0x0A;                          // 10 cm
        cmd[4] = cmd[0]^cmd[1]^cmd[2]^cmd[3];
        UART0_Send(fd,cmd,6);
    }
    UART0_Recv(fd,ack,6);
    return ack[3];                           //  0 为成功
}


float USART::GET_TEMP(void)
{
    int temp = 0;
    float T = 0;
    int i;
    temp = (rcv_buf[5]<<8) + rcv_buf[4];
    T = temp/100;               //整数部分
    temp =  temp % 100;         //小数部分
    T += (float) temp /100 ;    //
    return T ;
}

USART::~USART()
{
    close(fd);
}


//	主函数
int main(int argc, char** argv)
{
    // 定义变量
    int len, sx, i, t;
    
    ros::init(argc, argv, "Thermometry");	//	初始化节点
    ros::NodeHandle n;	//	创建句柄
    ros::NodeHandle node("~");
    std::string USB;    //  定义USB类型
    std_msgs::Float32 temp;
    ros::Rate r(1);
    ros::Publisher temp_pub = n.advertise<std_msgs::Float32>("Temperature", 1000);	//	发布体温话题    

    // 获取识别的端口
    node.param<std::string>("usb_port",  USB, "/dev/ttyUSB0");  //  获取测温端口,可以直接从launch文件输入
    char *usb_port = (char*)USB.c_str(); // 字符串转换成字符类型

   //float Therm;
    char buf[] = {"\\-+|-/-"};
    USART Port(usb_port);
    Port.UART0_Set(Port.fd, 9600, 0, 8, 1, 'N');

    //while(Port.temp_distance(1));    // 1 额头 0 手心
    //while(Port.temp_move(0x75));         // 0 默认偏移 (1.4)  40~70厘米 改为1.7度

    while (ros::ok()){

        Listener listener;
        ros::Subscriber sub = n.subscribe("NoMask", 1000, &Listener::callback, &listener);
        ros::Rate loop_rate(10);

        while(ros::ok() and listener.count <= 0){
            ros::spinOnce();
            loop_rate.sleep();
        }

        sx = listener.print_datax();
        if (sx == 10 || sx == 1){
			len = Port.UART0_Send(Port.fd, send_buf, 5);
        	usleep(1000*5);
            if(len > 0){
                len = 0;
                len = Port.UART0_Recv(Port.fd, rcv_buf, 8);
                
                if (!rcv_buf[3]){
                	temp.data = Port.GET_TEMP();
                	temp_pub.publish(temp);
                }
                else{
                	t = 0;
                }
            }
            else{
               	t = 0;
            }
        }
/*        else{
        	if (sx == 1)
        	{
        	temp.data = -999.9;
        	temp_pub.publish(temp);
        	}
        }*/
        //usleep(1000*300);
        
    }
    return 0;
}
