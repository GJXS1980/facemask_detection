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

#define DEBUG 0

const int FALSE = -1;
const int TURE = 0;

unsigned char rcv_buf[7] = {0};
unsigned char send_buf[4]={0xA5,0x55,0x01,0xFB};

class USART
{
    public:
            int fd;
            USART(char* port);  //串口号
            int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
            int UART0_Send(unsigned char *send_buf,int data_len);
            int UART0_Recv(unsigned char *rcv_buf,int data_len);

	    int setCalibration(int value);
	    float GET_TEMP_VALUE(void);
	    void set_measure_obj(unsigned short mode);
	    void set_switch_command(unsigned short sw);
	    uint8_t Calibration(unsigned char *data,int len);
            void set_operate_mode(unsigned short mode); 	//指令或评估模式
	    void request_data_mode(unsigned short mode);	//连续或者单帧
	    

	    ~USART();
};

USART::USART(char* port)      //打开串口
{
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == FALSE) cout<<"打开串口设备失败,请检查设备设备连接"<<endl;
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
        cout<<"串口设置失败"<<endl;
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

int USART::UART0_Send(unsigned char *send_buf,int data_len)
{
    int len = 0;
    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        return len;
    }
    else{
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}

int USART::UART0_Recv(unsigned char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec = 100;  //等待50MS
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel){
        len = read(fd,rcv_buf,data_len);
        return len;
    }
    else{
        cout<<"读取串口数据失败!"<<endl;
        return FALSE;
    }
    return TURE;
}

//设置串口控制指令
void USART::set_switch_command(unsigned short sw)
{
	if(sw==1)
	{
		unsigned char cmd[]={0x43,0x4D,0x44,0x43,0x01,0x18};
		UART0_Send(cmd,6);
	}
	else if(sw==0)
	{
		unsigned char cmd[]={0x43,0x4D,0x44,0x43,0x00,0x17};
		UART0_Send(cmd,6);
	}	

}

uint8_t USART::Calibration(unsigned char *data,int len)
{
	unsigned short CRC=0;

	for(int i=0; i<len; i++)
	{
		CRC += data[i];
	}	

	return CRC&0xff;
}       	

//设置校准偏移
int USART::setCalibration(int value)
{

//	unsigned char cmd[]={0x43,0x4D,0x44,0x43,0x01,0x00};		
//	UART0_Send(cmd,6);

	return TURE;
}


float USART::GET_TEMP_VALUE(void)
{

	return 0.0;
}
void USART::set_operate_mode(unsigned short mode) 	//指令或评估模式
{

	if(mode==0)		//指令模式 ->输出字符串
	{	
		unsigned char cmd[]={0x43,0x4D,0x44,0x45,0x01,0x1A};
		UART0_Send(cmd,6);
	}
	if(mode==1)		//评估模式 ->输出二进制
	{	
		unsigned char cmd[]={0x43,0x4D,0x44,0x45,0x00,0x19};
		UART0_Send(cmd,6);
	}

}

void USART::request_data_mode(unsigned short mode)	//连续或者单帧
{

	if(mode==0)		//单帧模式
	{	
		unsigned char cmd[]={0x43,0x3D,0x44,0x4D,0x00,0x21};
		UART0_Send(cmd,6);
	}
	else if(mode==1)
	{
		unsigned char cmd[]={0x43,0x4D,0x44,0x4D,0x01,0x22};
		UART0_Send(cmd,6);
	}

}
void USART::set_measure_obj(unsigned short mode)
{

	if(mode==0)		//物体
	{	
		unsigned char cmd[]={0x43,0x4D,0x44,0x4F,0x00,0x23};
		UART0_Send(cmd,6);
	}
	else if(mode==1)	//人体
	{	
		unsigned char cmd[]={0x43,0x4D,0x44,0x4F,0x01,0x24};
		UART0_Send(cmd,6);
	}

}	


USART::~USART()
{
    close(fd);
}


//	主函数
int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "Temperature_measurement_node");	//	初始化节点
    ros::NodeHandle n;	
    ros::NodeHandle node("~");

    std::string USB;   
    std_msgs::Float32 temp;
    ros::Publisher temp_pub = n.advertise<std_msgs::Float32>("Temperature_measurement_topic", 100);	
    node.param<std::string>("usb_port",  USB, "/dev/hg_infrared");  //  获取测温端口,可以直接从launch文件输入
    char *usb_port = (char*)USB.c_str();

    USART model(usb_port);
    if(model.UART0_Set(model.fd, 115200, 0, 8, 1, 'N')==FALSE)
      exit(0);

    //1.上电默认为指令模式
    //2.设置为连续模式	
    //3.打开串口数据

    //model.set_operate_mode(0); //指令模式	

    float t;
  
    uint8_t crc;
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
	 bzero(rcv_buf,sizeof(rcv_buf));
	 tcflush(model.fd,TCIOFLUSH);

	  model.UART0_Send(send_buf,4);	
	  model.UART0_Recv(rcv_buf,7);

	  crc=model.Calibration(rcv_buf,6);	

	  if(crc != rcv_buf[6])
	  {
 		//printf("crc:%02X data[6]:%02X\n",crc,rcv_buf[6]);
		continue;
	  }

	t = (rcv_buf[2]+rcv_buf[3]*256.0)/100.0f;	
	temp.data=t;
	temp_pub.publish(temp);
	

#if DEBUG
	  for(int i=0;i<7;i++)
		printf("data[%d]:%02X ",i,rcv_buf[i]);
	  		
	 printf("\n");
	 printf("\t校验位CRC:%02X\n",crc);	
	 printf("----------------------------------------\n");
	 printf("\t测试温度:%f\n",t);
 	 printf("----------------------------------------\n");

#endif		         	
	
	 loop_rate.sleep();
        
    }
    return 0;
}
