/*
 * uart.cpp
 *
 *  Created on: May 5, 2016
 *      Author: odroid
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <iostream>
#include "uart.h"
#include "trace.h"
#include "CharacterRecognition.h"

int set_opt(int,int,int,char,int);
void uart_test()
{
    int fd,wr_static,i=10;
    char *uart3 = "/dev/ttyUSB0";
    char *buffer = "hello world!\r\n";

    printf("\r\n4412 uart3 writetest start\r\n");//���������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������������<span style="font-family: MicrosoftYaHei; font-size: 12pt;">Exynos4412<br style="orphans: 2; text-align: -webkit-auto; widows: 2;" /></span>
    //������������������������������������������������������������������������������������������������������������
    if((fd = open(uart3, O_RDWR|O_NOCTTY|O_NDELAY))<0){
        printf("open %s is failed",uart3);
        //cout << "Uart_fail\n";
    }
    else{
        printf("open %s is success\n",uart3);
       // cout << "Uart_Good\n";
        set_opt(fd, 115200, 8, 'N', 1);//������������������������������������������������������������������������������������������������������������
        while(i--)
        {
            wr_static = write(fd,buffer, strlen(buffer));
            if(wr_static<0)
                printf("write failed\n");
            else{
                printf("wr_static is %d\n",wr_static);
            }
            sleep(1);
        }
    }
    close(fd);
}
char data_to_send[50];
int Length;
void Data_pre(void)
{int _cnt=0,i=0,sum=0;
  unsigned int  _temp;
    data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//���������������������������
	data_to_send[_cnt++]=0;//���������������������������

	_temp = 0;//flow_pix[0];
	data_to_send[_cnt++]=_temp>>8;
	data_to_send[_cnt++]=_temp;
	_temp = 0;//flow_pix[1];
	data_to_send[_cnt++]=_temp>>8;
	data_to_send[_cnt++]=_temp;

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length=_cnt;
}
int circles_x,circles_y,circle_check=0,circle_check_r,circle_check_g,circles_r,track_check=0;
int state_v=0;
unsigned char circle_control[4];
void Data_pre_circle(void)
{int _cnt=0,i=0,sum=0;
  unsigned int  _temp;
    data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//���������������������������
	data_to_send[_cnt++]=0;//���������������������������

	data_to_send[_cnt++] = circle_check;
	data_to_send[_cnt++] = int(circles_x) / 255;
	data_to_send[_cnt++] = int(circles_x) % 255;
	data_to_send[_cnt++] = int(circles_y);


	_temp = circle_control[3];
	data_to_send[_cnt++]=_temp;
	_temp = -circle_control[1];
	data_to_send[_cnt++]=_temp;
	data_to_send[_cnt++]=circles_r;

	data_to_send[_cnt++]=track_check;
	_temp = 0;//flow_pix[0];
	data_to_send[_cnt++]=_temp>>8;
	data_to_send[_cnt++]=_temp;
	_temp = 0;//flow_pix[1];
	data_to_send[_cnt++]=_temp>>8;
	data_to_send[_cnt++]=_temp;


	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Length=_cnt;
}


std::vector<NumberPosition> number_position_send;
bool is_print_character;
extern int char_num;
void Data_pre_character()
{
	int _cnt=0,i=0,sum=0;
	  unsigned int  _temp;
	    data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0xAF;
		data_to_send[_cnt++]=0x03;//���������������������������
		data_to_send[_cnt++]=0;//���������������������������

		//data_to_send[_cnt++] = (int)number_position_send.size();
//		for(size_t i=0;i<number_position_send.size();i++)
//		{
//			data_to_send[_cnt++] = number_position_send[i].number_;
//			data_to_send[_cnt++] = int(number_position_send[i].position_.x) / 255;
//			data_to_send[_cnt++] = int(number_position_send[i].position_.x) % 255;
//			data_to_send[_cnt++] = int(number_position_send[i].position_.y);
//		}

		if(number_position_send.size()==0)
		{
			data_to_send[_cnt++] = 0;
		}
		else
		{
			if(number_position_send[0].number_[0] == char_num+48)
			{
			data_to_send[_cnt++] = 1;
			data_to_send[_cnt++] = int(number_position_send[i].position_.x)
					/ 255;
			data_to_send[_cnt++] = int(number_position_send[i].position_.x)
					% 255;
			data_to_send[_cnt++] = int(number_position_send[i].position_.y);
			}
			else
			{
				data_to_send[_cnt++] = 0;
			}
		}


		data_to_send[3] = _cnt-4;

		for( i=0;i<_cnt;i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;
		Length=_cnt;
}


extern bool LBtnDown;

void Data_pre_mouse(int x,  int y)
{
	int _cnt=0,i=0,sum=0;
		  unsigned int  _temp;
		    data_to_send[_cnt++]=0xAA;
			data_to_send[_cnt++]=0xAF;
			data_to_send[_cnt++]=0x04;//���������������������������
			data_to_send[_cnt++]=0;//���������������������������

			data_to_send[_cnt++] = int(LBtnDown);
			data_to_send[_cnt++] = int(x) / 255;
			data_to_send[_cnt++] = int(x) % 255;
			data_to_send[_cnt++] = y;


			data_to_send[3] = _cnt-4;

			for( i=0;i<_cnt;i++)
				sum += data_to_send[i];
			data_to_send[_cnt++] = sum;
			Length=_cnt;
}

void uart_mouse(int x, int y)
{
	static int init=0;
	int wr_static,i=10;
	   //char *uart3 = "/dev/ttyUSB0";
	   char *uart3 ="/dev/ttySAC0";//direct connect
	   char *buffer = "hello world!\r\n";
	  if(!init){//init=1;
	   ///if((fd = open(uart3, O_RDWR|O_NOCTTY|O_NDELAY))<0){
	   if((fd = open(uart3, O_RDWR, S_IRUSR | S_IWUSR))<0){
	       //printf("open %s is failed\n",uart3);
	       uart_good=0;
	   }
	   else{
	       //printf("open %s is success\n",uart3);
	       set_opt(fd, 230400, 8, 'N', 1);uart_good=1;
	      // wr_static = write(fd,send_buf, strlen(send_buf));
	      // sleep(1);
	       //close(fd);
	   }
	 }

	if(uart_good){init=1;
	  //Data_pre_circle();
	printf("uart_good=%d\n",uart_good);
	Data_pre_mouse( x,  y);

	  write(fd,data_to_send, Length);
	  read_uart();
	 }
}


int fd;
int uart_good=0;
void uart_send()
{static int init=0;
int wr_static,i=10;
   //char *uart3 = "/dev/ttyUSB0";
   char *uart3 ="/dev/ttySAC0";//direct connect
   char *buffer = "hello world!\r\n";
  if(!init){init=1;
   ///if((fd = open(uart3, O_RDWR|O_NOCTTY|O_NDELAY))<0){
   if((fd = open(uart3, O_RDWR, S_IRUSR | S_IWUSR))<0){
       //printf("open %s is failed\n",uart3);
       uart_good=0;
   }
   else{
       //printf("open %s is success\n",uart3);
       set_opt(fd, 230400, 8, 'N', 1);uart_good=1;
      // wr_static = write(fd,send_buf, strlen(send_buf));
      // sleep(1);
       //close(fd);
   }
 }

if(uart_good){init=1;
  //Data_pre_circle();
printf("uart_good=%d\n",uart_good);
Data_pre_character();

  write(fd,data_to_send, Length);
  read_uart();
 }
//close(fd);
}

char  buff[10];
int    Len;
int  readByte = read(fd,buff,Len);
#define RX_len 1
void read_uart(void)
{
      //  read(fd, RXBUF , RX_len);

       int ret,n,pos,retval;
       fd_set rfds;
       struct timeval tv ;
       pos = 0;//������������������������

       for(n = 0; n < RX_len; n++)
       {
    	   buff[n] = 0x00;
       }

       FD_ZERO(&rfds);// ������������������������������������
       FD_SET(fd,&rfds);// ������������������������������������
       tv.tv_sec = 0;
       tv.tv_usec = 10;

       while(FD_ISSET(fd,& rfds)) // ��������������������������������������������
       {
// ����������������������������������������������������������������������������
              FD_ZERO(&rfds);// ������������������������������������
              FD_SET(fd,&rfds);// ������������������������������������

              retval = select(fd+1,&rfds,NULL,NULL,&tv);
              if(retval == -1)
              {
                     perror("select()");
                     break;
              }
              else if(retval)
              {  //��������������������������������
                     //sleep(2);
                     ret = read(fd, buff, RX_len);
                     pos += ret;
                     //printf("ret = %d \n",ret);

                     if((1)) // ��������������������������������������������������������
                     {       state_v=buff[0];
                            FD_ZERO(&rfds);
                            FD_SET(fd,&rfds);
                            retval = select(fd+1,&rfds,NULL,NULL,&tv);
                            if(!retval)//no datas
                            {
                                   break;
                            }
                     }
              }
              else
              {
                     break;
              }
       }
}
//������������������������������������������������������������������������������������������������������������������������������������������������������������������
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 230400:
         cfsetispeed(&newtio, B230400);
         cfsetospeed(&newtio, B230400);
         break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
    newtio.c_cflag |=  CSTOPB;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
//  printf("set done!\n\r");
    return 0;
}





