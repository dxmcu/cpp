#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include <iostream>

#ifndef FALSE
#define FALSE   (0)
#endif


#ifndef TRUE
#define TRUE    (!FALSE)
#endif

/***@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void*/

int speed_arr[] = { B921600, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                    B921600, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {921600, 38400,  19200,  9600,  4800,  2400,  1200,  300,
                  921600, 38400,  19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            //printf("baud: %s\n", speed_arr[i]);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0)
                perror("tcsetattr fd1");
            return;
        }
        tcflush(fd,TCIOFLUSH);
    }
}

int set_flow_ctrl(int fd, int c_flow)
{
    struct termios options;
    tcgetattr(fd, &options);
    /*设置数据流控制*/
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return (FALSE);
    }
    tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

int set_others(int fd)
{
    struct termios options;
    tcgetattr(fd, &options);
    /*设置输出模式为原始输出*/
    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/
    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN] = 1;//最少读取一个字符

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return (FALSE);
    }
    return (TRUE);
}

/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄*
*@param  databits 类型  int 数据位   取值 为 7 或者8*
*@param  stopbits 类型  int 停止位   取值为 1 或者2*
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;   /* Clear parity enable */
        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150; // 15 seconds
    options.c_cc[VMIN] = 0;

    tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}
/**
*@breif 打开串口
*/
int OpenDev(char *Dev)
{
    int	fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    { /*设置数据位数*/
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        return fd;

}
/**
*@breif 	main()
*/
int main(int argc, char **argv)
{
    int fd;
    int nread;
    char buff[512];
    char *dev ="/dev/ttyUSB0";
    fd = OpenDev(dev);
    if (fd>0)
        set_speed(fd, 9600);
    else
    {
        printf("Can't Open Serial Port!\n");
        exit(0);
    }
    if (set_flow_ctrl(fd, 0)== FALSE)
    {
        printf("Set flow Error\n");
        exit(1);
    }
    if (set_Parity(fd,8,1,'N')== FALSE)
    {
        printf("Set Parity Error\n");
        exit(1);
    }
    if (set_others(fd) == FALSE)
    {
        printf("Set flow Error\n");
        exit(1);
    }

    int index = 0;
    while (1)
    {
        usleep(100 * 1000);
        char c;
        std::cin.get(c);
        if (c == 'q') break;
        if (c == 'w')
        {
            char data[7] = {(char)0xDD, (char)0xA5, (char)0x03, (char)0x00, (char)0xFF, (char)0xFD, (char)0x77};
            int nrecv = write(fd, data, 7);
            printf("send: %d\n", nrecv);

            usleep(1000*1000);
            char buff[1024] = {0};
            printf("begin read:\n");
            int sum = 0;
            nread = 0;
            while(1)
            {
                printf("read 1\n");
                nread = read(fd, buff + sum, 512);
                printf("read 2\n");
                sum += nread;
                printf("read len: %d, %d\n", nread, sum);
                if (sum >= 34) break;
                usleep(50*1000);
                printf("read 3\n");
            }
            printf("read len: %d, %d\n", nread, sum);
            for (int i = 0; i < sum; ++i)
            {
                printf("%X ", (int)(buff[i] & 0xFF));
            }
            printf("\n");
        }
    }

    close(fd);
    //exit(0);
}
