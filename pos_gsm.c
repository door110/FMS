/*************************************************************************
    > File Name:pos_prc.c
    > Author:renk
    > Mail:door_110@163.com
    > Created Time: 2016年08月24日 星期三 13时42分40秒
 ************************************************************************/
#include "pos_gsm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

//#define DEBUG1

#define MAX_COM_NUM 3
#define MOUNT_COM   1
#define POS_COM     2
#define NON_PARITY 'S'
#define BUFFER_SIZE 30
#define Header_LSB 0x00
#define Header_MSB 0x93

#define SYNC1 0x56
#define SYNC2 0x4D
#define END1  0x0D
#define END2  0x0A
#define END3  '\0'
#define HEAD_LEN 5
#define END_LEN 3
#define SUM_LEN 3

#define GNR_COM  1
#define COM_TYPE 2 //COM_TYPE 为1时使用本机的串口，2为USB串口

#if 0
#define DATA_ACQ    "VM122@ /GA 10/MST 10/\r\n"  //飞行准备命令
#define START_FLY   "VM053@ /RC 1/MA 0/\r\n"
#define FLY_LINE    "VM216@ /SCA 1/ST 1/\r\n"
#define FLY_TURNS   "VM226@ /MA 0/AYN 0/\r\n"
#define END_TASK    "VM068@ /RC 0/\r\n"
#endif

#define DIFF_ANG                    50                      //限位角度差0.5----50
#define SPEED_LOW                   500                     //5m/s ----------- 500
#define ACTIVE_THRESHOLD_YAW        (2500-DIFF_ANG)         //2500 角度值*100
#define ACTIVE_THRESHOLD_PITCH      (880-DIFF_ANG)          //2500 角度值*100 MAX=880
#define ACTIVE_THRESHOLD_ROLL       (700-DIFF_ANG)           //2500 角度值*100 MAX=700
#define MIN_DIFF                    10                      //平台调整限差 0.1----10
//#define ACTIVE_THRESHOLD    35900        //2500 角度值*100


//#define CUT_TIME         2         //秒
//#define FRQ              10         //HZ
//#define RE_SEND          1
//#define NEW_SEND         0
//#define RING_SIZE  (CUT_TIME*FRQ)  //取奇数（中滤波值）,RING_SIZE开窗大小----->（HZ*TIME(s)）
//static int real_dir_buf[RING_SIZE];
//static int tail;
//static int flag;

static const char* DATA_ACQ   = "VM081@ /GA 0/\r\n";
static const char* INITIAL    = "VM045H /RC 1/MA 0/\r\n";
static const char* AUTO       = "VM216@ /SCA 1/ST 1/\r\n";   /****ST 1****/
//static const char* AUTO     = "VM217@ /SCA 1/ST 0/\r\n";   /****ST 0****/
static const char* MAN        = "VM226@ /MA 0/AYN 0/\r\n";
static const char* MOUNT_ZERO = "VM048@ /ARN 0/APN 0 /AYN 0/\r\n";
static const char* RET        = "VM081H /\r\n";
static const char* FASTL      = "VM151@ /FL/\r\n";
static const char* TABLE      = "VM068@ /RC 0/\r\n";
static const char* MOUNT_SPA_SET_70   = "VM189@ /SPA 70/\r\n";
static const char* MOUNT_SPA_ASK      = "VM069@ /SPA/\r\n";
static const char* MOUNT_SPD_SET_70   = "VM186@ /SPD 70/\r\n";
static const char* MOUNT_SPD_ASK      = "VM066@ /SPD/\r\n";
static const char* MOUNT_NO_READY     = "VM088A /\r\n";
//static const char* DATA_ACQ1 = "VM118@ /GA 5/MST 100/\r\n";  //飞行准备命令

typedef    struct{
    double    vtime;                    // UTC seconds of week or elapsed seconds if GPS not available
    char    roll_lsb;
    char    roll_msb;
    char    pitch_lsb;
    char    pitch_msb;
    char    heading_lsb;
    char    heading_msb;
    long    latitude;
    long    longitude;
    long    altitude;
    char    speed_lsb;
    char    speed_msb;
    char    track_lsb;
    char    track_msb;
} __attribute__ ((packed)) POS_PAST1;                        //  POS信息


static int cmp_int(const void *a, const void *b);
static int filter_middle(void *val,int len);
//static int filter_mean(int *val,int len);
static int ay_angle(unsigned short heading,unsigned short track);
static short real_val(char msb,char lsb);
static int packet(char *msg,int len,char **res);
//static int mount_date_prc(const char *tmp,int len,const attitude_t *atti_msg);


//static int mount_data_read(int fd_mount,char *buf,int buf_size);
//static int mount_data_write(int fd_mount,char *buf,int buf_size);
//static int pos_data_read(int fd_pos,char *buf,int buf_size);
//static int pos_data_write(int fd_pos,char *buf,int buf_size);
//static int pos_data_prc(void * buff,int buff_size,int fd_mount,int fd_pos,int mode,attitude_t *atti_msg);



/*
串口设置函数
波特率
数据位
校验位
停止位
*/
int set_com_config(int fd,int baud_rate,int data_bits,char parity,int stop_bits)
{
    struct termios new_cfg,old_cfg;
    int speed;
    /*保存并测试现有串口参数设置,在这里如果串口号等出错,会有相关的出错信息*/
    if (tcgetattr(fd, &old_cfg) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
    /* 设置字符大小*/
    new_cfg = old_cfg;
    cfmakeraw(&new_cfg); /* 配置为原始模式 */
    new_cfg.c_cflag &= ~CSIZE;
    /*设置波特率*/
    switch (baud_rate)
    {
        case 2400:  speed = B2400;
        break;
        case 4800:  speed = B4800;
        break;
        case 9600:  speed = B9600;
        break;
        case 19200: speed = B19200;
        break;
        case 38400: speed = B38400;
        break;
        default:
        case 115200:speed = B115200;
        break;
    }
    cfsetispeed(&new_cfg, speed);
    cfsetospeed(&new_cfg, speed);
    /*设置数据位*/
    switch (data_bits)
    {
        case 7: new_cfg.c_cflag |= CS7;
        break;
        default:
        case 8: new_cfg.c_cflag |= CS8;
        break;
    }
    /*设置奇偶校验位*/
    switch (parity)
    {
        default:
        case 'n':
        case 'N':
        {
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
        }
        break;
        case 'o':
        case 'O':
        {
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
        }
        break;
        case 'e':
        case 'E':
        {
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD;
            new_cfg.c_iflag |= INPCK;
        }
        break;
        case 's': /*as no parity*/
        case 'S':
        {
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_cflag &= ~CSTOPB;
        }
        break;
    }
    /*设置停止位*/
    switch (stop_bits)
    {
        default:
        case 1:new_cfg.c_cflag &=~CSTOPB;
        break;
        case 2:new_cfg.c_cflag |= CSTOPB;
        break;
    }
    /*设置等待时间和最小接收字符*/
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 1;
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &new_cfg)) != 0)
    {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}


/*打开串口函数*/
int open_port(int com_port)
{
    int fd;
    #if (COM_TYPE == GNR_COM) /* 使用普通串口 */
    char *dev[] = {"/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2"};
    #else /* 使用 USB 转串口 */
    char *dev[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
    #endif
    if ((com_port < 0) || (com_port > MAX_COM_NUM))
    {
        return -1;
    }
    /* 打开串口 */
    fd = open(dev[com_port - 1], O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd < 0)
    {
        perror("open serial port");
        return(-1);
    }
    /*恢复串口为阻塞状态*/
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl F_SETFL\n");
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
    }
    return fd;
}

#if 0
int read_pos_data(int fd_pos)
{
    char buff[BUFFER_SIZE];
    char head_tmp,data_tmp;
    int head_num,flag;//msg_num;
    int n = 0;
    puts("read pos");
    while(1)
    {
        //puts("get pos data");
        head_num =  read(fd_pos,&head_tmp,1);
        if(head_num<0)
        {
            perror("read()");
            return 1;
        }
        if(head_num == 1 && head_tmp == Header_LSB)
        {
            //printf("head_num = %d,head_lsb = %x\n",head_num,head_tmp);
            //fflush(stdout);
            head_num =  read(fd_pos,&head_tmp,1);
            //printf("head_num = %d,head_msb = %x\n",head_num,head_tmp);
            if(head_num < 0)
            {
                perror("read()");
                return 1;
            }
            if(head_num == 1 && (head_tmp&0xff) == Header_MSB)
            {
#ifdef DEBUG
                printf("read msg\n");
#endif
                for(n=0;n<30;n++)
                {
                    read(fd_pos,&data_tmp,1);
                    buff[n]= data_tmp;
#ifdef DEBUG
                    printf("%d-%x \n",n,data_tmp);
#endif
                }
                flag = 1;
             }
            else
                continue;
            if(flag)
                return 0;
        }
    }
    //return 0;
}
#endif

#if 0
int read_mount(int fd_mount,int fd_pos) //no use
{
    char tmp[64];
    int ret,num;
    if((num=read(fd_mount,tmp,64))>=1)
    {
        ret=write(fd_pos,tmp,num);
        printf("%s",tmp);
        if(ret != 1)
            perror("write_pos err");
    }
    else
    {
        perror("read_mount err");
    }
}
#endif

#if 0
int pos_to_mount(int fd_pos,int fd_mount,int mode)//no use
{
    char buff[BUFFER_SIZE];
    char head_tmp,data_tmp;
    int head_num,flag;//msg_num;
    int n = 0;
    puts("pos_to_mount");
    while(1)
    {
        //puts("get pos data");
        head_num =  read(fd_pos,&head_tmp,1);
        if(head_num<0)
        {
            perror("read()");
            return 1;
        }
        if(head_num == 1 && head_tmp == Header_LSB)
        {
            //printf("head_num = %d,head_lsb = %x\n",head_num,head_tmp);
            //fflush(stdout);
            head_num =  read(fd_pos,&head_tmp,1);
            //printf("head_num = %d,head_msb = %x\n",head_num,head_tmp);
            if(head_num < 0)
            {
                perror("read()");
                return 1;
            }
            if(head_num == 1 && (head_tmp&0xff) == Header_MSB)
            {
#ifdef DEBUG
                printf("read msg\n");
#endif
                for(n=0;n<30;n++)
                {
                    read(fd_pos,&data_tmp,1);
                    buff[n]= data_tmp;
#ifdef DEBUG
                    printf("%d-%x \n",n,data_tmp);
#endif
                }
                flag = 1;
             }
            else
                continue;
            if(flag)
            {
                //data_prc(buff,BUFFER_SIZE,fd_mount,fd_pos);
                flag = 0;
                //read_mount(fd_mount,fd_pos);
                continue;
            }
        }
    }
    return 0;
}
#endif


/*打包字符串为 VM+CHKSUM+MSG格式*/
static int packet(char *msg,int len,char **res)
{
    unsigned char i;                // index variable
    unsigned short sum = 0;         // (unsigned short) temporary sum of the byte complements
    unsigned char chkSum;           // (unsigned byte) the resultant checksum
    unsigned char SendchkSum[3];    // (unsigned byte)CHK1

    //unsigned char End[END_LEN]={END1,END2,END3};
    unsigned char Head[HEAD_LEN]={SYNC1,SYNC2,0x00,0x00,0x00};
#ifdef DEBUG
    printf("len = %d\n",len);
#endif
    *res = malloc(sizeof(char) * (len+HEAD_LEN+END_LEN));

    if(*res == NULL)
    {
        fprintf(stderr,"malloc err\n");
        return 1;
    }
    //printf("res addr is %p\n",*res);
    for(i=0;i<len;i++)       // start at index 0
    {
        sum = sum + (unsigned char)(~(msg[i])); // sum of complements
        //printf("sum[%d] = %x\n",i,sum);
    }
    sum = (sum & 0x00FF) + ((sum & 0xFF00) >> 8); // add MSB to LSB
    chkSum = (unsigned char)(sum & 0x00FF); // mask LSB for checksum
#ifdef DEBUG1
    printf("chkSum = %x\n",chkSum);
#endif
    SendchkSum[0] = chkSum/100+0x30;
    SendchkSum[1] = (chkSum-(SendchkSum[0]-0x30)*100)/10+0x30;
    SendchkSum[2] = chkSum-(SendchkSum[0]-0x30)*100-(SendchkSum[1]-0x30)*10+0x30;
    memcpy(Head+2,SendchkSum,SUM_LEN);
    memcpy(*res,Head,HEAD_LEN);
    memcpy(*res+HEAD_LEN,msg,len);
    //memcpy(*res+len+HEAD_LEN,End,END_LEN);
    return strlen(*res);
}

#define PI     18000
#define PI2    36000
/*计算航偏角*/
static int ay_angle(unsigned short heading,unsigned short track)
{
    //printf("heading = %d,track = %d\n",heading,track);
    int val = track - heading;
    if(abs(val)>PI && val>0)
        val -= PI2;
    else if(abs(val)>PI && val<0)
        val += PI2;
    //if(abs(val)>ACTIVE_THRESHOLD)
    //    val = 0;
    //printf("real_dir_yaw = %d\n",val);
    return val;
}


static int cmp_int(const void *a, const void *b)
{
    return *(int *)a - *(int *)b;
}

/*YAW滤波器----中值滤波*/
static int filter_middle(void *val,int len)
{
    //1.排序
    qsort(val,len,sizeof(int),cmp_int);
    return  *(int*)(val+((len-1)/2)*sizeof(int));
}


/*YAW滤波器----均值滤波*/

int filter_mean(int *val,int len)
{
    int i;
    long sum = 0;
    for(i = 0;i<len;i++)
    {
        //printf("val[%d]=%d\n",i,*(val+i));
        sum += *(val+i);
    }
    //printf("sum = %ld,mean = %ld\n",sum,sum/len);
    return  (sum/len);
}

/*POS数据转换--高低四位的数的转换*/
static short real_val(char msb,char lsb)
{
    short real = 0;
    real = (msb<<8)+(lsb & 0x00FF);
    return real;
}



/*解析POS的数据包,发送至平台控制角度
 *
 *mode,判断是否发送POS，YAW角的新数据 1为重发，0为新发
 *
 *atti_msg为回填结构体
 */


#if 0
int pos_data_prc(void * buff,int buff_size,int fd_mount,int fd_pos,int mode,attitude_t *atti_msg)
{
    //int data_buf[200];
    //int flag;
    char p[64]={'\0'};
    POS_PAST1 pos_msg;
    //attitude_t atti_msg;
    memcpy(&pos_msg,buff,buff_size);
    atti_msg->roll =  real_val(pos_msg.roll_msb,pos_msg.roll_lsb);
    atti_msg->pitch =  real_val(pos_msg.pitch_msb,pos_msg.pitch_lsb);
    int  heading =  real_val(pos_msg.heading_msb,pos_msg.heading_lsb);
    int  track =  real_val(pos_msg.track_msb,pos_msg.track_lsb);
#ifdef DEBUG
    printf("roll = %d,pitch = %d,heading = %d,track = %d\n",roll,pitch,heading,track);
#endif
    //格式@ /AP -450/AR 2/AYN 1800/
    //int dir = 0;//目前设置为0

    if(mode == RE_SEND)
    {
        snprintf(p,64,"@ /AP %d/AR %d/AYN %d/\r\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
    }
    else if (mode == NEW_SEND)
    {
        //int real_val = real_dir(heading,track);//航偏角计算
        tail = (tail+1)%RING_SIZE;
        real_dir_buf[tail++] = real_dir(heading,track);
        if(tail == RING_SIZE)
        {
            atti_msg->dir = filter_mean(real_dir_buf,RING_SIZE);//滤波函数
            snprintf(p,64,"@ /AP %d/AR %d/AYN %d/\r\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
        }
    }

    char *send = NULL;

    int num = packet(p,strlen(p),&send);//打包为VM格式
#ifdef DEBUG
    fprintf(stdout,"%s",send);
#endif
    //int w_num = write(fd_mount, send, num);//向fd写文件格式 //写的有bug，不严谨
    mount_data_write(fd_mount,send,num);
    //write(fd_mount,DATA_ACQ,strlen(DATA_ACQ));//发送数据请求 GA 0
    //read_mount(fd_mount,fd_pos);
    free(send);
    return 0;
#if 0
    if(w_num == num)
    {
        free(send);
        return 0;
    }
    else if(w_num < num)
    {
        int temp = write(fd,&send-w_num,num-w_num);
        if(temp == num-w_num)
            return 0;
        else
            return 1;
    }
#endif
}
#endif
/*
 *测试例
 *yaw = track-heading
 *
 *track = 349 ,heading = 1 ,val = -12
 *
 *track = 349 ,heading = 359, val = -10
 *
 *track = 1,heading =349 ,val = 12
 *
 *track = 359,heading = 349 ,val = 10
 *
 *track = 10,heading = 20,val = -10
 *
 *track = 20,heading = 10 ,val = 10
 *
*/
void pos_data_parse(void * buff,int buff_size,attitude_t *atti_msg)//
{
    POS_PAST1 pos_msg;
    memcpy(&pos_msg,buff,buff_size);
    atti_msg->roll =  real_val(pos_msg.roll_msb,pos_msg.roll_lsb);
    atti_msg->pitch =  real_val(pos_msg.pitch_msb,pos_msg.pitch_lsb);
    unsigned short  heading =  real_val(pos_msg.heading_msb,pos_msg.heading_lsb);
    unsigned short  track =  real_val(pos_msg.track_msb,pos_msg.track_lsb);
    //track = 27000;
#if 0
    //测试角度
    track   =  3000;
    heading =  4500 ;
#endif
    atti_msg->yaw =  ay_angle(heading,track);

    atti_msg->speed =  real_val(pos_msg.speed_msb,pos_msg.speed_lsb);
#ifdef DEBUG
    printf("roll = %d,pitch = %d,yaw = %d\n",atti_msg->roll,atti_msg->pitch,atti_msg->dir);
#endif
    fprintf(stdout,"roll = %d,pitch = %d,yaw = %d,speed = %d\n",atti_msg->roll,atti_msg->pitch,atti_msg->yaw,atti_msg->speed);
}

int ayn_angle(int ay_angle,int mount_yaw)
{

    int val = ay_angle + mount_yaw;
    //if(abs(val)>ACTIVE_THRESHOLD_YAW)
    //    val = 0;
    if(val>ACTIVE_THRESHOLD_YAW)
        val = ACTIVE_THRESHOLD_YAW;
    else if(val<-ACTIVE_THRESHOLD_YAW)
        val = -ACTIVE_THRESHOLD_YAW;
    return val;
}

short roll_angle(short mount_roll)
{
    short val = mount_roll;
    if(val >  ACTIVE_THRESHOLD_ROLL)
        val = ACTIVE_THRESHOLD_ROLL;
    else if(val<-ACTIVE_THRESHOLD_ROLL)
        val = -ACTIVE_THRESHOLD_ROLL;
    return val;
}

short pitch_angle(short mount_pitch)
{
    short val = mount_pitch;
    if(val>ACTIVE_THRESHOLD_PITCH)
        val = ACTIVE_THRESHOLD_PITCH;
    else if(val<-ACTIVE_THRESHOLD_PITCH)
        val = -ACTIVE_THRESHOLD_PITCH;
    return val;
}


//ret:1  速度低

int speed_is_low(int speed)
{
    if(speed>=SPEED_LOW)
        return 0;
    else
        return 1;
}

#if 0
int pos_data_packet(attitude_t *atti_msg, int mode, char **send,int yaw_old)
{
    //格式@ /AP -450/AR 2/AYN 1800/
    //printf("IN:pitch = %d,roll = %d,yaw = %d\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
    //printf("mode value = %d\n",mode);
    int num;
    char p[64]={'\0'};

    tail = (tail+1)%RING_SIZE;
    real_dir_buf[tail++] = atti_msg->dir;
    if(tail == RING_SIZE)
    {
        flag = 1;
    }

    if(mode == RE_SEND)
    {
        atti_msg->dir = yaw_old;
        snprintf(p,64,"@ /AP %d/AR %d/AYN %d/\r\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
        fprintf(stdout,"RE_SEND=%s",p);
    }
    else if (mode == NEW_SEND)
    {
        //int real_val = real_dir(heading,track);//航偏角计算
        //tail = (tail+1)%RING_SIZE;
        //printf("tail = %d\n",tail);
        //real_dir_buf[tail++] = atti_msg->dir;
        //printf("real_dir_buf[%d] = %d\n",tail-1,real_dir_buf[tail-1]);
        //printf("atti_msg->dir = %d\n",atti_msg->dir);
        //if(tail == RING_SIZE)
        //{
        //	flag = 1;
        //}
	    if(flag)
	    {
	        atti_msg->dir = filter_mean(real_dir_buf,RING_SIZE);//滤波函数
            printf("filter_mean dir = %d\n",atti_msg->dir);
            snprintf(p,64,"@ /AP %d/AR %d/AYN %d/\r\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
            fprintf(stdout,"NEW_SEND=%s",p);
	    }
    }
    num = packet(p,strlen(p),send);//打包为VM格式
#ifdef DEBUG
    fprintf(stdout,"%s",send);
#endif
    return num;
}
#endif

int pos_data_packet(attitude_t *atti_msg, char **send)
{
    //格式@ /AP -450/AR 2/AYN 1800/
    //printf("IN:pitch = %d,roll = %d,yaw = %d\n",atti_msg->pitch,atti_msg->roll,atti_msg->dir);
    //printf("mode value = %d\n",mode);
    int num;
    char p[64]={'\0'};
    snprintf(p,64,"@ /AP %d/AR %d/AYN %d/\r\n",atti_msg->pitch,atti_msg->roll,atti_msg->yaw);
    num = packet(p,strlen(p),send);//打包为VM格式
#if 1
    fprintf(stdout,"%s",*send);
#endif
    return num;
}


/*
 *  与pos握手，使其接受平台数据
 *   ret = 0成功
*/
int init_pos(int fd_pos)
{
    int head_num,n;
    //int flag;
    char head_tmp,data_tmp;
    char buf[20];
    //printf("init_pos\n");
    head_num = read(fd_pos,&head_tmp,1);
    if(head_num == 1 && head_tmp == 'V') //pos请求握手
    {
        buf[0] = 'V';
        head_num = read(fd_pos,&head_tmp,1);
        if(head_num == 1 && head_tmp == 'M')
        {
            buf[1] = 'M';
            for(n = 2;n<strlen(INITIAL);n++)
            {
                if(read(fd_pos,&data_tmp,1)==1)
                {
                    buf[n] = data_tmp;
                    //printf("data_tmp = %d\n",data_tmp);
                }
                else
                    return 1;
            }
            //printf("buf = %s\n",buf);
        }
    }
    //printf("INITIAL:%s\n",INITIAL);
    //int ret = strncmp(INITIAL,buf,strlen(INITIAL));
    //printf("ret = %d\n",ret);
    if(strncmp(INITIAL,buf,strlen(INITIAL)) == 0)
    {
            printf("send pos\n");
            if(write(fd_pos,RET,strlen(RET))== strlen(RET))
            {
                printf("send msg:%s\n",RET);
                return 0;
            }
            else
                return 1;
    }
    else
        return 1;
}


/*
 *与平台握手
 *ret为0成功
*/
int init_mount(int fd_mount)
{
    //char tmp[10];
    printf("init_mount\n");
    char *tmp = NULL;
    tmp = malloc(sizeof(char)*10);
    int i;
    write(fd_mount,INITIAL,strlen(INITIAL));
    sleep(1);
    for(i = 0;i <= 9;i++)
    {
        if(read(fd_mount,tmp+i,1)==1)
        {
           continue;
        }
        else
           break;
    }
    if(i<9)
    {
        fprintf(stderr,"read mount initial information err\n");
        free(tmp);
        return 1;
    }
    else if(i>=9)
    {
        printf("message is %s",tmp);
        int ret = strncmp(RET,tmp,strlen(RET));
        printf("ret = %d\n",ret);
        if(ret == 0)
        {
            free(tmp);
            return 0;
        }
        else
        {
            fprintf(stderr,"read mount initial information %s\n",tmp);
            free(tmp);
            return 1;
        }
    }
    return 1;
}

/*发送自动控制命令*/
int mount_auto(int fd_mount)
{
    int ret;
    ret = write(fd_mount,AUTO,strlen(AUTO));
    if(ret != strlen(AUTO))
    {
        perror("error");
        return -1;
    }
    return 0;
}

/*送手动控制命令*/
int mount_man(int fd_mount)
{
    int ret;
    ret = write(fd_mount,MAN,strlen(MAN));
    if(ret != strlen(MAN))
    {
        perror("error");
        return -1;
    }
    return 0;
}

/*发送平台安抚命令*/
int mount_fl(int fd_mount)
{
    int ret;
    ret = write(fd_mount,FASTL,strlen(FASTL));
    if(ret != strlen(FASTL))
    {
        perror("error");
        return -1;
    }
    return 0;
}

/*平台TABLE命令*/
int mount_table(int fd_mount)
{
    int ret;
    ret = write(fd_mount,TABLE,strlen(TABLE));
    if(ret != strlen(TABLE))
    {
        perror("error");
        return -1;
    }
    return 0;
}

/*发送平台数据请求命令*/
int mount_data_acq(int fd_mount)
{
    int ret;
    ret = write(fd_mount,DATA_ACQ,strlen(DATA_ACQ));
    if(ret != strlen(DATA_ACQ))
    {
        perror("error");
        return -1;
    }
    //printf("%s",DATA_ACQ);
    return 0;
}


/*发送平台位置0*/
int mount_data_zero(int fd_mount)
{
    int ret;
    ret = write(fd_mount,MOUNT_ZERO,strlen(MOUNT_ZERO));
    if(ret != strlen(MOUNT_ZERO))
    {
        perror("error");
        return -1;
    }
    //printf("%s",DATA_ACQ);
    return 0;
}

//设置平台航向（heading）增益
//ret :0成功
int mount_set_spd(int fd_mount)
{
    int ret;
    char buf[64]={'0'};
    ret = write(fd_mount,MOUNT_SPD_SET_70,strlen(MOUNT_SPD_SET_70));
    if(ret != strlen(MOUNT_SPD_SET_70))
    {
        perror("error");
        return -1;
    }
    //printf("%s",DATA_ACQ);
    usleep(200);
    ret = write(fd_mount,MOUNT_SPD_ASK,strlen(MOUNT_SPD_ASK));
    if(ret != strlen(MOUNT_SPD_ASK))
    {
        perror("error");
        return -1;
    }
    sleep(1);

    ret = mount_data_read(fd_mount,buf,sizeof(buf));
    if(ret == 1)
    {
        perror("error");
        return -1;
    }
    printf("%s",buf);
    ret = strncmp(MOUNT_SPD_SET_70,buf,strlen(MOUNT_SPD_SET_70));
    if(ret == 0)
    {
        return 0;
    }
    else
        return -1;
}

//设置平台水平增益
//ret :0成功
int mount_set_spa(int fd_mount)
{
    int ret;
    char buf[64]={'0'};
    ret = write(fd_mount,MOUNT_SPA_SET_70,strlen(MOUNT_SPA_SET_70));
    if(ret != strlen(MOUNT_SPA_SET_70))
    {
        perror("error");
        return -1;
    }
    usleep(200);
    ret = write(fd_mount,MOUNT_SPA_ASK,strlen(MOUNT_SPA_ASK));
    if(ret != strlen(MOUNT_SPA_ASK))
    {
        perror("error");
        return -1;
    }
    sleep(1);

    ret = mount_data_read(fd_mount,buf,sizeof(buf));
    if(ret == 1)
    {
        perror("error");
        return -1;
    }
    printf("%s",buf);
    ret = strncmp(MOUNT_SPA_SET_70,buf,strlen(MOUNT_SPA_SET_70));
    if(ret == 0)
    {
        return 0;
    }
    else
        return -1;
}
//printf("%s",DATA_ACQ);
#if 0
int mount_data(int fd_mount)//no use
{
    int ret;

    ret = write(fd_mount,DATA_ACQ1,strlen(DATA_ACQ1));//发送GA20 MST100
    if(ret != strlen(DATA_ACQ1))
    {
        perror("error");
        return -1;
    }
    return 0;
}

int pos_req(int fd_pos)//no use
{
    int ret;

    ret = write(fd_pos,RET,strlen(RET));//
    printf("%s",RET);
    if(ret != strlen(RET))
    {
        perror("error");
        return -1;
    }
    return 0;
}
#endif


int mount_date_parse(const char *tmp)
{
    int ret,chk;
    attitude_t atti_tmp;
    ret = sscanf(tmp,"VM%d@ /GR %hd/GP %hd/GY %d\r\n",&chk,&(atti_tmp.roll),&(atti_tmp.pitch),&(atti_tmp.yaw));
    if(ret == 4)
        return 0;
    else
        return 1;
}
/*
 *解析平台返回数据
 *判断姿态角是否需要重发
 *ret：0 发送新的位置
 *ret：1 重发上一次的位置
 *ret:-1 格式错误
*/
int mount_date_prc(const char *tmp,int len,const attitude_t *atti_msg,int *yaw_mount)
{
    int sum = 0;
    int i;
    unsigned char chkSum = 0;
    unsigned int chk;
    attitude_t atti_tmp;
    int ret = sscanf(tmp,"VM%d@ /GR %hd/GP %hd/GY %d\r\n",&chk,&(atti_tmp.roll),&(atti_tmp.pitch),&(atti_tmp.yaw));
    //printf("******************\n");
    //printf("len = %d\n",len);
    for(i=5;i<len;i++)       // start at index 5
    {
        //printf("in for \n");
        sum = sum + (unsigned char)(~(tmp[i])); // sum of complements
        //printf("sum[%d] = %x\n",i,sum);
    }

    //printf("chkSum = %d,chk = %d\n",chkSum,chk);

    sum = (sum & 0x00FF) + ((sum & 0xFF00) >> 8); // add MSB to LSB
    chkSum = (unsigned char)(sum & 0x00FF); // mask LSB for checksum

    //printf("chkSum = %d,chk = %d\n",chkSum,chk);

    if(chkSum == chk && ret == 4)
    {
        *yaw_mount  = atti_tmp.yaw;
        if(abs(atti_msg->yaw - atti_tmp.yaw) <= MIN_DIFF)
            return 0;
        else
            return 1;
    }

    else
        return -1;
}

#if 0
/*
 *读取座架返回数据
 *
 *并返回数据给pos
 *
 *ret：0 发送新的位置
 *ret：1 重发上一次的位置
 *ret:-1 格式错误
 *
 */
int read_mount_data(int fd_pos,int fd_mount,const attitude_t *atti_msg)//判断座架是否需要调整，传入之前发送给平台的atti_msg结构体
{
    int i = 0;
    int ret;
    int head_num,n;
    char head_tmp,data_tmp;
    char tmp[128]={'\0'};
    head_num = read(fd_pos,&head_tmp,1);
    if(head_num == 1 && head_tmp == 'V') //
    {
        tmp[0] = 'V';
        n = 1;
        while(1)
        {
            if(read(fd_mount,&data_tmp,1)==1)
                tmp[n++] = data_tmp;
            if(data_tmp == '\n')
                break;
        }
    }

    //write(fd_pos,tmp,strlen(tmp));

    ret = mount_date_prc(tmp,strlen(tmp),atti_msg);
    if(ret == 1)//重发
    {
        write(fd_pos,tmp,strlen(tmp));
        return 1;
    }
    else if(ret == 0)//发送新的位置
    {
        write(fd_pos,tmp,strlen(tmp));
        return 0;
    }
    else //格式错误
        return -1;
}
#endif



/*
pos数据读取：BINARY
ret:0读取BINARY
   :-1错误
*/
int pos_data_read(int fd_pos,char *buf,int buf_size)
{
    char head_tmp,data_tmp;
    int head_num;
    int n = 0;
    //puts("pos_data_read");
    while(1)
    {
        //puts("get pos data");
        head_num =  read(fd_pos,&head_tmp,1); //POS data 处理
        if(head_num<0)
        {
            perror("read()");
            return -1;
        }
        if(head_num == 1 && head_tmp == Header_LSB) //pos 发送 PAST1
        {
            head_num =  read(fd_pos,&head_tmp,1);
            if(head_num < 0)
            {
                perror("read()");
                return -1;
            }
            if(head_num == 1 && (head_tmp&0xff) == Header_MSB)
            {
#ifdef DEBUG
                printf("read msg\n");
#endif
                for(n=0;n<buf_size;n++)
                {
                    if(read(fd_pos,&data_tmp,1)==1)
                    buf[n]= data_tmp;
		            else
                    {
 			            perror("read()");
			            return -1;
	       	        }
#ifdef DEBUG
                    printf("%d-%x \n",n,data_tmp);
#endif
                }
		        return 0;
             }
        }
#if 0
        else if(head_tmp == 'V' && head_num == 1)
        {
            memset(buf,'\0',buf_size);
            buf[0] = 'V';
            head_num = read(fd_pos,&head_tmp,1);
            if(head_num == 1 && head_tmp == 'M')
            {
                buf[1] = 'M';
                n=2;
                while(1)
                {
         	        if(read(fd_pos,&data_tmp,1)==1)
                    {
         	            buf[n++] = data_tmp;
                    }
         	        if(data_tmp == '\n')
                    {
                        return 1;
                    }
                }
            }
            else
                continue;
         }
         else
            continue;
    }
    return 1;
#endif
    }
}

/*
 *pos请求读取(ASCII)
 *ret:-1错误
 *   : 0有数据请求
 *   : 1数据有误
 */
int pos_ack_read(int fd_pos,char *buf,int buf_size)
{
    char head_tmp,data_tmp;
    int head_num;
    int n;
    //puts("get pos acq data");
    while(1)
    {
        n=0;
        memset(buf,'\0',buf_size);

        //puts("get pos acq data");
        head_num =  read(fd_pos,&head_tmp,1); //POS data 处理
        if(head_num<0)
        {
            perror("read()");
            return -1;
        }
#if 1
        if(head_tmp == 'V' && head_num == 1)
        {
            memset(buf,'\0',buf_size);
            buf[0] = 'V';
            head_num = read(fd_pos,&head_tmp,1);
            if(head_num == 1 && head_tmp == 'M')
            {
                buf[1] = 'M';
                n=2;
                while(1)
                {
         	        if(read(fd_pos,&data_tmp,1)==1)
                    {
         	            buf[n++] = data_tmp;
                    }
                    else
                        return -1;
         	        if(data_tmp == '\n')
                    {
                        //printf("bufxx = %s",buf);
                        if(strncmp(buf,DATA_ACQ,strlen(DATA_ACQ))==0){
                           // printf("bufyy = %s",buf);
                            return 0;
                        }
                        //else if(strncmp(buf,INITIAL,strlen(INITIAL))==0){
                        //    printf("bufzz = %s",buf);
                        //    pos_data_write(fd_pos,RET,strlen(RET));
                        //    return 0;
                        //}
                        else
                            break;
                    }
                }
            }
            else
                continue;
         }
         else
            continue;
    }
    return 1;
#endif
}

//ret :0 成功
int pos_exit(int fd_pos)
{
    //1.读到pos的数据请求VM081@ /GA 0/   ---DATA_ACQ
    //2.发送平台没有连接的数据VM088A /   ---MOUNT_NO_READY
    //3.读到pos发送VM045H /RC 1/MA 0/    ---INITIAL
    char buf[64];
    char head_tmp,data_tmp;
    int head_num;
    int n;
    //puts("get pos acq data");
    while(1)
    {
        n=0;
        memset(buf,'\0',sizeof(buf));
        head_num =  read(fd_pos,&head_tmp,1); //POS data 处理
        if(head_num<0)
        {
            perror("read()");
            return -1;
        }
#if 1
        if(head_tmp == 'V' && head_num == 1)
        {
            memset(buf,'\0',sizeof(buf));
            buf[0] = 'V';
            head_num = read(fd_pos,&head_tmp,1);
            if(head_num == 1 && head_tmp == 'M')
            {
                buf[1] = 'M';
                n=2;
                while(1)
                {
         	        if(read(fd_pos,&data_tmp,1)==1)
                    {
         	            buf[n++] = data_tmp;
                    }
                    else
                        return -1;
         	        if(data_tmp == '\n')
                    {
                        if(strncmp(buf,DATA_ACQ,strlen(DATA_ACQ))==0)
                        {
                            //printf("bufyy = %s",buf);
                            pos_data_write(fd_pos,MOUNT_NO_READY,strlen(MOUNT_NO_READY));
                            break;
                        }
                        else if(strncmp(buf,INITIAL,strlen(INITIAL))==0)
                        {
                            //printf("bufzz = %s",buf);
                            return 0;
                        }
                        else
                            break;
                    }
                }
            }
            else
                continue;
         }
         else
            continue;
    }
    return 1;
#endif
}

/*
mount数据读取：ASCII
ret:n 读取完成
   :1错误
*/
int mount_data_read(int fd_mount,char *buf,int buf_size)
{
    memset(buf,'\0',buf_size);
    int head_num,n;
    //int flag;
    char head_tmp,data_tmp;
    while((head_num = read(fd_mount,&head_tmp,1)) == 1)
    {
        if(head_tmp == 'V')
        {
             buf[0] = 'V';
             head_num = read(fd_mount,&head_tmp,1);
             if(head_num == 1 && head_tmp == 'M')
             {
                 buf[1] = 'M';
                 n=2;
                 while(1)
                 {
         	        if(read(fd_mount,&data_tmp,1)==1)
                     {
         	            buf[n++] = data_tmp;
                         //printf("buf is %s\n",buf+n);
                         //printf("mount_data_tmp is %d\n",data_tmp);
                     }
         	        if(data_tmp == '\n')
                     {
                        //n=0;
         	            //printf("mount_data_read = %s\n",buf);
                        return n;
                     }
                 }
             }
             else
                 continue;
         }
         else
            continue;
    }
    return 1;
}



/*
pos数据写入：ASCII
ret :0 写正确
    :1 err
*/
int pos_data_write(int fd_pos,const char *buf,int buf_size)
{
    int ret;
    ret = write(fd_pos,buf,buf_size);
    if(ret == buf_size)
        return 0;
    else
        return 1;
}

/*
mount数据写入：ASCII
ret :0 写正确
    :1 err
*/
int mount_data_write(int fd_mount,const char *buf,int buf_size)
{
    int ret;
    ret = write(fd_mount,buf,buf_size);
    if(ret == buf_size)
        return 0;
    else
        return 1;
}


#if 0
/*
 *pos_mount的数据交换
 *
 *
 * */

attitude_t atti_msg;
int mode;
int pos_mount_dataexc(int fd_pos,int fd_mount)
{
    char buff[BUFFER_SIZE];
    char head_tmp,data_tmp;
    int head_num,flag_pos,flag_mount;//msg_num;
    int n = 0;
    //attitude_t atti_msg;
    puts("pos_to_mount");
    while(1)
    {
        //puts("get pos data");
        head_num =  read(fd_pos,&head_tmp,1); //POS data 处理
        if(head_num<0)
        {
            perror("read()");
            return 1;
        }
        if(head_num == 1 && head_tmp == Header_LSB) //pos 发送 PAST1
        {
            //printf("head_num = %d,head_lsb = %x\n",head_num,head_tmp);
            //fflush(stdout);
            head_num =  read(fd_pos,&head_tmp,1);
            //printf("head_num = %d,he、ad_msb = %x\n",head_num,head_tmp);
            if(head_num < 0)
            {
                perror("read()");
                return 1;
            }
            if(head_num == 1 && (head_tmp&0xff) == Header_MSB)
            {
#ifdef DEBUG
                printf("read msg\n");
#endif
                for(n=0;n<30;n++)
                {
                    read(fd_pos,&data_tmp,1);
                    buff[n]= data_tmp;
#ifdef DEBUG
                    printf("%d-%x \n",n,data_tmp);
#endif
                }
                flag_pos = 1;
             }
            else
                continue;
            if(flag_pos)
            {
                pos_data_prc(buff,BUFFER_SIZE,fd_mount,fd_pos,mode,&atti_msg);//pos数据处理
                flag_pos = 0;
                //read_mount(fd_mount,fd_pos);
                continue;
            }
        }
        else if(head_num == 1 && head_tmp == 'V') //pos向平台请求数据
        {
           buff[0] = 'V';
           for(n = 1;n<strlen(DATA_ACQ);n++)
           {
               read(fd_pos,&data_tmp,1);
               buff[n] = data_tmp;
           }
           if(strncmp(DATA_ACQ,buff,strlen(DATA_ACQ) == 0))
           {
                write(fd_mount,DATA_ACQ,strlen(DATA_ACQ));
                flag_mount = 1;
           }
           else
               continue;
           /////////////////////////////////////////////////////////////mount数据回送
           if(flag_mount)
           {
               //mode = read_mount_data(fd_pos,fd_mount,&atti_msg);//判断返回值
               //data_ret(fd_pos,fd_mount);
               flag_mount = 0;
               continue;
           }
        }
    }
    return 0;
}
#endif
