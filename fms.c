/*************************************************************************
	> File Name: fms.c
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
#include <signal.h>

#define MAX_COM_NUM 3
#define MOUNT_COM   1
#define POS_COM     2
#define NON_PARITY 'S'
#define BUFFER_SIZE 30
#define Header_LSB 0x00
#define Header_MSB 0x93

//static const char* POS_DATA_ACQ   = "VM081@ /GA 0/\r\n";

//#define RE_SEND          1
//#define NEW_SEND         0

#define POS_BUF_SIZE 30

#define YAW_GAIN         1         //YAW角的增益
#define CUT_TIME         1        //0.5秒
#define FRQ              20        //POS数据的更新速率HZ
//#define RING_SIZE  (CUT_TIME*FRQ)/2  //（中滤波值取奇数）,RING_SIZE开窗大小----->（HZ*TIME(s)）
#define RING_SIZE        10

enum {
    MAN_MODE,
    STAB_MODE
};

enum {
    NEW_SEND,
    RE_SEND,
    NO_SPEED
};

static int fd_pos,fd_mount,tmp;

static int task_mode;//0:man模式
                     //1:stab模式

//飞行前
//GA 10 MST 10 获取平台姿态数据 （发给平台）
//RC 1 MA 0    设置平台工作模式

//飞行开始进入航线
//SCA 1     设置为外部数据控制
// ST 1     不控制航向角
// 发送三个角度

//飞行转弯
//MA 0
//AYN 0

//飞行结束
//RC 0
//
//
/***********************************
*分为两种工作模式，航线中，航线外
*1.航线中STAB模式
   在STAB模式下，发送角度为POS传输的角度值

*2.航线外为MAN模式
    在MAN模式中，始终发送的三个角度值为0
*
*
* **********************************/



/*Ctrl+c                切换模式*/
void sighandler1(int unuse)
{
    tmp++;
    if(tmp%2 != 0)
    {
        mount_auto(fd_mount);
        //usleep(10);
        //mount_fl(fd_mount);
        //printf("auto\n");
        task_mode = STAB_MODE;
        printf("****************航线中：STAB MODE********************\n");
    }
    else if(tmp%2 == 0)
    {
        mount_man(fd_mount);
        usleep(10);
        mount_fl(fd_mount);
        //printf("man and FL\n");
        task_mode = MAN_MODE;
        printf("****************航线外：MAN MODE*********************\n");
    }
#if 0
    if(task_mode)
        printf("****************航线中：STAB MODE********************\n");
    else
        printf("****************航线外：MAN MODE*********************\n");
#endif
}

/*Ctrl+\               退出程序*/
void sighandler2(int unuse)
{
    while (pos_exit(fd_pos) ==0 ){
        mount_man(fd_mount);
        usleep(10);
        mount_fl(fd_mount);
        usleep(10);
        mount_table(fd_mount);
        close(fd_pos);
        close(fd_mount);
        printf("\nexit\n");
        exit(0);
    }
}

int main(int argc,const char **argv)
{
    //1.配置串口，打开串口 分别打开两个串口，可用两个线程
    //2.读POS，---->处理格式------->写MOUNT_COM
    //3.读MOUNT_COM,存为POS-gimbal格式

    /**************OPEN POS_COM**************/
    fd_pos = open_port(POS_COM);
    if(fd_pos<0)
    {
        fprintf(stderr,"Open POS_COM ERR!\n");
        exit(1);
    }

    //pos的串口设置 115200， 8bit， 无校验，1bit停止位
    int ret = set_com_config(fd_pos,115200,8,NON_PARITY,1);
    if(ret<0)
    {
        fprintf(stderr,"set com_pos err\n");
        exit(1);
    }
        /***************OPEN MOUNT_COM***************/
    fd_mount = open_port(MOUNT_COM);
    if(fd_mount<0)
    {
        fprintf(stderr,"Open MOUNT_COM ERR!\n");
        exit(1);
    }
     /*mount的串口设置 115200， 8bit， 无校验，1bit停止位*/
    ret = set_com_config(fd_mount,115200,8,NON_PARITY,1);
    if(ret<0)
    {
        fprintf(stderr,"set com_mount err\n");
        exit(1);
    }
    /**************MOUNT握手**********/
#if 1
    tcflush(fd_mount, TCIFLUSH);
    while(1)
    {
        if(init_mount(fd_mount)==0)
        {
            printf("MOUNT conect\n");
            break;
        }
    }
#endif
    /*MOUNT设置增益*/
    while(1)
    {
        if (mount_set_spa(fd_mount) == 0)
        {
            if(mount_set_spd(fd_mount) == 0)
            {
                printf("mount set ok\n");
                break;
            }
            else
                continue;
        }
        else
            continue;
    }

    /***************与POS握手************/
#if 1
    tcflush(fd_pos, TCIFLUSH);
    while(1)
    {
        if(init_pos(fd_pos)==0)
        {
            printf("POS conect\n");
            break;
        }
    }
#endif


    printf("set up ok!\n");

    signal(SIGINT,sighandler1); //注册信号函数，auto/man控制
    signal(SIGQUIT,sighandler2);//注册信号函数，退出控制

    /*开机将MOUNT切换至MAN模式*/
    mount_man(fd_mount);

    //与mount握手后，持续发送控制角度和数据请求

    //pos_to_mount(fd_pos,fd_mount,0);
    //pos_mount_dataexc(fd_pos,fd_mount);

    char pos_buf[POS_BUF_SIZE];
    int yaw_mode = 0;//NEW_SEND:0  RE_SEND:1 NO_SPEED:2
    int yaw_tmp;
    attitude_t atti_msg;
   // static int tail = 0;
   // static int flag = 0;
   // static int real_dir_buf[RING_SIZE];
    char mount_buf[128];
    char pos_ack_buf[128];
    //int data_acq_flag = 0;
    //int data_acq_flag = 1;
    //ring_array变量
    int tail_cnt = 0;
    int yaw_mount = 0;
    int real_dir_buf[RING_SIZE]={0};
    int tail=0;
    int flag=0;

    while(1)
    {
        //读POS
        tcflush(fd_pos, TCIFLUSH);
        if(pos_data_read(fd_pos,pos_buf,POS_BUF_SIZE)<0)
            continue;
        //printf("read pos\n");

       //处理POS数据（PAST1解析+打包+VM@GA0）

        pos_data_parse(pos_buf,POS_BUF_SIZE,&atti_msg);//PAST1解析
        //snprinf(PAD)
        atti_msg.yaw  = ayn_angle(atti_msg.yaw,yaw_mount);//计算转动角度 AYN
        atti_msg.roll  = roll_angle(atti_msg.roll);
        atti_msg.pitch  = pitch_angle(atti_msg.pitch);


        //判断速度
        if(speed_is_low(atti_msg.speed)){
            yaw_mode = NO_SPEED;
        }

        //ring_array
#if 1
        tail = (tail)%RING_SIZE;
        tail_cnt++;
        real_dir_buf[tail++] = atti_msg.yaw;
        //printf("atti_msg_yaw = %d\n",atti_msg.yaw);
        //int i;
        //for(i = 0; i < RING_SIZE; i++)
        //printf("real_dir_buf[%d] = %d\n",i,real_dir_buf[i]);
#if 0
        if(tail_cnt >= RING_SIZE)
        {
            tail_cnt = RING_SIZE;
            flag = 1;
            for(i = 0; i < RING_SIZE; i++)
                printf("real_dir_buf[%d] = %d\n",i,real_dir_buf[i]);
        }
        else
            continue;
#endif
#endif
        flag = 1;

        if(flag)
        {
	        //atti_msg.yaw = filter_mean(real_dir_buf,RING_SIZE) * YAW_GAIN;//滤波
            //printf("filter_yaw = %d\n",atti_msg.yaw);

            //判断模式
            if(yaw_mode == NEW_SEND){
                yaw_tmp = atti_msg.yaw;
                //printf("YAW_MODE IS NEW_SEND\n");
            }
            else if(yaw_mode == RE_SEND){
                atti_msg.yaw = yaw_tmp;
                //printf("YAW_MODE IS RE_SEND\n");
            }
            else if(yaw_mode == NO_SPEED){
                atti_msg.yaw = 0;
                //printf("YAW_MODE IS NO_SPEED\n");
            }
            //printf("pitch = %d,roll = %d,yaw = %d,speed = %d\n",atti_msg.pitch,atti_msg.roll,atti_msg.yaw,atti_msg.speed);
            //printf("yaw = %d,speed = %d\n",atti_msg.yaw,atti_msg.speed);

            char *send = NULL;
	        int num = pos_data_packet(&atti_msg,&send);       //VM格式打包
            if(task_mode == MAN_MODE)                         //写平台数据
            {
                tcflush(fd_mount, TCOFLUSH);
                mount_data_zero(fd_mount);
                //printf("***************MAN_MODE*******************\n");
            }
            else if(task_mode == STAB_MODE)
            {
                tcflush(fd_mount, TCOFLUSH);
                mount_data_write(fd_mount,send,num);
                //printf("**************STAB_MODE :%s",send);
            }
            free(send);

	        mount_data_acq(fd_mount);//平台数据请求
                //读MOUNT **必须读到有效数据***
            tcflush(fd_mount, TCIFLUSH);
            while((num = mount_data_read(fd_mount,mount_buf,128))<20)
            {
	            mount_data_acq(fd_mount);
                tcflush(fd_mount, TCIFLUSH);
            }
            //num = mount_data_read(fd_mount,mount_buf,128);
            //printf("mount_data_read is %s\n",mount_buf);

            if(mount_date_parse(mount_buf)==0)//判断平台数据
            {
                //printf("mount data parse\n");
                //tcflush(fd_pos, TCIFLUSH);//清除缓冲区
#if 1
                while(1)
                // while(pos_ack_read(fd_pos,pos_ack_buf,128) == 0)
                //if(data_acq_flag)
                {
                    int tmp_cnt = 0;
                    int tmp = pos_ack_read(fd_pos,pos_ack_buf,128);
                    tmp_cnt++;
                    if(tmp == 0){
                    //写POS（MOUNT数据转发）
                        tcflush(fd_pos, TCOFLUSH);
                        pos_data_write(fd_pos,mount_buf,num);
                        //printf("pos_data_write :%s",mount_buf);
                        break;
                    }
                    else if(tmp_cnt>10)
                    {
                        tcflush(fd_pos, TCOFLUSH);
                        pos_data_write(fd_pos,mount_buf,num);
                        //printf("pos_data_write :%s",mount_buf);
                        break;
                    }
                   //else
                   // continue;
                }
#endif
                //printf("mount_data_read = %s\n",mount_buf);
                //printf("num = %d\n",num);
                //处理MOUNT数据（VM@GR/GP/GY/解析+处理YAW角重发标志位）,得到平台的实时角度
                yaw_mode = mount_date_prc(mount_buf,num,&atti_msg,&yaw_mount);
                //printf("end\n");
                if(yaw_mode<0)
                {
                    printf("MOUNT_DATA_ERR\n");
                    continue;
                }
            }
        }
        else
            continue;
        //num = mount_data_read(fd_mount,mount_buf,128);
        //printf("mount_data_read num is %d\n",num);
        //ret = memcmp(mount_buf,"VM081H /\r\n",sizeof("VM081H /\r\n"));
        //printf("retH = %d\n",ret);
        //ret = memcmp(mount_buf,"VM082G /\r\n",sizeof("VM082G /\r\n"));
        //printf("retG = %d\n",ret);
        //if(memcmp(mount_buf,"VM081H /\r\n",sizeof("VM081H /\r\n")!=0))
    }
    return 0;
}





//thread1 ---- 接收POS信息（位置，姿态）
//thread2 ---- 处理姿态信息
//thread3 ---- 处理位置信息
//thread4 ---- 通过TCP发送/接受信息 （做主机）
//thread5 ---- 曝光控制
//main thread ---初始化工作 ：处理航线，判断位置，创建进程......

//pthread_create         创建新的控制流
//pthread_exit           从现有控制流中退出
//pthread_join           从控制流中得到退出状态
//pthread_cancel_push    注册在退出控制流时调用的函数
//pthread_self           获得控制流ID
//pthread_cancel         请求控制流的非正常退出

//thread_input(void *arg)
//{


//}


//thread_ctrl_mount(void *arg)
//{


//}

//thread_posdata(void *arg)
//{


//}
