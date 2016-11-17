/*************************************************************************
	> File Name:pos_prc.h
	> Author:renk
	> Mail:door_110@163.com
	> Created Time: 2016年08月24日 星期三 13时42分40秒
 ************************************************************************/
#ifndef __POS_GSM_H__
#define __POS_GSM_H__


typedef struct{
    short pitch;
    short roll;
    unsigned short speed;
    int   yaw;
    long latitude;
    long longitude;
    long altitude;
    double time;                    // UTC seconds of week or elapsed seconds if GPS not available
}attitude_t;                        //姿态信息

int set_com_config(int fd,int baud_rate,int data_bits,char parity,int stop_bits);//设置串口
int open_port(int com_port);//打开串口
int init_mount(int fd);//mount handshake
int init_pos(int fd_pos);//pos handshake

int mount_auto(int fd_mount);//平台指令
int mount_man(int fd_mount);
int mount_fl(int fd_mount);
int mount_table(int fd_mount);
int mount_data_acq(int fd_mount);
int mount_data_zero(int fd_mount);
int mount_set_spa(int fd_mount);
int mount_set_spd(int fd_mount);

int pos_data_read(int fd_pos,char *buf,int buf_size);//数据读写
int pos_ack_read(int fd_pos,char *buf,int buf_size);
int mount_data_read(int fd_mount,char *buf,int buf_size);
int pos_data_write(int fd_pos,const char *buf,int buf_size);
int mount_data_write(int fd_mount,const char *buf,int buf_size);

void pos_data_parse(void * buff,int buff_size,attitude_t *atti_msg);//PAST1解析
//int  pos_data_packet(attitude_t *atti_msg, int mode, char **send,int yaw_old);      //VM格式打包
int pos_data_packet(attitude_t *atti_msg, char **send);//VM格式打包
int pos_exit(int fd_pos);

//int pos_data_prc(void * buff,int buff_size,int fd_mount,int fd_pos,int mode,attitude_t *atti_msg);
int mount_date_prc(const char *tmp,int len,const attitude_t *atti_msg,int *yaw_mount);
int mount_date_parse(const char *tmp);

int pos_mount_dataexc(int fd_pos,int fd_mount);
int fms_to_mount(int fd_pos,int fd_mount,int mode);//FMS发送控制平台的数据，
//mode为状态(0--初始化，1---起飞，2---航线中，3---转弯，4---结束)
int filter_mean(int *val,int len);
int speed_is_low(int speed);
int ayn_angle(int ay_angle,int mount_yaw);
short roll_angle(short mount_roll);
short pitch_angle(short pitch_roll);



//int pos_to_mount(int fd_pos,int fd_mount,int mode);//pos数据处理打包
//int real_val(char msb,char lsb,int mode);//mode = 0 (0 <--> 360)   mode = 1 (-180 <-->- +180)
//short real_val(char msb,char lsb);
//int data_prc(void * buff,int buff_size,int fd_mount,int fd_pos);
//int data_prc_pitch(void * buff,int buff_size,int fd);
//int packet(char *msg,int len,char **res);
//int read_pos_data(int fd);
//int mount_data(int fd_mount);
//int pos_req(int fd_pos);
//int data_ret(int fd_pos,int fd_mount);
#endif
