#ifndef __MY_DATA_PROTO_H__
#define __MY_DATA_PROTO_H__


#pragma pack(1)//因为串口是一个字节一个字节发送数据的，所以在存放消息时要按照字节对齐
typedef struct data
{

    uint8_t start;//开始码定义为255
    uint8_t start1;//开始码定义为255
    uint8_t start2;//开始码定义为255
    uint8_t start3;//开始码定义为255


    float Communicate[16];
    uint8_t flag;
    uint8_t sum;
    //预留结束码
    uint8_t stop_1;//结束码1定义为'\r'
    uint8_t stop_2;//结束码2定义为'\n'
}TranData;

#pragma pack()//恢复系统默认字节对齐
#endif


