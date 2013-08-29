#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <jni.h>

int fd;

// 打开传开并初始化串口
// dev 设备绝对地址8N1
// rate 波特率
int  Java_wujiwang_jr_Serial_OpenSerial(JNIEnv *env, jclass thiz,jstring dev, unsigned int rate);
//读串口
// buf 读到哪里去 需要一块空间 比如数组
// num 读多少byte  返回实际读到的个数
int  Java_wujiwang_jr_Serial_ReadSerial(JNIEnv *env, jclass thiz,jstring *buf,int num);

// 写串口
// buf 从哪里取数据 要写的字符串 数组等
// num 写多少byte 返回实际写的个数
int  Java_wujiwang_jr_Serial_WriteSerial(JNIEnv *env, jclass thiz, jstring *buf,int num);

// 关闭串口
void  Java_wujiwang_jr_Serial_CloseSerial(JNIEnv *env, jclass thiz);

