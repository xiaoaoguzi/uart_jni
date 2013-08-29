#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <jni.h>

int fd;

#define  NUM(X)    (sizeof(X)/sizeof(X[0]))

int  Java_wujiwang_jr_Serial_OpenSerial(JNIEnv *env, jclass thiz,jstring dev, int rate)
{
	struct termios options;
	int i;
	int rel_rate[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400};
	int sys_rate[] = {B4800, B9600, B19200, B38400, B57600, B115200, B230400};

	const char *dev_dir = (*env)->GetStringUTFChars(env, dev, NULL);
	if(dev_dir == NULL)
	{
		return -1;
	}

    fd = open(dev_dir, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0)
    {
        perror("open");
        return -2;
    }

    fcntl(fd, F_SETFL, 0); // set block
    /////set serial//////////////////////////////


    tcgetattr(fd, &options);
	
	for(i = 0; i < NUM(rel_rate); i++)
	{
		if(rel_rate[i] == rate)
		{
    		cfsetispeed(&options, sys_rate[i]);
    		cfsetospeed(&options, sys_rate[i]);
			break;
		}
	}
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~CRTSCTS; //disable hw flow control
    options.c_cflag |= CREAD | CLOCAL;

	tcsetattr(fd, TCSANOW, &options);
	/////////////////////////////////////

	return fd;
}

//读串口
// buf 读到哪里去 需要一块空间 比如数组
// num 读多少byte
// 返回实际读到的个数
int  Java_wujiwang_jr_Serial_ReadSerial(JNIEnv *env, jclass thiz, jstring *buf,int num)
{
	int ret;
	const char *str = (*env)->GetStringUTFChars(env, buf, NULL);
	if(str == NULL)
	{
		return -1;
	}

	ret = read(fd, buf, num);

	return ret;
}

// 写串口
// buf 从哪里取数据 要写的字符串 数组等
// num 写多少byte
// 返回实际写的个数
int  Java_wujiwang_jr_Serial_WriteSerial(JNIEnv *env, jclass thiz, jstring *buf,int num)
{
	int ret;
	const char *str = (*env)->GetStringUTFChars(env, buf, NULL);
	if(str == NULL)
	{
		return -1;
	}

	ret = write(fd, buf, num);

	return ret;
}

//关闭串口
void  Java_wujiwang_jr_Serial_CloseSerial(JNIEnv *env, jclass thiz)
{
	close(fd);
}

