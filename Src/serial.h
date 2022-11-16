#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <vector>
#define RECEIVE_SIZE 3
#define SEND_SIZE 5

class Serial {
public:

/**
  * ?????
  * @return ??????
  */

/**
 * ???????
 * @param fd ????????
 * @return ??????
 */
int configurePort(int fd);

/**
 * TX2???????????
 * @return ??????
 */
bool sendCmd(unsigned char data1,unsigned char data2);

bool sendCmd();

/**
 * ????????????????????
 * @param data ????????????
 * @param size ???????
 */
void preVerifyData(const unsigned char *data, size_t size);

/**
 * ??????????
 */
void paraReceiver();

/**
 * ?????????????????????
 * @param array ????????
 */
//void getRealData(unsigned char *array);

/**
   * ????????????
   * @param array ???????
   */
//  void receive(unsigned char *array);

  void openPort();

private:

int _serial;
int i = 0;
unsigned char data[9] = {'1','2','3','4','5','6','7','8','9'};


bool Flag = false;
unsigned char _sendData[SEND_SIZE];
uint8_t _receiveData[RECEIVE_SIZE];
    /**
     * 计算串口线程每秒运行速度
     */


    /**
     * 线程执行体
     */
    
};

#endif
