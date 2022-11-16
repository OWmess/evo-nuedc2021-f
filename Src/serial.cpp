#include "serial.h"

#define DEBUG_SERIAL_CHANGE 0
#define DEBUG_SERIAL_NO_CHANGE 0
#define SHOW_SERIAL_TIME 0
#define DEBUG_TX2_SEND 0
#define DEBUG_TX2_SEND_NO_CHANGE 0

int Serial::configurePort(int fd) {
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);
    /* Enable the receiver and set local mode...*/
    port_settings.c_cflag |= (CLOCAL | CREAD);
    /* Set c_cflag options.*/
    port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~PARODD;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;
    //port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    port_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
    /* Set c_oflag output options */
    port_settings.c_oflag &= ~OPOST;
    /* Set the timeout options */
    port_settings.c_cc[VTIME] = 0;
    port_settings.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
    return fd;
}

bool Serial::sendCmd(unsigned char data1,unsigned char data2){
    _sendData[0] = 0x3a;
    _sendData[1] = 0x3b;
    _sendData[2] = data1;
    _sendData[3] = data2;
    _sendData[4] = 0x3c;
    if(5 == write(_serial, _sendData, 5)){
    }
    return true;
}

void Serial::paraReceiver() {
    uint8_t buf[255] = {0};
    size_t bytes = 0;
    ioctl(_serial, FIONREAD, &bytes);

    static int noDataNum = 0;
    if(bytes == 0){
        noDataNum++;
    }

    //std::cout << "in"<< read(_serial, (unsigned char *) buf, 32)<< std::endl;
    read(_serial, (unsigned char *) buf, 32);

    if (bytes > 32) {
        bytes = read(_serial, (unsigned char *) buf, 32);
    } else if (bytes <= 32) {
        bytes = read(_serial, (unsigned char *) buf, bytes);
    }

    //????��?��?????
    preVerifyData(buf, bytes);
}

void Serial::preVerifyData(const unsigned char *data, size_t size) {
    std::vector< uint8_t > _FIFOBuffer;
    if (size > 0) {
        for (int i = 0; i < size; i++) {
            _FIFOBuffer.push_back(data[i]);
        }
    }
    //std::cout << "in" << std::endl;
    for(i = 0; i < _FIFOBuffer.size(); i++){
        //std::cout << "data: " << (int)_FIFOBuffer[i]<< std::endl;
        if(_FIFOBuffer[i] == 0x4D){

            std::cout << "in" << std::endl;
            Flag = true;
            break;
        }
    }

    if(Flag){
        for(int j = 0; j < RECEIVE_SIZE; j++,i++){

            _receiveData[j] = _FIFOBuffer[i];

            if(_receiveData[j] == 0xA3)
            {
                std::cout<<"is 0xA3"<<std::endl;
                break;
            }

            if(j != 0)
                std::cout <<_receiveData[j] - 48 << std::endl;

          // std::cout << "receiveData: " << _receiveData[1] -48 << std::endl;
//            std::cout << "receiveData: " << _receiveData[2] - 48<< std::endl;
        }
        Flag = false;
        //i = 0;
    }

}


void Serial::openPort() {
access("/dev/ttyAMA0", F_OK);
int fd = -1;
_serial = open("/dev/ttyAMA0",  O_RDWR | O_NOCTTY | O_NDELAY);
if(_serial == -1){
  std::cout << "port is not open !" << std::endl;
  return;
}
else{
  fcntl(_serial, F_SETFL, FNDELAY);
  configurePort(_serial);
  std::cout << "port is open !" << std::endl;
}

}
