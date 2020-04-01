#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "../inc/MySerial.hpp"



void Serial::SerialOpen(char *portname, _u32 baudrate) {
    int i = 10;
    if (_rxtx == NULL) {
        _rxtx = rp::hal::serial_rxtx::CreateRxTx();
    }

    while (i--) {
        if (!_rxtx->bind(portname, baudrate) || !_rxtx->open()) {
            printf("/dev/ttyUSB0 open error\n");
            sleep(1);
            continue;
        }
        printf("open /dev/ttyUSB0 success\n");
        return;
    }
}


void Serial::SerialRecv(uint8_t *recvBuffer, size_t recvSize) {
    TranData data;
    size_t DataSize = sizeof(data);
    uint8_t *databuf = (uint8_t * ) & data;
    do {
        recvBuffer[0] = 0;
        if (rp::hal::serial_rxtx::ANS_OK != _rxtx->waitfordata(DataSize, -1, &recvSize)) {
            printf("wait syncode error\n");
            continue;
        }
        _rxtx->recvdata(&recvBuffer[0], DataSize);
//        for (int j=0;j<sizeof(TranData);j++){
//            printf("%x", recvBuffer[j]);
//        }
//        printf("\n");

        if (recvBuffer[0] != 0x55 || recvBuffer[1] != 0xfa || recvBuffer[2] != 0x56 || recvBuffer[3] != 0xfb) {
            printf("recv syn code error\n");
//
//            printf("%x\n", recvBuffer[0]);
//            printf("%x\n", recvBuffer[1]);
//            printf("%x\n", recvBuffer[2]);
//            printf("%x\n", recvBuffer[3]);

            _rxtx->close();

            usleep(1000*100 );
            SerialOpen((char *) "/dev/ttyUSB0", 230400);
            usleep(1000*100 );
            SendData(databuf, DataSize);
            continue;
        }
//
        if (recvBuffer[sizeof(TranData) - 2] != '\r' || recvBuffer[sizeof(TranData) - 1] != '\n') {
            printf("recv data error\n");
//            printf("%x\n", recvBuffer[sizeof(TranData) - 3]);
//            printf("%x\n", recvBuffer[sizeof(TranData) - 2]);
//            printf("%x\n", recvBuffer[sizeof(TranData) - 1]);
//
            continue;
        }
        return;
    } while (1);
}


void Serial::SendData(uint8_t *data, int datasize) {

    _rxtx->senddata(data, datasize);
}
