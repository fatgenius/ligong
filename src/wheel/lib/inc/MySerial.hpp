#ifndef __MYSERIAL_H__
#define __MYSERIAL_H__

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "net_serial.hpp"
#include "types.hpp"
#include "MyDataProto.hpp"

class Serial
{
	private:
	
	rp::hal::serial_rxtx *_rxtx = NULL;
	
	
	public:
	
	void SerialOpen(char *portname,_u32 baudrate);
	void SerialRecv(uint8_t *recvBuffer,size_t recvSize);
	void imuSerialRecv(uint8_t *recvBuffer,size_t recvSize);
	void SendData(uint8_t *data,int datasize);
	
};















#endif


