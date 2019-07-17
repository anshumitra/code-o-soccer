//	Warning! Do not modify this file
/*	Read for fun																*
*	This file implements socket communication between client and server			*
*	communication works via TCP communication on port: 8888						*/

#pragma once
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
class server
{
	server(void);
	~server(void);
	SOCKET SocketServer; 
	void RecieveMessage();
};