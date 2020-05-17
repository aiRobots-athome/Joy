#pragma once
#ifndef TCPIP_NETWORK_H // __TEST_H__ �� progrmmer �ۤv���W
#define TCPIP_NETWORK_H

/* 
TCP/IP �q�T 
�ϥ� Windows �� API
�n�N��x�q������t Server ���q�����} socket , �A�} Client �� socket
( ��x�q���n�b�P�@�ӰϺ��� )

����G 2018. 11. 15
*/

#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <vector>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

enum who_network
{
	server_use,
	client_use
};

class tcpip_network
{
public:
	tcpip_network();   /* �غc�l */
	~tcpip_network();  /* �Ѻc�l */

	/* �o���ثe�s�u���A (-1 : �S���s�u 0 : ���s�u) */
	int get_check_connect();
	/* �o���ثe���� ( server_use : server , client_use : client ) */
	who_network get_who_use();
	/* �]�w�ثe���� ( server_use : server , client_use : client ) */
	void set_who_use(who_network);
	/* �o�e���O */
	void sendmessage(std::string, int);
	/* �������O */
	std::string recvmessage();
	/* ���}�s�usocket (�s���ɻݶ�J server �� ip �M port number) */
	void socketopen_client(PCWSTR, int);
	/* ���}�s�usocket (�s���ɻݶ�J server �� ip �M port number) */
	void socketopen_server(PCWSTR, int);
	/* �����s�usocket */
	void socketclose();

private:
	WSADATA WSAData;

	/* �� client */
	SOCKET server;
	SOCKADDR_IN addr;
	PCWSTR server_ip;  /* the ip number of server (�n�s�W�� server �� ip )*/
	int server_port_num = 0;  /* the port number of server (�n�s�W�� server �� port ) */
	/* ��server */
	SOCKET sListen;
	SOCKET sConnect;
	SOCKADDR_IN addr_server_use;
	SOCKADDR_IN clinetaddr_server_use;
	PCWSTR server_use_ip;  /* server �n��ť�ۤv������ ip ( = server �ۤv�� ip) */
	int server_use_port_num = 0;  /* server �n��ť�ۤv ip ������ port ( = server �ۤv�n�}���@�� port �� client ) */

	who_network who_use;
	int check_connect = -1;  /* -1 : �S���s�u 0 : ���s�u */

	/* combine strings to "~string1,string2,...,stringn@" */
	std::string CombinesString(std::string*, int);  /* �S�� string[] �n���n�令 vector<string> */
	
	/* split string to strings splitsplit */
	std::vector<std::string> splitsplit(const std::string &s, const std::string &seperator);

};
#endif