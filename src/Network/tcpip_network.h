#pragma once
#ifndef TCPIP_NETWORK_H // __TEST_H__ 由 progrmmer 自己取名
#define TCPIP_NETWORK_H

/* 
TCP/IP 通訊 
使用 Windows 的 API
要將兩台電腦中扮演 Server 的電腦先開 socket , 再開 Client 的 socket
( 兩台電腦要在同一個區網中 )

日期： 2018. 11. 15
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
	tcpip_network();   /* 建構子 */
	~tcpip_network();  /* 解構子 */

	/* 得知目前連線狀態 (-1 : 沒有連線 0 : 有連線) */
	int get_check_connect();
	/* 得知目前身分 ( server_use : server , client_use : client ) */
	who_network get_who_use();
	/* 設定目前身分 ( server_use : server , client_use : client ) */
	void set_who_use(who_network);
	/* 發送指令 */
	void sendmessage(std::string, int);
	/* 接收指令 */
	std::string recvmessage();
	/* 打開連線socket (連結時需填入 server 的 ip 和 port number) */
	void socketopen_client(PCWSTR, int);
	/* 打開連線socket (連結時需填入 server 的 ip 和 port number) */
	void socketopen_server(PCWSTR, int);
	/* 關掉連線socket */
	void socketclose();

private:
	WSADATA WSAData;

	/* 當 client */
	SOCKET server;
	SOCKADDR_IN addr;
	PCWSTR server_ip;  /* the ip number of server (要連上的 server 的 ip )*/
	int server_port_num = 0;  /* the port number of server (要連上的 server 的 port ) */
	/* 當server */
	SOCKET sListen;
	SOCKET sConnect;
	SOCKADDR_IN addr_server_use;
	SOCKADDR_IN clinetaddr_server_use;
	PCWSTR server_use_ip;  /* server 要監聽自己的哪個 ip ( = server 自己的 ip) */
	int server_use_port_num = 0;  /* server 要監聽自己 ip 的哪個 port ( = server 自己要開哪一個 port 給 client ) */

	who_network who_use;
	int check_connect = -1;  /* -1 : 沒有連線 0 : 有連線 */

	/* combine strings to "~string1,string2,...,stringn@" */
	std::string CombinesString(std::string*, int);  /* 猶豫 string[] 要不要改成 vector<string> */
	
	/* split string to strings splitsplit */
	std::vector<std::string> splitsplit(const std::string &s, const std::string &seperator);

};
#endif