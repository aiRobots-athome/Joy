#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <vector>
#include "tcpip_network.h"

#pragma comment(lib, "Ws2_32.lib")


/* 建構子 (建構時需填入 server 的 ip 和 port number ) */
tcpip_network::tcpip_network()
{
	WSAStartup(MAKEWORD(2, 0), &WSAData);
}

/* 解構子 */
tcpip_network::~tcpip_network()
{
	closesocket(sConnect);
	closesocket(sListen);
	closesocket(server);
	WSACleanup();
	std::cout << " Socket closed. " << std::endl << std::endl;

}

/* 得知目前連線狀態 (-1 : 沒有連線 0 : 有連線) */
int tcpip_network::get_check_connect()
{
	return check_connect;
}

/* 得知目前身分 ( server_use : server , client_use : client ) */
who_network tcpip_network::get_who_use()
{
	return who_use;
}

/* 設定目前身分 ( server_use : server , client_use : client ) */
void tcpip_network::set_who_use(who_network set_value)
{
	if (set_value == server_use || set_value == client_use)
	{
		who_use = set_value;
	}
	else
	{
		std::cout << " Error : invalid set_value of who_use . " << std::endl;
	}
}

/* 打開連線socket ( client 用 ) (連結時需填入 server 的 ip 和 port number) */
void tcpip_network::socketopen_client(PCWSTR server_ipnum, int server_portnum)
{
	if (check_connect == 0)
	{
		socketclose();
	}
	if (check_connect == -1)
	{
		who_use = client_use;

		server_ip = server_ipnum;
		server_port_num = server_portnum;

		server = socket(AF_INET, SOCK_STREAM, 0);
		InetPton(AF_INET, server_ip, &addr.sin_addr.s_addr);
		//addr.sin_addr.s_addr = inet_addr("192.168.0.8"); // replace the ip with your futur server ip address. If server AND client are running on the same computer, you can use the local ip 127.0.0.1
		addr.sin_family = AF_INET;
		addr.sin_port = htons(server_port_num);

		check_connect = connect(server, (SOCKADDR *)&addr, sizeof(addr));

		if (check_connect == 0)
		{
			std::cout << " Connected to server Successful! " << std::endl;
		}
		else if (check_connect == -1)
		{
			std::cout << " Connected to server Failed! " << std::endl;
		}
	}
}

/* 打開連線socket ( server 用 ) (連結時需填入 client 的 ip 和 port number) */
void tcpip_network::socketopen_server(PCWSTR client_ipnum, int client_portnum)
{
	if (check_connect == 0)
	{
		socketclose();
	}
	if (check_connect == -1)
	{
		who_use = server_use;

		server_use_ip = client_ipnum;
		server_use_port_num = client_portnum;

		sConnect = socket(AF_INET, SOCK_STREAM, 0);
		InetPton(AF_INET, server_use_ip, &addr_server_use.sin_addr.s_addr);
		//addr.sin_addr.s_addr = inet_addr("192.168.0.8"); // replace the ip with your futur server ip address. If server AND client are running on the same computer, you can use the local ip 127.0.0.1
		addr_server_use.sin_family = AF_INET;
		addr_server_use.sin_port = htons(server_use_port_num);

		sListen = socket(AF_INET, SOCK_STREAM, 0);
		bind(sListen, (SOCKADDR*)&addr_server_use, sizeof(addr_server_use));
		listen(sListen, SOMAXCONN); /* SOMAXCONN: listening without any limit */

		int addrlen = (sizeof(addr_server_use));
		
		while (check_connect == -1)
		{
			if (sConnect = accept(sListen, (SOCKADDR *)&clinetaddr_server_use, &addrlen))
			{
				check_connect = 0;
				std::cout << " Connected to client Successful! " << std::endl;
			}
		}

		/*if (check_connect == 0)
		{
			std::cout << " Connected to client Successful! " << std::endl;
		}
		else if (check_connect == -1)
		{
			std::cout << " Connected to client Failed! " << std::endl;
		}*/
	}
}


/* 關掉連線socket */
void tcpip_network::socketclose()
{
	if (who_use == server_use)
	{
		closesocket(sConnect);
		closesocket(sListen);	
	}
	else if (who_use == client_use)
	{
		closesocket(server);
	}
	std::cout << "Socket closed." << std::endl << std::endl;
	check_connect = -1;
}

/* 發送指令 */
void tcpip_network::sendmessage(std::string allstr, int strnum)
{
	std::string sendobject = allstr;

	int sendobjectSize = sendobject.size();  /* 得到要傳送的字串的長度 */
	const char* buffer = sendobject.c_str(); /* string 轉成 const char */

	if (who_use == server_use && (check_connect == 0))
	{
		send(sConnect, buffer, sendobjectSize, 0);
	}
	else if (who_use == client_use && (check_connect == 0))
	{
		send(server, buffer, sendobjectSize, 0);  /* 傳送指令 */
	}
}

/* 接收指令 */
std::string tcpip_network::recvmessage()
{
	char buffer_recv[1024] = {};
	if (who_use == server_use && (check_connect == 0))
	{
		recv(sConnect, buffer_recv, sizeof(buffer_recv), 0);  /* 接收指令 */
	}
	else if (who_use == client_use && (check_connect == 0))
	{
		recv(server, buffer_recv, sizeof(buffer_recv), 0);  /* 接收指令 */
	}
	std::string recvobject = {};
	recvobject = buffer_recv;  /* const char 轉成 string */

	return recvobject;
}



/* 合併所有的 strings (combine strings to "~string1,string2,...,stringn@") */
std::string tcpip_network::CombinesString(std::string* allstr, int strnum)
{
	std::string command_wave = { "~" };
	std::string command_comma = { "," };
	std::string command_littlemouse = { "@" };
	
	std::string ans = {};  /* string ans to store the result */

	ans = ans + command_wave; /* add wave~ */
	for (int i = 0; i < strnum; i++)
	{
		if (i != (strnum - 1))
		{
			ans = ans + allstr[i] + command_comma;  /* add string and comma, */
		}
		else if (i == (strnum - 1))  /* add last string  */
		{
			ans = ans + allstr[i];  /* only need to add string (not need to add comma) */
		}
	}
	ans = ans + command_littlemouse;  /* add little mouse @ */

	return ans;

}

/* 將一字串分割成許多段字串 (split string to strings) splitsplit */
std::vector<std::string> tcpip_network::splitsplit(const std::string &s, const std::string &seperator)
{
	std::vector<std::string> result;
	typedef std::string::size_type string_size;  /* 將 string::size_type 型態取一個綽號 string_size */
	string_size i = 0;

	while (i != s.size())
	{
		/* 找到字符串中首個不等於分隔符的字母 */
		int flag = 0;
		while (i != s.size() && flag == 0)
		{
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
			{
				if (s[i] == seperator[x])
				{
					++i;
					flag = 0;
					break;
				}
			}
		}

		/* 找到又一個分隔符，將兩個分隔符之間的字符串取出 */
		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0)
		{
			for (string_size x = 0; x < seperator.size(); ++x)
			{
				if (s[j] == seperator[x])
				{
					flag = 1;
					break;
				}
			}
			if (flag == 0)
			{
				++j;
			}
		}
		if (i != j)
		{
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}
	return result;
}
