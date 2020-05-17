#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <vector>
#include "tcpip_network.h"

#pragma comment(lib, "Ws2_32.lib")


/* �غc�l (�غc�ɻݶ�J server �� ip �M port number ) */
tcpip_network::tcpip_network()
{
	WSAStartup(MAKEWORD(2, 0), &WSAData);
}

/* �Ѻc�l */
tcpip_network::~tcpip_network()
{
	closesocket(sConnect);
	closesocket(sListen);
	closesocket(server);
	WSACleanup();
	std::cout << " Socket closed. " << std::endl << std::endl;

}

/* �o���ثe�s�u���A (-1 : �S���s�u 0 : ���s�u) */
int tcpip_network::get_check_connect()
{
	return check_connect;
}

/* �o���ثe���� ( server_use : server , client_use : client ) */
who_network tcpip_network::get_who_use()
{
	return who_use;
}

/* �]�w�ثe���� ( server_use : server , client_use : client ) */
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

/* ���}�s�usocket ( client �� ) (�s���ɻݶ�J server �� ip �M port number) */
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

/* ���}�s�usocket ( server �� ) (�s���ɻݶ�J client �� ip �M port number) */
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


/* �����s�usocket */
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

/* �o�e���O */
void tcpip_network::sendmessage(std::string allstr, int strnum)
{
	std::string sendobject = allstr;

	int sendobjectSize = sendobject.size();  /* �o��n�ǰe���r�ꪺ���� */
	const char* buffer = sendobject.c_str(); /* string �ন const char */

	if (who_use == server_use && (check_connect == 0))
	{
		send(sConnect, buffer, sendobjectSize, 0);
	}
	else if (who_use == client_use && (check_connect == 0))
	{
		send(server, buffer, sendobjectSize, 0);  /* �ǰe���O */
	}
}

/* �������O */
std::string tcpip_network::recvmessage()
{
	char buffer_recv[1024] = {};
	if (who_use == server_use && (check_connect == 0))
	{
		recv(sConnect, buffer_recv, sizeof(buffer_recv), 0);  /* �������O */
	}
	else if (who_use == client_use && (check_connect == 0))
	{
		recv(server, buffer_recv, sizeof(buffer_recv), 0);  /* �������O */
	}
	std::string recvobject = {};
	recvobject = buffer_recv;  /* const char �ন string */

	return recvobject;
}



/* �X�֩Ҧ��� strings (combine strings to "~string1,string2,...,stringn@") */
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

/* �N�@�r����Φ��\�h�q�r�� (split string to strings) splitsplit */
std::vector<std::string> tcpip_network::splitsplit(const std::string &s, const std::string &seperator)
{
	std::vector<std::string> result;
	typedef std::string::size_type string_size;  /* �N string::size_type ���A���@�Ӻ︹ string_size */
	string_size i = 0;

	while (i != s.size())
	{
		/* ���r�Ŧꤤ���Ӥ�������j�Ū��r�� */
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

		/* ���S�@�Ӥ��j�šA�N��Ӥ��j�Ť������r�Ŧ���X */
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
