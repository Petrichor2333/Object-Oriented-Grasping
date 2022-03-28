#include "pch.h"
#include "ControlSystem.h"

ControlSystem::ControlSystem() {
	arm_ip_ = "0.0.0.0";
	arm_port_ = 0;
	hand_ip_ = "0.0.0.0";
	hand_port_ = 0;
}

ControlSystem::ControlSystem(string arm_ip, unsigned short arm_port, string hand_ip, unsigned short hand_port) {
	arm_ip_ = arm_ip;
	arm_port_ = arm_port;
	hand_ip_ = hand_ip;
	hand_port_ = hand_port;

	//加载套接字库
	WSADATA wsaDataArm, wsaDataHand;
	int iRetArm = 0, iRetHand = 0;

	iRetArm = WSAStartup(MAKEWORD(2, 2), &wsaDataArm);
	iRetHand = WSAStartup(MAKEWORD(2, 2), &wsaDataHand);
	try {
		if (iRetArm != 0 || iRetHand != 0)
		{
			throw "WSAStartup(MAKEWORD(2, 2), &wsaData) execute failed!";
		}
	}
	catch (const char* msg) {
		cerr << msg << endl;
		exit(90);
	}
	cout << "WSAStartup ok" << endl;

	try {
		if (2 != LOBYTE(wsaDataArm.wVersion) || 2 != HIBYTE(wsaDataArm.wVersion) || 2 != LOBYTE(wsaDataHand.wVersion) || 2 != HIBYTE(wsaDataHand.wVersion))
		{
			WSACleanup();
			throw "WSADATA version is not correct!";
		}
	}
	catch (const char* msg) {
		cerr << msg << endl;
		exit(91);
	}


	//创建套接字
	clientSocketArm_ = socket(AF_INET, SOCK_STREAM, 0);
	clientSocketHand_ = socket(AF_INET, SOCK_STREAM, 0);
	try {
		if (clientSocketArm_ == INVALID_SOCKET || clientSocketHand_ == INVALID_SOCKET)
		{
			closesocket(clientSocketArm_);
			closesocket(clientSocketHand_);
			throw "clientSocket = socket(AF_INET, SOCK_STREAM, 0) execute failed!";
		}
	}
	catch (const char* msg) {
		cerr << msg << endl;
		exit(92);
	}
	cout << "WSAStartup ok" << endl;


	//初始化服务器端地址族变量
	SOCKADDR_IN srvAddrArm, srvAddrHand;
	inet_pton(AF_INET, arm_ip_.c_str(), &srvAddrArm.sin_addr);
	inet_pton(AF_INET, hand_ip_.c_str(), &srvAddrHand.sin_addr);
	srvAddrArm.sin_family = AF_INET;
	srvAddrHand.sin_family = AF_INET;
	srvAddrArm.sin_port = htons(arm_port_);
	srvAddrHand.sin_port = htons(hand_port_);

	//连接服务器
	iRetArm = connect(clientSocketArm_, (SOCKADDR*)&srvAddrArm, sizeof(SOCKADDR));
	iRetHand = connect(clientSocketHand_, (SOCKADDR*)&srvAddrHand, sizeof(SOCKADDR));
	try {
		if (0 != iRetArm || 0 != iRetHand)
		{
			closesocket(clientSocketArm_);
			closesocket(clientSocketHand_);
			throw "connect(clientSocket, (SOCKADDR*)&srvAddr, sizeof(SOCKADDR)) execute failed!";
		}
	}
	catch (const char* msg) {
		cerr << msg << endl;
		exit(92);
	}
	cout << "connect ok" << endl;


	char sendBuf[100] = { '\0' }, recvBuf[100] = { '\0' };
	int sig;
	sprintf_s(sendBuf, "%s", "hello hand");
	send(clientSocketHand_, sendBuf, strlen(sendBuf) + 1, 0);
	sig = recv(clientSocketHand_, recvBuf, 100, 0);
	if (sig <= 0)
	{
		printf("Error: Hand lost connection!");
	}
	printf("%s\n", recvBuf);

	sprintf_s(sendBuf, "%s", "HelloArm");
	send(clientSocketArm_, sendBuf, strlen(sendBuf) + 1, 0);
	sig = recv(clientSocketArm_, recvBuf, 100, 0);
	if (sig <= 0)
	{
		printf("Error: Arm lost connection!\n");
	}
	printf("%s\n", recvBuf);

}

void ControlSystem::MoveAsGraspMethod(GraspMethod GspMethod) {
	cout << "start grasp Method"<<endl;
	int i;
	for (i = 0;i < GspMethod.GetControlCommands().size();i++) {
		ControlCommand command = GspMethod.GetControlCommands()[i];
		char sendBuf[100] = { '\0' }, recvBuf[100] = { '\0' };
		int sig;
		Device device = command.GetDevice();
		string command_type = command.GetCommandType();
		vector<double> data = command.GetData();
		int sleep_time = command.GetSleepTime();
		if (device == Arm) {
			sprintf_s(sendBuf, "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", command_type.c_str(), data[0], data[1], data[2], data[5], data[4], data[3]);//此处需要转化为机械臂接受数据的格式:XYZ
			send(clientSocketArm_, sendBuf, strlen(sendBuf) + 1, 0);			
			if (sleep_time != 0) {
				Sleep(sleep_time);
			}
			sig = recv(clientSocketArm_, recvBuf, 100, 0);
			if (sig <= 0)
			{
				printf("Error: Arm lost connection!\n");
			}
			printf("%s\n", recvBuf);
			TranslationOfRobotbaseToCurrentHand_ << data[0], data[1], data[2];
			EularAngleOfRobotbaseToCurrentHand_ << data[3], data[4], data[5];
		}
		else if (device == Hand) {
			sprintf_s(sendBuf, "%s,%d,%d,%d,%d,%d,%d", command_type.c_str(), (int)(data[0]), (int)(data[1]), (int)(data[2]), (int)(data[3]), (int)(data[4]), (int)(data[5]));
			send(clientSocketHand_, sendBuf, strlen(sendBuf) + 1, 0);
			if (sleep_time != 0) {
				Sleep(sleep_time);
			}
			sig = recv(clientSocketHand_, recvBuf, 100, 0);
			if (sig <= 0)
			{
				printf("Error: Hand lost connection!\n");
			}
			printf("%s\n", recvBuf);
			CurrentPosOfHand_.clear();
			for (int i = 0;i < 6;i++) 
			{
				CurrentPosOfHand_.push_back((int)(data[i]));
			}
		}
		else {
			printf("Error: Device Undefined!\n");
		}
	}
}

void ControlSystem::MoveToInitState()
{
	char sendBuf[100] = { '\0' }, recvBuf[100] = { '\0' };
	int sig;

	char command1[100] = "SetSpeed,2000,2000,2000,2000,2000,2000";
	strcpy_s(sendBuf, command1);
	send(clientSocketHand_, sendBuf, strlen(sendBuf) + 1, 0);
	Sleep(10);
	sig = recv(clientSocketHand_, recvBuf, 100, 0);
	if (sig <= 0)
	{
		printf("Error: Hand lost connection!\n");
	}

	char command2[100] = "SetCurPos,2000,2000,2000,2000,1300,2000";
	strcpy_s(sendBuf, command2);
	send(clientSocketHand_, sendBuf, strlen(sendBuf) + 1, 0);
	Sleep(300);
	sig = recv(clientSocketHand_, recvBuf, 100, 0);
	if (sig <= 0)
	{
		printf("Error: Hand lost connection!\n");
	}

	char command3[100] = "MoveJoint,0.18,-0.37,0.19,-60,-35,-160";
	strcpy_s(sendBuf, command3);
	send(clientSocketArm_, sendBuf, strlen(sendBuf) + 1, 0);
	Sleep(10);
	sig = recv(clientSocketArm_, recvBuf, 100, 0);
	if (sig <= 0)
	{
		printf("Error: Arm lost connection!\n");
	}
}