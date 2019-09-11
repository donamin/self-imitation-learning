#include "ZeroMQInterface.h"

void AppendInt2String(std::string &str, int x)
{
	char temp[20];
	sprintf(temp, "%d|\0", x);
	str += temp;
}

void AppendFloat2String(std::string &str, float x)
{
	char temp[30];
	sprintf(temp, "%.5f|\0", x);
	str += temp;
}

VectorXf ParseStringIntoVectorXf(std::string msg)
{
	float temp[5000];
	int index = ParseStringIntoFloatArray(msg, temp);
	return Map<VectorXf>(temp, index);
}

int ParseStringIntoFloatArray(std::string msg, float *temp)
{
	int index = 0;
	std::regex reg("[\\n|]");
	std::sregex_token_iterator it(msg.begin(), msg.end(), reg, -1);
	sregex_token_iterator reg_end;
	while (it != reg_end)
	{
		temp[index++] = std::stof(it->str());
		it++;
	}
	return index;
}

std::string ParseIntoString(char id)
{
	std::string message;
	message += id;
	message += '|';
	message += char(0);
	return message;
}

std::string ParseIntoString(char id, const char *data)
{
	std::string message;
	message += id;
	message += '|';
	message += data;
	message += char(0);
	return message;
}

std::string ParseIntoString(char id, const float *data, int count)
{
	std::string message;
	message += id;
	message += '|';
	char temp[50];
	for (int i = 0; i < count; ++i)
	{
		sprintf(temp, "%.5f|\0", data[i]);
		message += temp;
	}
	message += char(0);
	return message;
}

std::string ParseIntoString(char id, const int *data, int count)
{
	std::string message;
	message += id;
	message += '|';
	char temp[50];
	for (int i = 0; i < count; ++i)
	{
		sprintf(temp, "%d|\0", data[i]);
		message += temp;
	}
	message += char(0);
	return message;
}

void SendBySocket(zmq::socket_t *socket, const std::string &message)
{
	zmq::message_t request(message.data(), message.size());
	socket->send(request);
}

std::string RecieveFromSocket(zmq::socket_t *socket)
{
	zmq::message_t reply;
	socket->recv(&reply);
	return std::string((const char *)reply.data());
}

ZMQ_Messanger::ZMQ_Messanger()
{
	context = new zmq::context_t(1);
	socket = new zmq::socket_t(*context, ZMQ_REQ);
	std::cout << "Connecting to server..." << std::endl;
	socket->connect("tcp://localhost:5555");
}

ZMQ_Messanger::~ZMQ_Messanger()
{
	if (context)
	{
		delete socket;
		delete context;
	}
}

std::string ZMQ_Messanger::Send(const std::string &message)
{
	SendBySocket(socket, message);
	return RecieveFromSocket(socket);
}

ZMQ_MultiplayerInterface::ZMQ_MultiplayerInterface(bool server)
{
	this->isServer = server;
	context = new zmq::context_t(1);
	if (isServer)
	{
		socket = new zmq::socket_t(*context, ZMQ_REP);
		socket->bind("tcp://*:5555");
		std::cout << "Server setup successful." << std::endl;
	}
	else
	{
		socket = new zmq::socket_t(*context, ZMQ_REQ);
		socket->connect("tcp://localhost:5555");
		std::cout << "Client setup successful." << std::endl;
	}
}

ZMQ_MultiplayerInterface::ZMQ_MultiplayerInterface()
{
	ifstream fin("MartialArts_NetworkSettings.txt");
	char temp[100];
	fin >> temp;
	this->isServer = strcmp(temp, "Server") == 0;
	context = new zmq::context_t(1);
	if (isServer)
	{
		socket = new zmq::socket_t(*context, ZMQ_REP);
		socket->bind("tcp://*:5555");
		std::cout << "Server setup successful." << std::endl;
	}
	else
	{
		socket = new zmq::socket_t(*context, ZMQ_REQ);
		fin >> temp;
		fin.close();
		socket->connect(temp);
		std::cout << "Client setup successful." << std::endl;
	}
}

ZMQ_MultiplayerInterface::~ZMQ_MultiplayerInterface()
{
	delete socket;
	delete context;
}

string ZMQ_MultiplayerInterface::SendAndRecieveMessage(string sendMsg)
{
	string msg = "";
	if (isServer)
	{
		msg = RecieveFromSocket(socket);
		if (msg[0] == 'F')
			return msg;
		SendBySocket(socket, sendMsg);
	}
	else
	{
		SendBySocket(socket, sendMsg);
		msg = RecieveFromSocket(socket);
		if (msg[0] == 'F')
			return msg;
	}
	return msg;
}

void ZMQ_MultiplayerInterface::Terminate(bool sendMessage)
{
	if (sendMessage)
	{
		if (isServer)
		{
			RecieveFromSocket(socket);
		}
		SendBySocket(socket, ParseIntoString('F'));
	}
}