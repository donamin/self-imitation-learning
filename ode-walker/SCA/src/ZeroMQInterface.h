#pragma once
#include "zmq.hpp"

#ifndef EIGEN_INCLUDED
	#define EIGEN_INCLUDED
	#include <Eigen/Geometry>
	//#include <Eigen/StdVector>
	using namespace Eigen;
	//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif // !EIGEN_INCLUDED

#ifndef STD_INCLUDED
	#define STD_INCLUDED
	#include <vector>
	#include <map>
	#include <queue>
	#include <iostream>
	#include <fstream>
	#include <cstdlib>
	#include <cstdio>
	#include <cmath>
	#include <string>
	#include <regex>
	#include <memory>
	#include <chrono>
	#include <future>
	using namespace std;
	using namespace std::chrono;
#endif // !STD_INCLUDED

void AppendInt2String(std::string &str, int x);
void AppendFloat2String(std::string &str, float x);
VectorXf ParseStringIntoVectorXf(std::string msg);
int ParseStringIntoFloatArray(std::string msg, float *temp);
std::string ParseIntoString(char id);
std::string ParseIntoString(char id, const char *data);
std::string ParseIntoString(char id, const float *data, int count);
std::string ParseIntoString(char id, const int *data, int count);
void SendBySocket(zmq::socket_t *socket, const std::string &message);
std::string RecieveFromSocket(zmq::socket_t *socket);

class ZMQ_Messanger
{
public:
	ZMQ_Messanger();
	~ZMQ_Messanger();
	std::string Send(const std::string &message);
private:
	zmq::context_t *context;
	zmq::socket_t *socket;
};

class ZMQ_MultiplayerInterface
{
public:
	ZMQ_MultiplayerInterface(bool server);
	ZMQ_MultiplayerInterface();
	~ZMQ_MultiplayerInterface();
	string SendAndRecieveMessage(string sendMsg);
	void Terminate(bool sendMessage);
	inline bool IsServer() { return isServer; }
private:
	zmq::context_t *context;
	zmq::socket_t *socket;
	bool isServer;
};