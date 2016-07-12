#ifndef API_H
#define API_H

#include "tobby/hello.h"
#include "device.hpp"
#include "ros/ros.h"
#include <unordered_map>

using namespace ros;
using namespace std;

class Api
{
private:
  NodeHandle *nh;
  unordered_map<string, Device> devices;
  bool hello(tobby::hello::Request &helloReq, tobby::hello::Response &helloResp);

public:
  Api(NodeHandle *nodehandle);
  ~Api();
  void Run();
};

#endif // API_H
