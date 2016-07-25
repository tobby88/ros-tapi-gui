#include "api.hpp"
#include "enums.hpp"
#include "feature.hpp"
#include "tobby/feature.h"

#define STANDARD_HEARTBEAT_INTERVAL 10000L
#define DEBUG

using namespace ros;
using namespace std;

Api::Api(NodeHandle *nh)
{
  this->nh = nh;
  helloServ = nh->advertiseService("TobbyAPI/Hello", &Api::hello, this);
  ROS_INFO("Started Hello-Service, ready for API-connections.");
}

Api::~Api()
{
  ROS_INFO("Hello-Service has been stopped.");
}

bool Api::hello(tobby::hello::Request &helloReq, tobby::hello::Response &helloResp)
{
  string uuid = helloReq.uuid;
  if(devices.empty() || devices.count(helloReq.uuid) == 0)
  {
    // New device:
    unsigned long last_seq = helloReq.header.seq;
    Time last_seen_timestamp = helloReq.header.stamp;
    string uuid = helloReq.uuid;
    string name = helloReq.product_name;
    Device_Type type = (Device_Type) helloReq.type;
    unsigned long heartbeat_interval = STANDARD_HEARTBEAT_INTERVAL;
    Device device(type, name, uuid, last_seq, last_seen_timestamp, heartbeat_interval);
    devices.emplace(helloReq.uuid, device);
    for(int i=0; i<helloReq.features.capacity(); i++)
    {
      Feature feature((Feature_Type)helloReq.features[i].type, helloReq.features[i].name, helloReq.features[i].id);
      devices.at(helloReq.uuid).addFeature(feature);
    }
    helloResp.status = (unsigned short) Device_Status_Response::OK;
    helloResp.heartbeat_interval = heartbeat_interval;
  }
  return true;
}

void Api::DebugOutput()
{
  for(unordered_map<string, Device>::iterator it = devices.begin(); it != devices.end(); it++)
  {
    ROS_INFO("Debug: Device-Element name: %s", it->first.c_str());
    ROS_INFO("Debug: Device-Data: Type: %u, Name: %s, UUID: %s, Last Seq: %lu, Last Seen: %f, Heartbeat-Interval: %lu", (unsigned short) it->second.getType(), it->second.getName().c_str(), it->second.getUUID().c_str(), it->second.getLastSeq(), it->second.getLastSeenTimestamp().toSec(), it->second.getHeartbeatInterval());
  }
}

void Api::Run()
{
#ifndef DEBUG
  spin();
#endif
#ifdef DEBUG
  while(ok())
  {
    spinOnce();
    DebugOutput();
  }
#endif
}
