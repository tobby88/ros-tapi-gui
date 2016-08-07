#include "api.hpp"
#include "enums.hpp"
#include "feature.hpp"
#include "tobby/Config.h"
#include "tobby/Feature.h"

#define STANDARD_HEARTBEAT_INTERVAL 10000L
#define DEBUG

using namespace ros;
using namespace std;

Api::Api(NodeHandle* nh)
{
  this->nh = nh;
  spinner = new AsyncSpinner(1);
  helloServ = nh->advertiseService("TobbyAPI/HelloServ", &Api::hello, this);
  configPub = nh->advertise<tobby::Config>("TobbyAPI/Config", 1000);
  ROS_INFO("Started Hello-Service, ready for API-connections.");
  changes = false;
}

Api::~Api()
{
  spinner->stop();
  delete spinner;
  ROS_INFO("Hello-Service has been stopped.");
}

bool Api::hello(tobby::Hello::Request& helloReq,
                tobby::Hello::Response& helloResp)
{
  string uuid = helloReq.uuid;
  if (devices.empty() || devices.count(helloReq.uuid) == 0)
  {
    // New device:
    unsigned long lastSeq = helloReq.header.seq;
    Time lastSeen = helloReq.header.stamp;
    string uuid = helloReq.uuid;
    string name = helloReq.name;
    DeviceType type = (DeviceType)helloReq.type;
    unsigned long heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    Device device(type, name, uuid, lastSeq, lastSeen, heartbeat);
    devices.emplace(helloReq.uuid, device);
    for (unsigned int i = 0; i < helloReq.features.capacity(); i++)
    {
      Feature feature(
          (FeatureType)helloReq.features[i].type, helloReq.features[i].name,
          helloReq.features[i].description, helloReq.features[i].uuid);
      devices.at(helloReq.uuid).addFeature(feature);
    }
    helloResp.status = (unsigned short)DeviceStatusResponse::OK;
    helloResp.heartbeat = heartbeat;
    changes = true;
  }
  else if (devices.count(helloReq.uuid) == 1)
  {
    unsigned long lastSeq = helloReq.header.seq;
    Time lastSeen = helloReq.header.stamp;
    string name = helloReq.name;
    DeviceType type = (DeviceType)helloReq.type;
    unsigned long heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    devices.at(helloReq.uuid).Update(type, name, lastSeq, lastSeen, heartbeat);
    // TODO: Updating feature-list
    helloResp.status = (unsigned short)DeviceStatusResponse::OK;
    helloResp.heartbeat = heartbeat;
    changes = true;
  }
  else
  {
    ROS_ERROR("Hello message couldn't be decoded, looks like there is "
              "something wrong with the devices database. Please try to "
              "restart the Hello-Service.");
    helloResp.status = (unsigned short)DeviceStatusResponse::Error;
    helloResp.heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    return false;
  }
#ifdef DEBUG
  DebugOutput();
#endif
  return true;
}

void Api::DebugOutput()
{
  if (changes)
  {
    for (unordered_map<string, Device>::iterator it = devices.begin();
         it != devices.end(); it++)
    {
      ROS_INFO("Debug: Device-Element UUID: %s", it->first.c_str());
      ROS_INFO("Debug: Device-Data: Type: %u, Name: %s, UUID: %s, Last Seq: "
               "%lu, Last Seen: %f, Heartbeat-Interval: %lu",
               (unsigned short)it->second.getType(),
               it->second.getName().c_str(), it->second.getUUID().c_str(),
               it->second.getLastSeq(), it->second.getLastSeen().toSec(),
               it->second.getHeartbeat());
      unordered_map<string, Feature> features = it->second.getFeatureMap();
      for (unordered_map<string, Feature>::iterator it2 = features.begin();
           it2 != features.end(); it2++)
      {
        ROS_INFO("Debug: Device-Feature: Map-ID: %s, ID: %s, Feature-Type: %u, "
                 "Feature-Name: %s, Feature-Description: %s",
                 it2->first.c_str(), it2->second.getUUID().c_str(),
                 (unsigned short)it2->second.getType(),
                 it2->second.getName().c_str(),
                 it2->second.getDescription().c_str());
      }
    }
    changes = false;
  }
}

void Api::Run() { spinner->start(); }
