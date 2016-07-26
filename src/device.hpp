#ifndef DEVICE_H
#define DEVICE_H

#include "enums.hpp"
#include "feature.hpp"
#include "ros/ros.h"
#include <string>

using namespace ros;
using namespace std;

class Device
{
private:
  Device_Type type;
  string name;
  string uuid; // Not necessary as this is saved in the hash-table (unordered_map)
  unsigned long last_seq;
  Time last_seen_timestamp;
  unsigned long heartbeat_interval;
  map<unsigned long, Feature> features;

public:
  Device(Device_Type type, string name, string uuid, unsigned long last_seq, Time last_seen_timestamp, unsigned long heartbeat_interval);
  ~Device();
  void addFeature(Feature feature);
  map<unsigned long, Feature> getFeatureMap();
  Device_Type getType();
  string getName();
  string getUUID();
  unsigned long getLastSeq();
  Time getLastSeenTimestamp();
  unsigned long getHeartbeatInterval();
  void Update(Device_Type type, string name, unsigned long last_seq, Time last_seen_timestamp, unsigned long heartbeat_interval);
};

#endif // API_H
