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
  Device(Device_Type type, string name, string uuid, Time last_seen_timestamp, unsigned long heartbeat_interval);
  void addFeature(Feature feature);
  ~Device();
};

#endif // API_H
