#include "device.hpp"

using namespace ros;
using namespace std;

Device::Device(DeviceType type, string name, string uuid, unsigned long lastSeq, Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}

Device::~Device()
{ }

void Device::addFeature(Feature feature)
{
  if (features.count(feature.getUUID()) == 0)
    features.emplace(feature.getUUID(), feature);
}

DeviceType Device::getType()
{
  return type;
}
unordered_map<string, Feature> Device::getFeatureMap()
{
  return features;
}

string Device::getName()
{
  return name;
}

string Device::getUUID()
{
  return uuid;
}

unsigned long Device::getLastSeq()
{
  return lastSeq;
}

Time Device::getLastSeen()
{
  return lastSeen;
}

unsigned long Device::getHeartbeat()
{
  return heartbeat;
}

void Device::Update(DeviceType type, string name, unsigned long lastSeq, Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}
