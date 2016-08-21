#include "device.hpp"

using namespace ros;
using namespace std;

Device::Device(DeviceType type, string name, string uuid, unsigned long lastSeq,
               Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}

Device::~Device() {}

void Device::addFeature(Feature feature)
{
  if (features.count(feature.getUUID()) == 0)
    features.emplace(feature.getUUID(), feature);
}

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  return first->getName() < second->getName();
}

DeviceType Device::getType() { return type; }

map<string, Feature> Device::getFeatureMap() { return features; }

vector<Feature*> Device::GetSortedFeatures()
{
  vector<Feature*> featureList;
  for (map<string, Feature>::iterator it = features.begin();
       it != features.end(); it++)
    featureList.push_back(&it->second);
  if (featureList.size() > 1)
    sort(featureList.begin(), featureList.end(), compareFeatureNames);
  return featureList;
}

string Device::getName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

string Device::getUUID() { return uuid; }

unsigned long Device::getLastSeq() { return lastSeq; }

Time Device::getLastSeen() { return lastSeen; }

unsigned long Device::getHeartbeat() { return heartbeat; }

void Device::Update(DeviceType type, string name, unsigned long lastSeq,
                    Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}

Feature* Device::getFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}
