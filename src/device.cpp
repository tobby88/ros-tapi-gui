#include "device.hpp"

using namespace ros;
using namespace std;

// Constructor/Destructor

Device::Device(uint8_t type, string name, string uuid, unsigned long lastSeq, Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
  active = true;
}

Device::~Device()
{
}

// Public member functions

bool Device::Active()
{
  return active;
}

void Device::AddFeature(Feature feature)
{
  if (features.count(feature.GetUUID()) == 0)
    features.emplace(feature.GetUUID(), feature);
}

void Device::Deactivate()
{
  active = false;
}

Feature* Device::GetFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}

unsigned long Device::GetHeartbeat()
{
  return heartbeat;
}

Time Device::GetLastSeen()
{
  return lastSeen;
}

unsigned long Device::GetLastSeq()
{
  return lastSeq;
}

string Device::GetName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

vector<Feature*> Device::GetSortedFeatures()
{
  vector<Feature*> featureList;
  for (map<string, Feature>::iterator it = features.begin(); it != features.end(); it++)
    featureList.push_back(&it->second);
  if (featureList.size() > 1)
    sort(featureList.begin(), featureList.end(), compareFeatureNames);
  return featureList;
}

uint8_t Device::GetType()
{
  return type;
}

string Device::GetUUID()
{
  return uuid;
}

void Device::Update(uint8_t type, string name, unsigned long lastSeq, Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
  active = true;
}

// Private member functions

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  return first->GetName() < second->GetName();
}
