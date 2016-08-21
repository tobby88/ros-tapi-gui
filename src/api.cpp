#include "api.hpp"
#include "enums.hpp"
#include "feature.hpp"
#include "tobby/Config.h"
#include "tobby/Feature.h"

#define STANDARD_HEARTBEAT_INTERVAL 10000L
#define DEBUG

using namespace ros;
using namespace std;

// Public member functions

Api::Api(NodeHandle* nh)
{
  this->nh = nh;
  spinner = new AsyncSpinner(1);
  helloServ = nh->advertiseService("TobbyAPI/HelloServ", &Api::hello, this);
  configPub = nh->advertise<tobby::Config>("TobbyAPI/Config", 1000);
  ROS_INFO("Started Hello-Service, ready for API-connections.");
  pendingChanges = false;
}

Api::~Api()
{
  helloServ.shutdown();
  configPub.shutdown();
  spinner->stop();
  delete spinner;
  ROS_INFO("Hello-Service has been stopped.");
}

bool Api::CheckPending() { return pendingChanges; }

bool Api::ConnectFeatures(string feature1uuid, string feature2uuid,
                          double coefficient)
{
  Device *device1, *device2;
  device1 = getDeviceByFeatureUUID(feature1uuid);
  device2 = getDeviceByFeatureUUID(feature2uuid);
  if (device1 == 0 || device2 == 0)
    // At least one Device not found
    return false;
  if (device1->getType() == device2->getType())
    // Cannont connect devices of same type (sender-sender or receiver-receiver)
    return false;
  if (device1->getFeatureMap().at(feature1uuid).getType() !=
      device2->getFeatureMap().at(feature2uuid).getType())
    // Cannot connect features of different types
    return false;

  // Who is sender, who receiver?
  string senderUUID, receiverUUID, senderFeatureUUID, receiverFeatureUUID;
  if (device1->getType() == DeviceType::ReceiverDevice)
  {
    receiverUUID = device1->getUUID();
    receiverFeatureUUID = feature1uuid;
    senderUUID = device2->getUUID();
    senderFeatureUUID = feature2uuid;
  }
  else
  {
    receiverUUID = device2->getUUID();
    receiverFeatureUUID = feature2uuid;
    senderUUID = device1->getUUID();
    senderFeatureUUID = feature1uuid;
  }

  if (connections.count(receiverFeatureUUID) > 0)
    // Old connection was not removed before reassigning!
    return false;
  else
  {
    // Connect devices/features
    Assignment connection(senderUUID, senderFeatureUUID, receiverUUID,
                          receiverFeatureUUID, coefficient);
    connections.emplace(receiverFeatureUUID, connection);
    device1->getFeatureByUUID(feature1uuid)->incrementConnections();
    device2->getFeatureByUUID(feature2uuid)->incrementConnections();
    changed();
    return true;
  }
  // "Error" handler - should never be reached:
  return false;
}

void Api::DebugOutput()
{
  for (map<string, Device>::iterator it = devices.begin(); it != devices.end();
       it++)
  {
    ROS_INFO("Debug: Device-Element UUID: %s", it->first.c_str());
    ROS_INFO("Debug: Device-Data: Type: %u, Name: %s, UUID: %s, Last Seq: %lu, "
             "Last Seen: %f, Heartbeat-Interval: %lu",
             (unsigned short)it->second.getType(), it->second.getName().c_str(),
             it->second.getUUID().c_str(), it->second.getLastSeq(),
             it->second.getLastSeen().toSec(), it->second.getHeartbeat());
    map<string, Feature> features = it->second.getFeatureMap();
    for (map<string, Feature>::iterator it2 = features.begin();
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
  // TODO: Print connections
}

bool Api::DeleteConnection(string receiverFeatureUUID)
{
  if (connections.count(receiverFeatureUUID) > 0)
  {
    Assignment* connection = &connections.at(receiverFeatureUUID);
    string senderUUID = connection->GetSenderUUID();
    string senderFeatureUUID = connection->GetSenderFeatureUUID();
    string receiverUUID = connection->GetReceiverUUID();
    if (devices.count(senderUUID) > 0)
      devices.at(senderUUID)
          .getFeatureByUUID(senderFeatureUUID)
          ->decrementConnections();
    if (devices.count(receiverUUID) > 0)
      devices.at(receiverUUID)
          .getFeatureByUUID(receiverFeatureUUID)
          ->decrementConnections();
    tobby::Config msg;
    msg.SenderUUID = "0";
    msg.SenderFeatureUUID = "0";
    msg.ReceiverUUID = receiverUUID;
    msg.ReceiverFeatureUUID = receiverFeatureUUID;
    msg.Coefficient = 0;
    configPub.publish(msg);
    connections.erase(receiverFeatureUUID);
  }
}

void Api::Done() { pendingChanges = false; }

vector<Assignment*> Api::GetConnections()
{
  vector<Assignment*> connectionList;
  for (map<string, Assignment>::iterator it = connections.begin();
       it != connections.end(); it++)
    connectionList.push_back(&it->second);
  return connectionList;
}

vector<Device*> Api::GetDevicesSorted()
{
  vector<Device*> devicesList;
  for (map<string, Device>::iterator it = devices.begin(); it != devices.end();
       it++)
    devicesList.push_back(&it->second);
  if (devicesList.size() > 1)
    sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
  return devicesList;
}

void Api::Run() { spinner->start(); }

// Private memeber functions

void Api::changed()
{
  pendingChanges = true;
  sendAllConnections();
#ifdef DEBUG
  DebugOutput();
#endif
}

bool Api::compareDeviceNames(const Device* first, const Device* second)
{
  return first->getName() < second->getName();
}

Device* Api::getDeviceByFeatureUUID(string uuid)
{
  for (map<string, Device>::iterator it = devices.begin(); it != devices.end();
       it++)
  {
    if (it->second.getFeatureMap().count(uuid) > 0)
      return &(it->second);
  }
  return 0;
}

bool Api::hello(tobby::Hello::Request& helloReq,
                tobby::Hello::Response& helloResp)
{
  string uuid = helloReq.UUID;
  if (devices.empty() || devices.count(uuid) == 0)
  {
    // New device:
    unsigned long lastSeq = helloReq.Header.seq;
    Time lastSeen = helloReq.Header.stamp;
    string name = helloReq.Name;
    DeviceType type = (DeviceType)helloReq.DeviceType;
    unsigned long heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    Device device(type, name, uuid, lastSeq, lastSeen, heartbeat);
    devices.emplace(uuid, device);
    for (unsigned int i = 0; i < helloReq.Features.capacity(); i++)
    {
      Feature feature((FeatureType)helloReq.Features[i].FeatureType,
                      helloReq.Features[i].Name,
                      helloReq.Features[i].Description,
                      helloReq.Features[i].UUID);
      devices.at(uuid).addFeature(feature);
    }
    helloResp.Status = (unsigned short)DeviceStatusResponse::OK;
    helloResp.Heartbeat = heartbeat;
    changed();
  }
  else if (devices.count(uuid) == 1)
  {
    unsigned long lastSeq = helloReq.Header.seq;
    Time lastSeen = helloReq.Header.stamp;
    string name = helloReq.Name;
    DeviceType type = (DeviceType)helloReq.DeviceType;
    unsigned long heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    devices.at(uuid).Update(type, name, lastSeq, lastSeen, heartbeat);
    // TODO: Updating feature-list
    helloResp.Status = (unsigned short)DeviceStatusResponse::OK;
    helloResp.Heartbeat = heartbeat;
    changed();
  }
  else
  {
    ROS_ERROR("Hello message couldn't be decoded, looks like there is "
              "something wrong with the devices database. Please try to "
              "restart the Hello-Service.");
    helloResp.Status = (unsigned short)DeviceStatusResponse::Error;
    helloResp.Heartbeat = STANDARD_HEARTBEAT_INTERVAL;
    return false;
  }

  return true;
}

void Api::sendAllConnections()
{
  for (map<string, Assignment>::iterator it = connections.begin();
       it != connections.end(); it++)
  {
    tobby::Config msg;
    msg.SenderUUID = it->second.GetSenderUUID();
    msg.SenderFeatureUUID = it->second.GetSenderFeatureUUID();
    msg.ReceiverUUID = it->second.GetReceiverUUID();
    msg.ReceiverFeatureUUID = it->second.GetReceiverFeatureUUID();
    msg.Coefficient = it->second.GetCoefficient();
    configPub.publish(msg);
  }
}
