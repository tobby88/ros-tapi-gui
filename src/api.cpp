#include "api.hpp"
#include "feature.hpp"
#include "std_msgs/String.h"
#include "tapi_msgs/Connect.h"
#include "tapi_msgs/Connection.h"
#include "tapi_msgs/Device.h"
#include "tapi_msgs/Feature.h"
#include "tapi_msgs/GetConnectionList.h"
#include "tapi_msgs/GetDeviceList.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Api::Api(ros::NodeHandle* nh) : nh(nh)
{
  spinner = new ros::AsyncSpinner(1);
  pendingChanges = false;
  heartbeatCheckTimer = nh->createTimer(ros::Duration(HEARTBEAT_CHECK_INTERVAL / 1000.0), &Api::heartbeatCheck, this);
  heartbeatCheckTimer.start();
  devListClient = nh->serviceClient<tapi_msgs::GetDeviceList>("Tapi/GetDeviceList");
  lastUpdatedSub = nh->subscribe("Tapi/LastChanged", 5, &Api::updateData, this);
  delPub = nh->advertise<std_msgs::String>("Tapi/DeleteConnection", 1000);
  conPub = nh->advertise<tapi_msgs::Connect>("Tapi/ConnectFeatures", 1000);
  conListClient = nh->serviceClient<tapi_msgs::GetConnectionList>("Tapi/GetConnectionList");
  clearPub = nh->advertise<std_msgs::Bool>("Tapi/Clear", 2);
}

Api::~Api()
{
  heartbeatCheckTimer.stop();
  spinner->stop();
  delete spinner;
  devListClient.shutdown();
  lastUpdatedSub.shutdown();
  delPub.shutdown();
  conPub.shutdown();
  clearPub.shutdown();
}

// Public member functions

void Api::AddDeviceWithoutHello(uint8_t type, string name, string uuid, unsigned long heartbeat,
                                map<string, Tapi::Feature> features)
{
  Tapi::Device device(type, name, uuid, 0, ros::Time(0.0), heartbeat, features);
  device.Deactivate();
  devices.emplace(uuid, device);
  changed();
}

bool Api::CheckPending()
{
  return pendingChanges;
}

void Api::Clear()
{
  std_msgs::Bool msg;
  msg.data = true;
  clearPub.publish(msg);
}

bool Api::ConnectFeatures(string feature1uuid, string feature2uuid, double coefficient)
{
  tapi_msgs::Connect msg;
  msg.Coefficient = coefficient;
  msg.Feature1UUID = feature1uuid;
  msg.Feature2UUID = feature2uuid;
  conPub.publish(msg);
  return true;
}

void Api::DebugOutput()
{
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    ROS_INFO("Debug: Device-Element UUID: %s", it->first.c_str());
    ROS_INFO("Debug: Device-Data: Type: %d, Name: %s, UUID: %s, Last Seq: %lu, Last Seen: %f, Heartbeat-Interval: %lu",
             (unsigned int)it->second.GetType(), it->second.GetName().c_str(), it->second.GetUUID().c_str(),
             it->second.GetLastSeq(), it->second.GetLastSeen().toSec(), it->second.GetHeartbeat());
    vector<Tapi::Feature*> features = it->second.GetSortedFeatures();
    for (auto it2 = features.begin(); it2 != features.end(); ++it2)
    {
      ROS_INFO("Debug: Device-Feature: ID: %s, Feature-Type: %s, Feature-Name: %s", (*it2)->GetUUID().c_str(),
               (*it2)->GetType().c_str(), (*it2)->GetName().c_str());
    }
  }
  // TODO: Print connections
}

bool Api::DeleteConnection(string receiverFeatureUUID)
{
  std_msgs::String msg;
  msg.data = receiverFeatureUUID;
  delPub.publish(msg);
  changed();
}

void Api::Done()
{
  pendingChanges = false;
}

vector<Tapi::Connection*> Api::GetConnections()
{
  vector<Tapi::Connection*> connectionList;
  for (auto it = connections.begin(); it != connections.end(); ++it)
    connectionList.push_back(&it->second);
  return connectionList;
}

vector<Tapi::Device*> Api::GetDevicesSorted()
{
  vector<Tapi::Device*> devicesList;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    devicesList.push_back(&it->second);
  if (devicesList.size() > 1)
    sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
  return devicesList;
}

void Api::Run()
{
  spinner->start();
}

// Private memeber functions

void Api::changed()
{
  pendingChanges = true;
  sendAllConnections();
#ifdef DEBUG
  DebugOutput();
#endif
}

bool Api::compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second)
{
  return first->GetName() < second->GetName();
}

Tapi::Device* Api::getDeviceByFeatureUUID(string uuid)
{
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    if (it->second.GetFeatureByUUID(uuid))
      return &(it->second);
  }
  return 0;
}

void Api::heartbeatCheck(const ros::TimerEvent& e)
{
  bool deactivatedDevices = false;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    if ((it->second.Active()) &&
        (ros::Time::now().toSec() - it->second.GetLastSeen().toSec() > 2.5 * STANDARD_HEARTBEAT_INTERVAL / 1000.0))
    {
      it->second.Deactivate();
      deactivatedDevices = true;
    }
  if (deactivatedDevices)
    changed();
}

void Api::sendAllConnections()
{
  /*for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    tapi_msgs::Connection msg;
    msg.SenderUUID = it->second.GetSenderUUID();
    msg.SenderFeatureUUID = it->second.GetSenderFeatureUUID();
    msg.ReceiverUUID = it->second.GetReceiverUUID();
    msg.ReceiverFeatureUUID = it->second.GetReceiverFeatureUUID();
    msg.Coefficient = it->second.GetCoefficient();
    configPub.publish(msg);
  }*/
}

void Api::updateData(const std_msgs::Time::ConstPtr& time)
{
  if (time->data.toNSec() > lastUpdated.toNSec())
  {
    lastUpdated = time->data;
    bool updates = false;

    tapi_msgs::GetDeviceList devSrv;
    devSrv.request.get = true;
    if (!devListClient.call(devSrv))
    {
      ROS_ERROR("Failed to establish connection to core");
    }
    vector<tapi_msgs::Device> devVect = devSrv.response.Devices;

    for (auto it = devVect.begin(); it != devVect.end(); ++it)
    {
      bool active = it->Active;
      uint8_t deviceType = it->DeviceType;
      unsigned long heartbeat = it->Heartbeat;
      ros::Time lastSeen = it->LastSeen;
      unsigned long lastSeq = it->LastSeq;
      string name = it->Name;
      string uuid = it->UUID;
      vector<tapi_msgs::Feature> featureVec = it->Features;
      map<string, Tapi::Feature> featureMap;
      for (auto it2 = featureVec.begin(); it2 != featureVec.end(); ++it2)
      {
        string featureType = it2->FeatureType;
        string featureName = it2->Name;
        string featureUUID = it2->UUID;
        Tapi::Feature feature(featureType, featureName, featureUUID);
        featureMap.emplace(featureUUID, feature);
      }
      if (devices.empty() || devices.count(uuid) == 0)
      {
        Tapi::Device device(deviceType, name, uuid, lastSeq, lastSeen, heartbeat, featureMap);
        devices.emplace(uuid, device);
        updates = true;
      }
      else if (devices.count(uuid) == 1)
      {
        devices.at(uuid).Update(deviceType, name, lastSeq, lastSeen, heartbeat, featureMap);
        updates = true;
      }
      else
        return;

      if (!active)
        devices.at(uuid).Deactivate();
    }

    tapi_msgs::GetConnectionList conSrv;
    conSrv.request.get = true;
    if (!conListClient.call(conSrv))
    {
      ROS_ERROR("Failed to establish connection to core");
      return;
    }
    vector<tapi_msgs::Connection> conVect = conSrv.response.Connections;
    for (auto it = conVect.begin(); it != conVect.end(); ++it)
    {
      if (connections.count(it->ReceiverFeatureUUID) == 0)
      {
        Tapi::Connection connection(it->SenderUUID, it->SenderFeatureUUID, it->ReceiverUUID, it->ReceiverFeatureUUID,
                                    it->Coefficient);
        connections.emplace(it->ReceiverFeatureUUID, connection);
        devices.at(it->SenderUUID).GetFeatureByUUID(it->SenderFeatureUUID)->IncrementConnections();
        devices.at(it->ReceiverUUID).GetFeatureByUUID(it->ReceiverFeatureUUID)->IncrementConnections();
        updates = true;
      }
      else if (connections.at(it->ReceiverFeatureUUID).GetSenderFeatureUUID() != it->SenderFeatureUUID)
      {
        string receiverFeatureUUID = it->ReceiverFeatureUUID;
        Tapi::Device* receiverDevice = getDeviceByFeatureUUID(receiverFeatureUUID);
        string senderFeatureUUID = connections.at(receiverFeatureUUID).GetSenderFeatureUUID();
        Tapi::Device* senderDevice = getDeviceByFeatureUUID(senderFeatureUUID);
        receiverDevice->GetFeatureByUUID(receiverFeatureUUID)->DecrementConnections();
        senderDevice->GetFeatureByUUID(senderFeatureUUID)->DecrementConnections();
        connections.erase(receiverFeatureUUID);
        Tapi::Connection connection(it->SenderUUID, it->SenderFeatureUUID, it->ReceiverUUID, it->ReceiverFeatureUUID,
                                    it->Coefficient);
        connections.emplace(receiverFeatureUUID, connection);
        devices.at(it->SenderUUID).GetFeatureByUUID(it->SenderFeatureUUID)->IncrementConnections();
        devices.at(it->ReceiverUUID).GetFeatureByUUID(it->ReceiverFeatureUUID)->IncrementConnections();
        updates = true;
      }
    }
    vector<string> deletableConnections;
    for (auto it = connections.begin(); it != connections.end(); ++it)
    {
      bool found = false;
      string searchUUID = it->second.GetReceiverFeatureUUID();
      for (auto it2 = conVect.begin(); it2 != conVect.end(); ++it2)
        if (searchUUID == it2->ReceiverFeatureUUID)
          found = true;
      if (!found)
      {
        Tapi::Device* receiverDevice = getDeviceByFeatureUUID(searchUUID);
        string senderFeatureUUID = connections.at(searchUUID).GetSenderFeatureUUID();
        Tapi::Device* senderDevice = getDeviceByFeatureUUID(senderFeatureUUID);
        receiverDevice->GetFeatureByUUID(searchUUID)->DecrementConnections();
        senderDevice->GetFeatureByUUID(senderFeatureUUID)->DecrementConnections();
        deletableConnections.push_back(searchUUID);
        updates = true;
      }
    }
    for (auto it = deletableConnections.begin(); it != deletableConnections.end(); ++it)
      connections.erase(*it);
    if (updates)
      changed();
  }
}
}
