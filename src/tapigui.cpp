/******************************************************************************
*  This file is part of tapi_gui.                                             *
*                                                                             *
*  tapi_gui is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_gui is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_gui.  If not, see <http://www.gnu.org/licenses/>.          *
*                                                                             *
*  Diese Datei ist Teil von tapi_gui.                                         *
*                                                                             *
*  tapi_gui ist Freie Software: Sie können es unter den Bedingungen           *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_gui wird in der Hoffnung, dass es nützlich sein wird, aber            *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

#include "tapigui.hpp"
#include <QCursor>
#include <QFileDialog>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QString>
#include <fstream>
#include <string>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tapi_lib/Connect.h"
#include "tapi_lib/Feature.h"
#include "tapi_lib/GetConnectionList.h"
#include "tapi_lib/GetDeviceList.h"
#include "tapi_lib/Hello.h"
#include "ui_tapigui.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TapiGui::TapiGui(ros::NodeHandle* nh, QWidget* parent) : QWidget(parent), ui(new Ui::TapiGui), nh(nh), parent(parent)
{
  spinner = new ros::AsyncSpinner(1);
  pendingChanges = false;
  devListClient = nh->serviceClient<tapi_lib::GetDeviceList>("/Tapi/GetDeviceList");
  delPub = nh->advertise<std_msgs::String>("/Tapi/DeleteConnection", 10000);
  conPub = nh->advertise<tapi_lib::Connect>("/Tapi/ConnectFeatures", 10000);
  conListClient = nh->serviceClient<tapi_lib::GetConnectionList>("/Tapi/GetConnectionList");
  clearAllPub = nh->advertise<std_msgs::Bool>("/Tapi/ClearAll", 2);
  clearInactivePub = nh->advertise<std_msgs::Bool>("/Tapi/ClearInactive", 2);
  helloClient = nh->serviceClient<tapi_lib::Hello>("/Tapi/HelloServ");
  updateTimer.start();

  ui->setupUi(this);
  guitimer = new QTimer(this);
  connect(guitimer, SIGNAL(timeout()), this, SLOT(checkForGuiUpdate()));
  timerInterval = 15;
  guitimer->start(timerInterval);

  // Add vertical layouts to the scroll Areas
  layoutSubscriber = ui->verticalLayoutSubscriber;
  layoutPublisher = ui->verticalLayoutPublisher;
  layoutSubscriber->setAlignment(Qt::AlignTop);
  layoutPublisher->setAlignment(Qt::AlignTop);
  ui->verticalLayoutKeys->setAlignment(Qt::AlignTop);

  selectedFeature = 0;
  selectedGuiDevice = 0;

  connect(ui->loadButton, SIGNAL(clicked(bool)), this, SLOT(loadButtonClicked()));
  connect(ui->saveButton, SIGNAL(clicked(bool)), this, SLOT(saveButtonClicked()));
  connect(ui->clearAllButton, SIGNAL(clicked(bool)), this, SLOT(clearAllButtonClicked()));
  connect(ui->clearInactiveButton, SIGNAL(clicked(bool)), this, SLOT(clearInactiveButtonClicked()));

  lastUpdatedSub = nh->subscribe("/Tapi/LastChanged", 5, &TapiGui::updateAvailable, this);
  updateData();
  spinner->start();
  updateTimer = nh->createTimer(ros::Duration(CHECK_INTERVAL / 1000.0), &TapiGui::timer, this);
  updateTimer.start();
}

TapiGui::~TapiGui()
{
  guitimer->stop();
  delete guitimer;
  updateTimer.stop();
  spinner->stop();
  delete spinner;
  conListClient.shutdown();
  devListClient.shutdown();
  lastUpdatedSub.shutdown();
  delPub.shutdown();
  conPub.shutdown();
  clearAllPub.shutdown();
  helloClient.shutdown();
  delete ui;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    delete it->second;
}

// Protected member functions

void TapiGui::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setPen(Qt::black);

  // Draw all connections
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    string publisherUUID = it->second.GetPublisherUUID();
    string publisherFeatureUUID = it->second.GetPublisherFeatureUUID();
    string subscriberUUID = it->second.GetSubscriberUUID();
    string subscriberFeatureUUID = it->second.GetSubscriberFeatureUUID();
    Tapi::GuiDevice *publisher, *subscriber;
    publisher = 0;
    subscriber = 0;
    if (publisherGuiDevices.count(publisherUUID) > 0)
      publisher = publisherGuiDevices.at(publisherUUID);
    if (!publisher)
      continue;
    if (subscriberGuiDevices.count(subscriberUUID) > 0)
      subscriber = subscriberGuiDevices.at(subscriberUUID);
    if (!subscriber)
      continue;
    QPoint begin, end;
    Tapi::Feature* feature = publisher->GetFeatureByUUID(publisherFeatureUUID);
    if (!feature)
      continue;
    begin = publisher->mapTo(this, publisher->FeatureBoxPosition(feature));
    feature = subscriber->GetFeatureByUUID(subscriberFeatureUUID);
    if (!feature)
      continue;
    end = subscriber->mapTo(this, subscriber->FeatureBoxPosition(feature));
    painter.drawLine(begin, end);
  }

  // Draw line for current (pending) connection
  if (!selectedFeature || !selectedGuiDevice)
    return;
  QPoint end = mapFromGlobal(mousePosition);
  QPoint begin = selectedGuiDevice->mapTo(this, selectedGuiDevice->FeatureBoxPosition(selectedFeature));
  painter.drawLine(begin, end);
}

// Private member functions

void TapiGui::addDeviceToTapi(uint8_t type, string name, string uuid, map<string, Tapi::Feature> features)
{
  tapi_lib::Hello hello;
  hello.request.DeviceType = type;
  vector<tapi_lib::Feature> featureVec;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    tapi_lib::Feature feature;
    feature.FeatureType = it->second.GetType();
    feature.Name = it->second.GetName();
    feature.UUID = it->second.GetUUID();
    featureVec.push_back(feature);
  }
  hello.request.Features = featureVec;
  std_msgs::Header header;
  header.seq = 1;
  ros::Time now = ros::Time::now();
  header.stamp = now;
  hello.request.Header = header;
  hello.request.Name = name;
  hello.request.UUID = uuid;
  if (!helloClient.call(hello))
    ROS_ERROR("Couldn't connect to hello service.");
  if (hello.response.Status == tapi_lib::HelloResponse::StatusError)
    ROS_ERROR("Error when connection to hello service");
}

bool TapiGui::compareDeviceNames(const Tapi::GuiDevice* first, const Tapi::GuiDevice* second)
{
  string temp1, temp2;
  temp1 = first->GetName();
  temp2 = second->GetName();
  transform(temp1.begin(), temp1.end(), temp1.begin(), ::towlower);
  transform(temp2.begin(), temp2.end(), temp2.begin(), ::towlower);
  bool result = temp1 < temp2;
  return result;
}

void TapiGui::connectFeatures(string feature1uuid, string feature2uuid, double coefficient)
{
  tapi_lib::Connect msg;
  msg.Coefficient = coefficient;
  msg.Feature1UUID = feature1uuid;
  msg.Feature2UUID = feature2uuid;
  conPub.publish(msg);
}

void TapiGui::deleteConnection(string subscriberFeatureUUID)
{
  std_msgs::String msg;
  msg.data = subscriberFeatureUUID;
  delPub.publish(msg);
}

vector<Tapi::GuiDevice*> TapiGui::getDevicesSorted()
{
  vector<Tapi::GuiDevice*> devicesList;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    devicesList.push_back(it->second);
  if (devicesList.size() > 1)
    sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
  return devicesList;
}

void TapiGui::timer(const ros::TimerEvent& e)
{
  if (lastUpdated.toNSec() + (CHECK_INTERVAL * 1000) < ros::Time::now().toNSec())
    pendingChanges = true;
}

void TapiGui::updateAvailable(const std_msgs::Time::ConstPtr& time)
{
  if (time->data.toNSec() > lastUpdated.toNSec())
    pendingChanges = true;
}

void TapiGui::updateData()
{
  pendingChanges = false;
  lastUpdated = ros::Time::now();
  tapi_lib::GetDeviceList devSrv;
  devSrv.request.Get = true;
  if (!devListClient.call(devSrv))
  {
    ROS_ERROR("Failed to establish connection to core");
    return;
  }
  vector<tapi_lib::Device> devVect = devSrv.response.Devices;
  for (auto it = devVect.begin(); it != devVect.end(); ++it)
  {
    bool active = it->Active;
    uint8_t deviceType = it->DeviceType;
    unsigned long heartbeat = it->Heartbeat;
    ros::Time lastSeen = it->LastSeen;
    unsigned long lastSeq = it->LastSeq;
    string name = it->Name;
    string uuid = it->UUID;
    vector<tapi_lib::Feature> featureVec = it->Features;
    map<string, Tapi::Feature> featureMap;
    for (auto it2 = featureVec.begin(); it2 != featureVec.end(); ++it2)
    {
      string featureType = it2->FeatureType;
      string featureName = it2->Name;
      string featureUUID = it2->UUID;
      if ((connections.count(featureUUID) > 0) &&
          (featureType == "std_msgs/Byte" || featureType == "std_msgs/Float32" || featureType == "std_msgs/Float64" ||
           featureType == "std_msgs/Int16" || featureType == "std_msgs/Int32" || featureType == "std_msgs/Int64" ||
           featureType == "std_msgs/Int8" || featureType == "std_msgs/UInt16" || featureType == "std_msgs/UInt32" ||
           featureType == "std_msgs/UInt64" || featureType == "std_msgs/UInt8"))
      {
        double coefficient = connections.at(featureUUID).GetCoefficient();
        string strCoeff = to_string(coefficient);
        strCoeff.erase(strCoeff.find_last_not_of('0') + 1, string::npos);
        featureName = featureName + " (x" + strCoeff + "0)";
      }
      Tapi::Feature feature(featureType, featureName, featureUUID);
      featureMap.emplace(featureUUID, feature);
    }
    if (devices.empty() || devices.count(uuid) == 0)
    {
      Tapi::GuiDevice* device =
          new Tapi::GuiDevice(parent, deviceType, name, uuid, lastSeq, lastSeen, heartbeat, featureMap);
      devices.emplace(uuid, device);
      pendingChanges = true;
    }
    else if (devices.count(uuid) == 1)
    {
      devices.at(uuid)->Update(deviceType, name, lastSeq, lastSeen, heartbeat, featureMap);
      pendingChanges = true;
    }
    else
      continue;

    if (!active)
      devices.at(uuid)->Deactivate();
  }

  vector<string> toDelete;
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    bool found = false;
    string devUUID = it->first;
    for (auto it2 = devVect.begin(); it2 != devVect.end(); ++it2)
      if (it2->UUID == devUUID)
      {
        found = true;
        break;
      }
    if (!found)
      toDelete.push_back(devUUID);
  }
  for (auto it = toDelete.begin(); it != toDelete.end(); ++it)
  {
    if (subscriberGuiDevices.count(*it) > 0)
    {
      disconnect(subscriberGuiDevices.at(*it), SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), 0, 0);
      subscriberGuiDevices.at(*it)->hide();
      subscriberGuiDevices.erase(*it);
    }
    else if (publisherGuiDevices.count(*it) > 0)
    {
      disconnect(publisherGuiDevices.at(*it), SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), 0, 0);
      publisherGuiDevices.at(*it)->hide();
      publisherGuiDevices.erase(*it);
    }
    delete devices.at(*it);
    devices.erase(*it);
  }

  tapi_lib::GetConnectionList conSrv;
  conSrv.request.Get = true;
  if (!conListClient.call(conSrv))
  {
    ROS_ERROR("Failed to establish connection to core");
    return;
  }
  vector<tapi_lib::Connection> conVect = conSrv.response.Connections;
  for (auto it = conVect.begin(); it != conVect.end(); ++it)
  {
    if (devices.count(it->SubscriberUUID) == 0 || devices.count(it->PublisherUUID) == 0)
      // Try to connect non existend devices -> try next connection
      continue;
    if (connections.count(it->SubscriberFeatureUUID) == 0)
    // No connection on this device yet -> store connection
    {
      Tapi::Connection connection(it->PublisherUUID, it->PublisherFeatureUUID, it->SubscriberUUID,
                                  it->SubscriberFeatureUUID, it->Coefficient);
      connections.emplace(it->SubscriberFeatureUUID, connection);
      pendingChanges = true;
    }
    else if (connections.at(it->SubscriberFeatureUUID).GetPublisherFeatureUUID() != it->PublisherFeatureUUID)
    // Subscriber already connected, but publisher has changed, delete old connection and store new one
    {
      string subscriberFeatureUUID = it->SubscriberFeatureUUID;
      connections.erase(subscriberFeatureUUID);
      Tapi::Connection connection(it->PublisherUUID, it->PublisherFeatureUUID, it->SubscriberUUID,
                                  it->SubscriberFeatureUUID, it->Coefficient);
      connections.emplace(subscriberFeatureUUID, connection);
      pendingChanges = true;
    }
  }
  vector<string> deletableConnections;
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    bool found = false;
    string searchUUID = it->first;
    for (auto it2 = conVect.begin(); it2 != conVect.end(); ++it2)
      if (searchUUID == it2->SubscriberFeatureUUID)
        found = true;
    if (!found)
    {
      deletableConnections.push_back(searchUUID);
      pendingChanges = true;
    }
  }
  for (auto it = deletableConnections.begin(); it != deletableConnections.end(); ++it)
    connections.erase(*it);
}

// Slot functions

void TapiGui::clearAllButtonClicked()
{
  selectedFeature = 0;
  selectedGuiDevice = 0;
  std_msgs::Bool msg;
  msg.data = true;
  clearAllPub.publish(msg);
  update();
}

void TapiGui::clearInactiveButtonClicked()
{
  std_msgs::Bool msg;
  msg.data = true;
  clearInactivePub.publish(msg);
}

void TapiGui::checkForGuiUpdate()
{
  if (pendingChanges)
    updateData();
  if (pendingChanges)
  {
    string selectedFeatureUUID = "";
    if (selectedFeature)
      selectedFeatureUUID = selectedFeature->GetUUID();
    for (auto it = publisherGuiDevices.begin(); it != publisherGuiDevices.end(); ++it)
    {
      disconnect(it->second, SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), 0, 0);
      it->second->hide();
      layoutPublisher->removeWidget(it->second);
    }
    publisherGuiDevices.clear();
    for (auto it = subscriberGuiDevices.begin(); it != subscriberGuiDevices.end(); ++it)
    {
      disconnect(it->second, SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), 0, 0);
      it->second->hide();
      layoutSubscriber->removeWidget(it->second);
    }
    subscriberGuiDevices.clear();
    vector<Tapi::GuiDevice*> devicesSorted = getDevicesSorted();
    for (auto it = devicesSorted.begin(); it != devicesSorted.end(); ++it)
    {
      if ((*it)->GetType() == tapi_lib::Device::Type_Publisher)
      {
        layoutPublisher->addWidget(*it);
        publisherGuiDevices.emplace((*it)->GetUUID(), *it);
      }
      else
      {
        layoutSubscriber->addWidget(*it);
        subscriberGuiDevices.emplace((*it)->GetUUID(), *it);
      }
      (*it)->show();  // don't forget to show it

      connect(*it, SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), this,
              SLOT(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)));

      vector<Tapi::Feature*> features = (*it)->GetSortedFeatures();
      for (auto it2 = features.begin(); it2 != features.end(); ++it2)
      {
        string type = (*it2)->GetType();
        if (colorKeys.count(type) == 0)
        {
          QColor color = GuiDevice::stringToColor(type);
          colorKeys.emplace(type, color);
          QLabel* key = new QLabel();
          QString format("<font color=\"%1\">%2 </font> %3");
          key->setText(format.arg(color.name(), "█ ", type.c_str()));
          ui->verticalLayoutKeys->addWidget(key);
          key->show();
        }
      }
    }
    pendingChanges = false;
  }
  if (selectedFeature)
  {
    QPoint newMousePosition = QCursor::pos();
    if (newMousePosition != mousePosition)
    {
      mousePosition = newMousePosition;
    }
    if (!selectedGuiDevice->Active())
    {
      selectedFeature = 0;
      selectedGuiDevice = 0;
    }
  }
  update();
}

void TapiGui::featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature)
{
  if ((selectedFeature && selectedFeature == feature) || !guidevice->Active())
  // Clicked twice on the same feature or device is inactive -> demarcate selection
  {
    selectedFeature = 0;
    selectedGuiDevice = 0;
    update();
    return;
  }

  if (guidevice->GetType() == tapi_lib::Device::Type_Subscriber && connections.count(feature->GetUUID()) > 0)
  {
    QMessageBox msgBox;
    if (selectedFeature && guidevice->GetType() != selectedGuiDevice->GetType() &&
        selectedFeature->GetType() == feature->GetType())
    {
      string msg = "Replace connection of \"" + feature->GetName() + "\"?";
      msgBox.setWindowTitle("Feature already connected");
      msgBox.setText(QString::fromStdString(msg));
    }
    else
    {
      string msg = "Delete connection of \"" + feature->GetName() + "\"?";
      msgBox.setWindowTitle("Delete connection?");
      msgBox.setText(QString::fromStdString(msg));
    }
    msgBox.setStandardButtons(QMessageBox::No | QMessageBox::Yes);
    msgBox.setDefaultButton(QMessageBox::No);
    msgBox.setIcon(QMessageBox::Question);
    guitimer->stop();
    int ret = msgBox.exec();
    guitimer->start(timerInterval);
    switch (ret)
    {
      case QMessageBox::No:
        return;
      case QMessageBox::Yes:
        deleteConnection(feature->GetUUID());
        update();
        break;
      default:
        break;
    }
    if (!selectedFeature)
      return;
  }

  if (!selectedFeature)
  // Nothing selected before, select first feature of (possible coming)
  // connection
  {
    selectedFeature = feature;
    selectedGuiDevice = guidevice;
    return;
  }

  if (guidevice->GetType() == selectedGuiDevice->GetType())
  // Selected the same device type, so no connection is possible. Drop old
  // selection and select the new one
  {
    selectedFeature = feature;
    selectedGuiDevice = guidevice;
    return;
  }

  if (selectedFeature->GetType() != feature->GetType())
  // Selected different feature types, so no connection is possible. Drop old
  // selection and select the new one
  {
    selectedFeature = feature;
    selectedGuiDevice = guidevice;
    return;
  }

  // Everything ok -> try to connect them (and ask for coefficient if appliable)
  double coefficient = 1.0;
  bool ok = true;
  string type = feature->GetType();
  if (type == "std_msgs/Byte" || type == "std_msgs/Float32" || type == "std_msgs/Float64" || type == "std_msgs/Int16" ||
      type == "std_msgs/Int32" || type == "std_msgs/Int64" || type == "std_msgs/Int8" || type == "std_msgs/UInt16" ||
      type == "std_msgs/UInt32" || type == "std_msgs/UInt64" || type == "std_msgs/UInt8")
  {
    guitimer->stop();
    QString input =
        QInputDialog::getText(this, "Coefficient?", "Multiply all values with:", QLineEdit::Normal, "1.0", &ok);
    coefficient = input.toDouble(&ok);
    guitimer->start(timerInterval);
  }
  if (ok)
  {
    connectFeatures(selectedFeature->GetUUID(), feature->GetUUID(), coefficient);
    selectedFeature = 0;
    selectedGuiDevice = 0;
    update();
  }
}

void TapiGui::loadButtonClicked()
{
  string homedir = getenv("HOME");
  string filename = homedir + "/config.tapi";
  QString filePicker =
      QFileDialog::getOpenFileName(this, "Open File", QString::fromStdString(filename), "Tapi-files (*.tapi)");
  filename = filePicker.toStdString();
  ifstream fileInput;
  fileInput.open(filename);
  bool error = true;
  if (fileInput.is_open() && !fileInput.eof())
  {
    error = false;
    string temp;
    clearAllButtonClicked();
    getline(fileInput, temp);
    if (fileInput.eof())
      error = true;
    while (!fileInput.eof())
    {
      if (temp == "[Device]")
      {
        string uuid;
        string name;
        uint8_t type;
        unsigned long heartbeat;
        getline(fileInput, uuid);
        getline(fileInput, name);
        getline(fileInput, temp);
        type = (uint8_t)stoi(temp);
        getline(fileInput, temp);
        heartbeat = (unsigned long)stoul(temp);
        getline(fileInput, temp);
        map<string, Tapi::Feature> features;
        while (temp == "[DeviceFeature]")
        {
          string featureUUID;
          string featureName;
          string featureType;
          getline(fileInput, featureUUID);
          getline(fileInput, featureName);
          getline(fileInput, featureType);
          Tapi::Feature feature = Tapi::Feature(featureType, featureName, featureUUID);
          features.emplace(featureUUID, feature);
          getline(fileInput, temp);
        }
        addDeviceToTapi(type, name, uuid, features);
      }
      else if (temp == "[Connection]")
      {
        string subscriberUUID;
        string subscriberFeatureUUID;
        string publisherUUID;
        string publisherFeatureUUID;
        double coefficient;
        getline(fileInput, subscriberUUID);
        getline(fileInput, subscriberFeatureUUID);
        getline(fileInput, publisherUUID);
        getline(fileInput, publisherFeatureUUID);
        getline(fileInput, temp);
        coefficient = stod(temp);
        connectFeatures(subscriberFeatureUUID, publisherFeatureUUID, coefficient);
        getline(fileInput, temp);
      }
      else
      {
        ROS_ERROR("Wrong line: %s", temp.c_str());
        error = true;
        break;
      }
    }
  }

  if (error)
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Error");
    msgBox.setText("Error loading configuration");
    msgBox.setInformativeText("Maybe the file doesn't exist or is no valid configuration file?");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.setIcon(QMessageBox::Warning);
    guitimer->stop();
    msgBox.exec();
    guitimer->start(timerInterval);
  }
}

void TapiGui::saveButtonClicked()
{
  string homedir = getenv("HOME");
  string filename = homedir + "/config.tapi";
  QString filePicker =
      QFileDialog::getSaveFileName(this, "Save File", QString::fromStdString(filename), "Tapi-files (*.tapi)");
  filename = filePicker.toStdString();
  ofstream fileOutput;
  fileOutput.open(filename);
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    fileOutput << "[Device]\n";
    fileOutput << it->second->GetUUID() << "\n";
    fileOutput << it->second->GetName() << "\n";
    fileOutput << (int)it->second->GetType() << "\n";
    fileOutput << it->second->GetHeartbeat() << "\n";
    vector<Tapi::Feature*> features = it->second->GetSortedFeatures();
    for (int j = 0; j < features.size(); j++)
    {
      fileOutput << "[DeviceFeature]\n";
      fileOutput << features.at(j)->GetUUID() << "\n";
      fileOutput << features.at(j)->GetName() << "\n";
      fileOutput << features.at(j)->GetType() << "\n";
    }
  }
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    fileOutput << "[Connection]\n";
    fileOutput << it->second.GetSubscriberUUID() << "\n";
    fileOutput << it->second.GetSubscriberFeatureUUID() << "\n";
    fileOutput << it->second.GetPublisherUUID() << "\n";
    fileOutput << it->second.GetPublisherFeatureUUID() << "\n";
    fileOutput << it->second.GetCoefficient() << "\n";
  }
  fileOutput.close();
}
}
