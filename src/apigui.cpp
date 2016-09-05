#include "apigui.hpp"
#include <QCursor>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QString>
#include <fstream>
#include <string>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tapi_msgs/Connect.h"
#include "tapi_msgs/Feature.h"
#include "tapi_msgs/GetConnectionList.h"
#include "tapi_msgs/GetDeviceList.h"
#include "tapi_msgs/Hello.h"
#include "ui_apigui.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

ApiGui::ApiGui(ros::NodeHandle* nh, QWidget* parent) : QWidget(parent), ui(new Ui::ApiGui), nh(nh)
{
  spinner = new ros::AsyncSpinner(1);
  pendingChanges = false;
  devListClient = nh->serviceClient<tapi_msgs::GetDeviceList>("Tapi/GetDeviceList");
  delPub = nh->advertise<std_msgs::String>("Tapi/DeleteConnection", 1000);
  conPub = nh->advertise<tapi_msgs::Connect>("Tapi/ConnectFeatures", 1000);
  conListClient = nh->serviceClient<tapi_msgs::GetConnectionList>("Tapi/GetConnectionList");
  clearPub = nh->advertise<std_msgs::Bool>("Tapi/Clear", 2);
  helloClient = nh->serviceClient<tapi_msgs::Hello>("Tapi/HelloServ");
  updateTimer.start();

  ui->setupUi(this);
  guitimer = new QTimer(this);
  connect(guitimer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timerInterval = 15;
  guitimer->start(timerInterval);

  // Add vertical layouts to the scroll Areas
  layoutReceiver = ui->verticalLayoutReceiver;
  layoutSender = ui->verticalLayoutSender;
  layoutReceiver->setAlignment(Qt::AlignTop);
  layoutSender->setAlignment(Qt::AlignTop);
  ui->verticalLayoutKeys->setAlignment(Qt::AlignTop);

  selectedFeature = 0;
  selectedGuiDevice = 0;

  connect(ui->loadButton, SIGNAL(clicked(bool)), this, SLOT(loadButtonClicked()));
  connect(ui->saveButton, SIGNAL(clicked(bool)), this, SLOT(saveButtonClicked()));
  connect(ui->clearButton, SIGNAL(clicked(bool)), this, SLOT(clearButtonClicked()));

  lastUpdatedSub = nh->subscribe("Tapi/LastChanged", 5, &ApiGui::updateAvailable, this);
  run();
  updateData();
  updateTimer = nh->createTimer(ros::Duration(CHECK_INTERVAL / 1000.0), &ApiGui::timer, this);
}

ApiGui::~ApiGui()
{
  delete guitimer;
  delete ui;

  updateTimer.stop();
  spinner->stop();
  delete spinner;
  devListClient.shutdown();
  lastUpdatedSub.shutdown();
  delPub.shutdown();
  conPub.shutdown();
  clearPub.shutdown();
  helloClient.shutdown();
}

// Protected member functions

void ApiGui::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setPen(Qt::black);

  // Draw all connections
  vector<Tapi::Connection*> connections;
  connections = getConnections();
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    string senderUUID = (*it)->GetSenderUUID();
    string senderFeatureUUID = (*it)->GetSenderFeatureUUID();
    string receiverUUID = (*it)->GetReceiverUUID();
    string receiverFeatureUUID = (*it)->GetReceiverFeatureUUID();
    Tapi::GuiDevice *sender, *receiver;
    sender = 0;
    receiver = 0;
    for (auto it2 = senderGuiDevices.begin(); it2 != senderGuiDevices.end(); ++it2)
    {
      if ((*it2)->GetDevice()->GetUUID() == senderUUID)
      {
        sender = *it2;
        break;
      }
    }
    if (!sender)
      continue;
    for (auto it2 = receiverGuiDevices.begin(); it2 != receiverGuiDevices.end(); ++it2)
    {
      if ((*it2)->GetDevice()->GetUUID() == receiverUUID)
      {
        receiver = *it2;
        break;
      }
    }
    if (!receiver)
      continue;
    QPoint begin, end;
    Tapi::Feature* feature = sender->GetDevice()->GetFeatureByUUID(senderFeatureUUID);
    if (!feature)
      continue;
    begin = sender->mapTo(this, sender->FeatureBoxPosition(feature));
    feature = receiver->GetDevice()->GetFeatureByUUID(receiverFeatureUUID);
    if (!feature)
      continue;
    end = receiver->mapTo(this, receiver->FeatureBoxPosition(feature));
    painter.drawLine(begin, end);
  }

  // Draw line for current (pending) connection
  if (!selectedFeature)
    return;

  Tapi::GuiDevice* s = selectedGuiDevice;
  QPoint end = mapFromGlobal(mousePosition);

  Tapi::Feature* fs = selectedFeature;

  QPoint begin = s->mapTo(this, s->FeatureBoxPosition(fs));
  painter.drawLine(begin, end);
}

// Private member functions

void ApiGui::addDevice(Tapi::Device* device)
{
  Tapi::GuiDevice* guidevice = new Tapi::GuiDevice(this, device);
  if (device->GetType() == tapi_msgs::HelloRequest::Type_SenderDevice)
  {
    layoutSender->addWidget(guidevice);
    senderGuiDevices.push_back(guidevice);
  }
  else
  {
    layoutReceiver->addWidget(guidevice);
    receiverGuiDevices.push_back(guidevice);
  }
  guidevice->show();  // dont forget to show it ;)
  connect(guidevice, SIGNAL(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)), this,
          SLOT(featureClicked(Tapi::GuiDevice*, Tapi::Feature*)));
}

void ApiGui::addDevice(uint8_t type, string name, string uuid, map<string, Tapi::Feature> features)
{
  tapi_msgs::Hello hello;
  hello.request.DeviceType = type;
  vector<tapi_msgs::Feature> featureVec;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    tapi_msgs::Feature feature;
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
  if (hello.response.Status == tapi_msgs::HelloResponse::StatusError)
    ROS_ERROR("Error when connection to hello service");
}

void ApiGui::changed()
{
  pendingChanges = true;
}

bool ApiGui::checkPending()
{
  return pendingChanges;
}

void ApiGui::clear()
{
  std_msgs::Bool msg;
  msg.data = true;
  clearPub.publish(msg);
  connections.clear();
  devices.clear();
}

bool ApiGui::compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second)
{
  return first->GetName() < second->GetName();
}

bool ApiGui::connectFeatures(string feature1uuid, string feature2uuid, double coefficient)
{
  tapi_msgs::Connect msg;
  msg.Coefficient = coefficient;
  msg.Feature1UUID = feature1uuid;
  msg.Feature2UUID = feature2uuid;
  conPub.publish(msg);
  return true;
}

bool ApiGui::deleteConnection(string receiverFeatureUUID)
{
  std_msgs::String msg;
  msg.data = receiverFeatureUUID;
  delPub.publish(msg);
  changed();
}

void ApiGui::done()
{
  pendingChanges = false;
}

vector<Tapi::Connection*> ApiGui::getConnections()
{
  vector<Tapi::Connection*> connectionList;
  for (auto it = connections.begin(); it != connections.end(); ++it)
    connectionList.push_back(&it->second);
  return connectionList;
}

Tapi::Device* ApiGui::getDeviceByFeatureUUID(string uuid)
{
  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    if (it->second.GetFeatureByUUID(uuid))
      return &(it->second);
  }
  return 0;
}

vector<Tapi::Device*> ApiGui::getDevicesSorted()
{
  vector<Tapi::Device*> devicesList;
  for (auto it = devices.begin(); it != devices.end(); ++it)
    devicesList.push_back(&it->second);
  if (devicesList.size() > 1)
    sort(devicesList.begin(), devicesList.end(), compareDeviceNames);
  return devicesList;
}

void ApiGui::run()
{
  spinner->start();
}

void ApiGui::timer(const ros::TimerEvent& e)
{
  updateData();
}

void ApiGui::updateAvailable(const std_msgs::Time::ConstPtr& time)
{
  if (time->data.toNSec() > lastUpdated.toNSec())
  {
    lastUpdated = time->data;
    updateData();
  }
}

void ApiGui::updateData()
{
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
    if (devices.count(it->ReceiverUUID) == 0 || devices.count(it->SenderUUID) == 0)
      continue;
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

// Slot functions

void ApiGui::clearButtonClicked()
{
  for (auto it = senderGuiDevices.begin(); it != senderGuiDevices.end(); ++it)
  {
    (*it)->hide();
    layoutSender->removeWidget(*it);
    delete *it;
  }
  senderGuiDevices.clear();
  for (auto it = receiverGuiDevices.begin(); it != receiverGuiDevices.end(); ++it)
  {
    (*it)->hide();
    layoutReceiver->removeWidget(*it);
    delete *it;
  }
  receiverGuiDevices.clear();
  clear();
  update();
}

void ApiGui::checkApiForUpdate()
{
  if (selectedFeature)
  {
    QPoint newMousePosition = QCursor::pos();
    if (newMousePosition != mousePosition)
    {
      mousePosition = newMousePosition;
      update();
    }
  }

  if (checkPending())
  {
    for (auto it = senderGuiDevices.begin(); it != senderGuiDevices.end(); ++it)
    {
      (*it)->hide();
      layoutSender->removeWidget(*it);
      delete *it;
    }
    senderGuiDevices.clear();
    for (auto it = receiverGuiDevices.begin(); it != receiverGuiDevices.end(); ++it)
    {
      (*it)->hide();
      layoutReceiver->removeWidget(*it);
      delete *it;
    }
    receiverGuiDevices.clear();
    vector<Tapi::Device*> devices = getDevicesSorted();
    for (auto it = devices.begin(); it != devices.end(); ++it)
    {
      addDevice(*it);
      vector<Tapi::Feature*> features = (*it)->GetSortedFeatures();
      for (auto it2 = features.begin(); it2 != features.end(); ++it2)
      {
        string type = (*it2)->GetType();
        if (colorKeys.count(type) < 1)
        {
          QColor color = GuiDevice::stringToColor(type);
          colorKeys.emplace(type, color);

          QLabel* key = new QLabel();

          QString format("<font color=\"%1\">%2</font>  %3");
          key->setText(format.arg(color.name(), "█  ", type.c_str()));
          ui->verticalLayoutKeys->addWidget(key);
          key->show();
        }
      }
    }
    done();
    // Reselect Guidevice, because the selection was deleted above
    if (selectedFeature)
    {
      bool found = false;
      for (auto it = senderGuiDevices.begin(); it != senderGuiDevices.end(); ++it)
        if ((*it)->GetDevice()->GetFeatureByUUID(selectedFeature->GetUUID()) != 0)
        {
          selectedGuiDevice = *it;
          found = true;
        }
      for (auto it = receiverGuiDevices.begin(); it != receiverGuiDevices.end(); ++it)
        if ((*it)->GetDevice()->GetFeatureByUUID(selectedFeature->GetUUID()) != 0)
        {
          selectedGuiDevice = *it;
          found = true;
        }
      if (!found)
      {
        selectedGuiDevice = 0;
        selectedFeature = 0;
      }
    }
    update();
  }
}

void ApiGui::featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature)
{
  QString qs = QString::fromStdString(feature->GetName());
  ui->TestLabel->setText(qs);

  if ((selectedFeature && selectedFeature == feature) || !guidevice->GetDevice()->Active())
  // Clicked twice on the same feature or device is inactive -> demarcate selection
  {
    selectedFeature = 0;
    selectedGuiDevice = 0;
    update();
    return;
  }

  if (guidevice->GetDevice()->GetType() == tapi_msgs::HelloRequest::Type_ReceiverDevice &&
      feature->GetConnectionCount() > 0)
  {
    QMessageBox msgBox;
    if (selectedFeature && guidevice->GetDevice()->GetType() != selectedGuiDevice->GetDevice()->GetType() &&
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

  if (guidevice->GetDevice()->GetType() == selectedGuiDevice->GetDevice()->GetType())
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

void ApiGui::loadButtonClicked()
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
    clear();
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
        addDevice(type, name, uuid, features);
      }
      else if (temp == "[Connection]")
      {
        string receiverUUID;
        string receiverFeatureUUID;
        string senderUUID;
        string senderFeatureUUID;
        double coefficient;
        getline(fileInput, receiverUUID);
        getline(fileInput, receiverFeatureUUID);
        getline(fileInput, senderUUID);
        getline(fileInput, senderFeatureUUID);
        getline(fileInput, temp);
        coefficient = stod(temp);
        connectFeatures(receiverFeatureUUID, senderFeatureUUID, coefficient);
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

void ApiGui::saveButtonClicked()
{
  vector<Tapi::Device*> devices = getDevicesSorted();
  vector<Tapi::Connection*> connections = getConnections();
  string homedir = getenv("HOME");
  string filename = homedir + "/config.tapi";
  QString filePicker =
      QFileDialog::getSaveFileName(this, "Save File", QString::fromStdString(filename), "Tapi-files (*.tapi)");
  filename = filePicker.toStdString();
  ofstream fileOutput;
  fileOutput.open(filename);
  for (int i = 0; i < devices.size(); i++)
  {
    fileOutput << "[Device]\n";
    fileOutput << devices.at(i)->GetUUID() << "\n";
    fileOutput << devices.at(i)->GetName() << "\n";
    fileOutput << (int)devices.at(i)->GetType() << "\n";
    fileOutput << devices.at(i)->GetHeartbeat() << "\n";
    vector<Tapi::Feature*> features = devices.at(i)->GetSortedFeatures();
    for (int j = 0; j < features.size(); j++)
    {
      fileOutput << "[DeviceFeature]\n";
      fileOutput << features.at(j)->GetUUID() << "\n";
      fileOutput << features.at(j)->GetName() << "\n";
      fileOutput << features.at(j)->GetType() << "\n";
    }
  }
  for (int i = 0; i < connections.size(); i++)
  {
    fileOutput << "[Connection]\n";
    fileOutput << connections.at(i)->GetReceiverUUID() << "\n";
    fileOutput << connections.at(i)->GetReceiverFeatureUUID() << "\n";
    fileOutput << connections.at(i)->GetSenderUUID() << "\n";
    fileOutput << connections.at(i)->GetSenderFeatureUUID() << "\n";
    fileOutput << connections.at(i)->GetCoefficient() << "\n";
  }
  fileOutput.close();
}
}
