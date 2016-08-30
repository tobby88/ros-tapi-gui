#include "apigui.hpp"
#include <QCursor>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QString>
#include <fstream>
#include <string>
#include "assignment.hpp"
#include "tapi_msgs/Feature.h"
#include "tapi_msgs/HelloRequest.h"
#include "ui_apigui.h"

using namespace std;

// Constructor/Destructor

ApiGui::ApiGui(Api* api, QWidget* parent) : QWidget(parent), ui(new Ui::ApiGui), api(api)
{
  ui->setupUi(this);
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timerInterval = 15;
  timer->start(timerInterval);

  ui->keyAnalogValue->setText("<font color='red'>█</font>  Analog Value/Axis");
  ui->keyImages->setText("<font color='green'>█</font>  Images/Camera");
  ui->keySwitch->setText("<font color='blue'>█</font>  Switch/Button");
  ui->keyTristate->setText("<font color='cyan'>█</font>  Tristate");

  // Add vertical layouts to the scroll Areas
  layoutReceiver = ui->verticalLayoutReceiver;
  layoutSender = ui->verticalLayoutSender;

  selectedFeature = 0;
  selectedGuiDevice = 0;

  connect(ui->loadButton, SIGNAL(clicked(bool)), this, SLOT(loadButtonClicked()));
  connect(ui->saveButton, SIGNAL(clicked(bool)), this, SLOT(saveButtonClicked()));
}

ApiGui::~ApiGui()
{
  delete timer;
  delete ui;
}

// Protected member functions

void ApiGui::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setPen(Qt::black);

  // Draw all connections
  vector<Assignment*> connections;
  connections = api->GetConnections();
  for (auto it = connections.begin(); it != connections.end(); ++it)
  {
    string senderUUID = (*it)->GetSenderUUID();
    string senderFeatureUUID = (*it)->GetSenderFeatureUUID();
    string receiverUUID = (*it)->GetReceiverUUID();
    string receiverFeatureUUID = (*it)->GetReceiverFeatureUUID();
    GuiDevice *sender, *receiver;
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
    Feature* feature = sender->GetDevice()->GetFeatureByUUID(senderFeatureUUID);
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

  GuiDevice* s = selectedGuiDevice;
  QPoint end = mapFromGlobal(mousePosition);

  Feature* fs = selectedFeature;

  QPoint begin = s->mapTo(this, s->FeatureBoxPosition(fs));
  painter.drawLine(begin, end);
}

// Private member functions

void ApiGui::addDevice(Device* device)
{
  GuiDevice* guidevice = new GuiDevice(this, device);
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
  connect(guidevice, SIGNAL(featureClicked(GuiDevice*, Feature*)), this, SLOT(featureClicked(GuiDevice*, Feature*)));
}

// Slot functions

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

  if (api->CheckPending())
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
    vector<Device*> devices = api->GetDevicesSorted();
    for (auto it = devices.begin(); it != devices.end(); ++it)
      addDevice(*it);
    api->Done();
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

void ApiGui::featureClicked(GuiDevice* guidevice, Feature* feature)
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
    timer->stop();
    int ret = msgBox.exec();
    timer->start(timerInterval);
    switch (ret)
    {
      case QMessageBox::No:
        return;
      case QMessageBox::Yes:
        api->DeleteConnection(feature->GetUUID());
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
  if (feature->GetType() == tapi_msgs::Feature::Type_AnalogValue)
  {
    timer->stop();
    QString input =
        QInputDialog::getText(this, "Coefficient?", "Multiply all values with:", QLineEdit::Normal, "1.0", &ok);
    coefficient = input.toDouble(&ok);
    timer->start(timerInterval);
  }
  if (ok)
  {
    api->ConnectFeatures(selectedFeature->GetUUID(), feature->GetUUID(), coefficient);
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
    api->Clear();
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
        map<string, Feature> features;
        while (temp == "[DeviceFeature]")
        {
          string featureUUID;
          string featureName;
          string featureDescription;
          uint8_t featureType;
          getline(fileInput, featureUUID);
          getline(fileInput, featureName);
          getline(fileInput, featureDescription);
          getline(fileInput, temp);
          featureType = (uint8_t)stoi(temp);
          Feature feature = Feature(featureType, featureName, featureDescription, featureUUID);
          features.emplace(featureUUID, feature);
          getline(fileInput, temp);
        }
        api->AddDeviceWithoutHello(type, name, uuid, heartbeat, features);
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
        api->ConnectFeatures(receiverFeatureUUID, senderFeatureUUID, coefficient);
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
    timer->stop();
    msgBox.exec();
    timer->start(timerInterval);
  }
}

void ApiGui::saveButtonClicked()
{
  vector<Device*> devices = api->GetDevicesSorted();
  vector<Assignment*> connections = api->GetConnections();
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
    vector<Feature*> features = devices.at(i)->GetSortedFeatures();
    for (int j = 0; j < features.size(); j++)
    {
      fileOutput << "[DeviceFeature]\n";
      fileOutput << features.at(j)->GetUUID() << "\n";
      fileOutput << features.at(j)->GetName() << "\n";
      fileOutput << features.at(j)->GetDescription() << "\n";
      fileOutput << (int)features.at(j)->GetType() << "\n";
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
