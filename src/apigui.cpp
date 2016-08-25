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
#include "tobbyapi_msgs/Feature.h"
#include "tobbyapi_msgs/HelloRequest.h"
#include "ui_apigui.h"

using namespace std;

// Constructor/Destructor

ApiGui::ApiGui(Api* api, QWidget* parent) : QWidget(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timerInterval = 15;
  timer->start(timerInterval);
  // Add vertical layouts to the scroll Areas
  layoutReceiver = ui->verticalLayoutReceiver;
  layoutSender = ui->verticalLayoutSender;

  selectedFeature = 0;
  selectedGuiDevice = 0;
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
  for (auto it = connections.begin(); it != connections.end(); it++)
  {
    string senderUUID = (*it)->GetSenderUUID();
    string senderFeatureUUID = (*it)->GetSenderFeatureUUID();
    string receiverUUID = (*it)->GetReceiverUUID();
    string receiverFeatureUUID = (*it)->GetReceiverFeatureUUID();
    GuiDevice *sender, *receiver;
    sender = 0;
    receiver = 0;
    for (auto it2 = senderGuiDevices.begin(); it2 != senderGuiDevices.end(); it2++)
    {
      if ((*it2)->GetDevice()->GetUUID() == senderUUID)
      {
        sender = *it2;
        break;
      }
    }
    if (!sender)
      continue;
    for (auto it2 = receiverGuiDevices.begin(); it2 != receiverGuiDevices.end(); it2++)
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
  if (device->GetType() == tobbyapi_msgs::HelloRequest::Type_SenderDevice)
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
    for (auto it = senderGuiDevices.begin(); it != senderGuiDevices.end(); it++)
    {
      (*it)->hide();
      layoutSender->removeWidget(*it);
      delete *it;
    }
    senderGuiDevices.clear();
    for (auto it = receiverGuiDevices.begin(); it != receiverGuiDevices.end(); it++)
    {
      (*it)->hide();
      layoutReceiver->removeWidget(*it);
      delete *it;
    }
    receiverGuiDevices.clear();
    vector<Device*> devices = api->GetDevicesSorted();
    for (auto it = devices.begin(); it != devices.end(); it++)
      addDevice(*it);
    api->Done();
    // Reselect Guidevice, because the selection was deleted above
    if (selectedFeature)
    {
      bool found = false;
      for (auto it = senderGuiDevices.begin(); it != senderGuiDevices.end(); it++)
        if ((*it)->GetDevice()->GetFeatureByUUID(selectedFeature->GetUUID()) != 0)
        {
          selectedGuiDevice = *it;
          found = true;
        }
      for (auto it = receiverGuiDevices.begin(); it != receiverGuiDevices.end(); it++)
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

  if (selectedFeature && selectedFeature == feature)
  // Clicked twice on the same feature -> demarcate selection
  {
    selectedFeature = 0;
    selectedGuiDevice = 0;
    update();
    return;
  }

  if (guidevice->GetDevice()->GetType() == tobbyapi_msgs::HelloRequest::Type_ReceiverDevice &&
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
  if (feature->GetType() == tobbyapi_msgs::Feature::Type_AnalogValue)
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

void ApiGui::saveButtonClicked()
{
  vector<Device*> devices = api->GetDevicesSorted();
  vector<Assignment*> connections = api->GetConnections();
  string homedir = getenv("HOME");
  string filename = homedir + "/config.tobbyapi";
  QString filePicker =
      QFileDialog::getSaveFileName(this, "Save File", QString::fromStdString(filename), "TobbyAPI-Files (*.tobbyapi)");
  filename = filePicker.toStdString();
  ofstream FileOutput;
  FileOutput.open(filename);
  for (int i = 0; i < devices.size(); i++)
  {
    FileOutput << "[Device]\n";
    FileOutput << devices.at(i)->GetUUID() << "\n";
    FileOutput << devices.at(i)->GetName() << "\n";
    FileOutput << (int)devices.at(i)->GetType() << "\n";
    FileOutput << devices.at(i)->GetHeartbeat() << "\n";
    FileOutput << "[DeviceFeatures]\n";
    vector<Feature*> features = devices.at(i)->GetSortedFeatures();
    for (int j = 0; j < features.size(); j++)
    {
      FileOutput << features.at(j)->GetUUID() << "\n";
      FileOutput << features.at(j)->GetName() << "\n";
      FileOutput << features.at(j)->GetDescription() << "\n";
      FileOutput << (int)features.at(j)->GetType() << "\n";
    }
  }
  for (int i = 0; i < connections.size(); i++)
  {
    FileOutput << "[Connection]\n";
    FileOutput << connections.at(i)->GetReceiverUUID() << "\n";
    FileOutput << connections.at(i)->GetReceiverFeatureUUID() << "\n";
    FileOutput << connections.at(i)->GetSenderUUID() << "\n";
    FileOutput << connections.at(i)->GetSenderFeatureUUID() << "\n";
    FileOutput << connections.at(i)->GetCoefficient() << "\n";
  }
  FileOutput.close();
}
