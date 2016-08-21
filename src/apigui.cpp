#include "apigui.hpp"
#include "assignment.hpp"
#include "ui_apigui.h"
#include <QCursor>
#include <QMessageBox>
#include <QPainter>

ApiGui::ApiGui(Api* api, QWidget* parent) : QWidget(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
  temp2 = 0;
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timerInterval = 15;
  timer->start(timerInterval);
  // Add vertical layouts to the scroll Areas
  layoutReceiver = ui->verticalLayoutReceiver;
  layoutSender = ui->verticalLayoutSender;

  selectedFeature = 0;
  selectedGuiDevice = 0;
}

ApiGui::~ApiGui()
{
  delete timer;
  delete ui;
}

void ApiGui::addDevice(Device* device)
{
  GuiDevice* guidevice = new GuiDevice(this, device);
  if (device->getType() == DeviceType::SenderDevice)
  {
    layoutSender->addWidget(guidevice);
    senderGuiDevices.push_back(guidevice);
  }
  else
  {
    layoutReceiver->addWidget(guidevice);
    receiverGuiDevices.push_back(guidevice);
  }
  guidevice->show(); // dont forget to show it ;)
  connect(guidevice, SIGNAL(featureClicked(GuiDevice*, Feature*)), this,
          SLOT(featureClicked(GuiDevice*, Feature*)));
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

  if (api->CheckPending())
  {
    for (vector<GuiDevice*>::iterator it = senderGuiDevices.begin();
         it != senderGuiDevices.end(); it++)
    {
      (*it)->hide();
      layoutSender->removeWidget(*it);
      delete *it;
    }
    senderGuiDevices.clear();
    for (vector<GuiDevice*>::iterator it = receiverGuiDevices.begin();
         it != receiverGuiDevices.end(); it++)
    {
      (*it)->hide();
      layoutReceiver->removeWidget(*it);
      delete *it;
    }
    receiverGuiDevices.clear();
    temp2 = 0;
    vector<Device*> devices = api->GetDevicesSorted();
    for (vector<Device*>::iterator it = devices.begin(); it != devices.end();
         it++)
    {
      addDevice(*it);
      temp2++;
    }
    ui->TestLabel->setText(QString::number(temp2));
    api->Done();
  }
}

void ApiGui::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setPen(Qt::black);

  // Draw all connections
  vector<Assignment*> connections;
  connections = api->GetConnections();
  for (vector<Assignment*>::iterator it = connections.begin();
       it != connections.end(); it++)
  {
    string senderUUID = (*it)->getSenderUUID();
    string senderFeatureUUID = (*it)->getSenderFeatureUUID();
    string receiverUUID = (*it)->getReceiverUUID();
    string receiverFeatureUUID = (*it)->getReceiverFeatureUUID();
    GuiDevice *sender, *receiver;
    sender = 0;
    receiver = 0;
    for (vector<GuiDevice*>::iterator it2 = senderGuiDevices.begin();
         it2 != senderGuiDevices.end(); it2++)
    {
      if ((*it2)->device->getUUID() == senderUUID)
      {
        sender = *it2;
        break;
      }
    }
    if (!sender)
      continue;
    for (vector<GuiDevice*>::iterator it2 = receiverGuiDevices.begin();
         it2 != receiverGuiDevices.end(); it2++)
    {
      if ((*it2)->device->getUUID() == receiverUUID)
      {
        receiver = *it2;
        break;
      }
    }
    if (!receiver)
      continue;
    QPoint begin, end;
    Feature* feature = sender->device->getFeatureByUUID(senderFeatureUUID);
    if (!feature)
      continue;
    begin = sender->mapTo(this, sender->featureBoxPosition(feature));
    feature = receiver->device->getFeatureByUUID(receiverFeatureUUID);
    if (!feature)
      continue;
    end = receiver->mapTo(this, receiver->featureBoxPosition(feature));
    painter.drawLine(begin, end);
  }

  // Draw line for current (pending) connection
  if (!selectedFeature)
    return;

  GuiDevice* s = selectedGuiDevice;
  QPoint end = mapFromGlobal(mousePosition);

  Feature* fs = selectedFeature;

  QPoint begin = s->mapTo(this, s->featureBoxPosition(fs));
  painter.drawLine(begin, end);
}

void ApiGui::featureClicked(GuiDevice* guidevice, Feature* feature)
{
  QString qs = QString::fromStdString(feature->getName());
  ui->TestLabel->setText(qs);

  if (selectedFeature && selectedFeature == feature)
  // Clicked twice on the same feature -> demarcate selection
  {
    selectedFeature = 0;
    selectedGuiDevice = 0;
    update();
    return;
  }

  if (guidevice->device->getType() == DeviceType::ReceiverDevice &&
      feature->getConnectionCount() > 0)
  {
    QMessageBox msgBox;
    if (selectedFeature)
    {
      string msg = "Replace connection of \"" + feature->getName() + "\"?";
      msgBox.setWindowTitle("Feature already connected");
      msgBox.setText(QString::fromStdString(msg));
    }
    else
    {
      string msg = "Delete connection of \"" + feature->getName() + "\"?";
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
      api->DeleteConnection(feature->getUUID());
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

  if (guidevice->device->getType() == selectedGuiDevice->device->getType())
  // Selected the same device type, so no connection is possible. Drop old
  // selection and select the new one
  {
    selectedFeature = feature;
    selectedGuiDevice = guidevice;
    return;
  }

  if (selectedFeature->getType() != feature->getType())
  // Selected different feature types, so no connection is possible. Drop old
  // selection and select the new one
  {
    selectedFeature = feature;
    selectedGuiDevice = guidevice;
    return;
  }

  // Everything ok -> try to connect them
  api->ConnectFeatures(selectedFeature->getUUID(), feature->getUUID());
  selectedFeature = 0;
  selectedGuiDevice = 0;
  update();
}
