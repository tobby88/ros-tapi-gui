#include "apigui.hpp"
#include "ui_apigui.h"

ApiGui::ApiGui(Api* api, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
  temp2 = 0;
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timer->start(100);
  // Add vertical layouts to the scroll Areas
  layoutConnections = new QVBoxLayout;
  layoutReceiver = new QVBoxLayout;
  layoutSender = new QVBoxLayout;
  ui->scrollAreaWidgetContentsConnections->setLayout(layoutConnections);
  ui->scrollAreaWidgetContentsReceiver->setLayout(layoutReceiver);
  ui->scrollAreaWidgetContentsSender->setLayout(layoutSender);
}

ApiGui::~ApiGui()
{
  delete timer;
  delete layoutReceiver;
  delete layoutSender;
  delete layoutConnections;
  delete ui;
}

void ApiGui::addDevice(Device* device)
{
  GuiDevice* guidevice = new GuiDevice(this, device);
  guidevice->is_input_device = false;
  layoutSender->addWidget(guidevice);
  guidevice->show(); // dont forget to show it ;)
  guidevices.push_back(guidevice);
}

void ApiGui::checkApiForUpdate()
{

  if (api->CheckPending())
  {
    temp2 = 0;
    vector<Device*> devices = api->GetDevicesSorted();
    for (vector<Device*>::iterator it = devices.begin(); it != devices.end(); it++)
    {
      addDevice(*it);
      temp2++;
    }
    ui->TestLabel->setText(QString::number(temp2));
    api->Done();
  }
}
