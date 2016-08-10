#include "apigui.hpp"
#include "guidevice.hpp"
#include "ui_apigui.h"

ApiGui::ApiGui(Api* api, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
  temp = 0;
  ui->TestLabel->setText(QString::number(temp));
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timer->start(2000);
  temp2 = 0;
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

void ApiGui::addDevice()
{
  GuiDevice* guidevice = new GuiDevice(this);
  guidevice->is_input_device = false;
  layoutSender->addWidget(guidevice);
  guidevice->show(); // dont forget to show it ;)
}

void ApiGui::checkApiForUpdate()
{
  temp++;
  ui->TestLabel->setText(QString::number(temp));
  addDevice();
}
