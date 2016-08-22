#include "apigui.hpp"
#include "ui_apigui.h"

ApiGui::ApiGui(Api *api, QWidget* parent) : QMainWindow(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
  temp=0;
  ui->TestLabel->setText(QString::number(temp));
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(checkApiForUpdate()));
  timer->start(100);
}

ApiGui::~ApiGui() {
  delete timer;
  delete ui; }

void ApiGui::checkApiForUpdate()
{
  temp++;
  ui->TestLabel->setText(QString::number(temp));
}
