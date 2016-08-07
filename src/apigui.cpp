#include "apigui.hpp"
#include "ui_apigui.h"

ApiGui::ApiGui(Api *api, QWidget* parent) : QMainWindow(parent), ui(new Ui::ApiGui)
{
  this->api = api;
  ui->setupUi(this);
}

ApiGui::~ApiGui() { delete ui; }
