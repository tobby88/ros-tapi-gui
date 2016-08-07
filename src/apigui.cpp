#include "apigui.hpp"
#include "ui_apigui.h"

ApiGui::ApiGui(QWidget* parent) : QMainWindow(parent), ui(new Ui::ApiGui)
{
  ui->setupUi(this);
}

ApiGui::~ApiGui() { delete ui; }
