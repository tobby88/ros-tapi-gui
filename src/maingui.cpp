#include "maingui.hpp"
#include "ui_maingui.h"

MainGui::MainGui(Api* api, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainGui)
{
  this->api = api;
  ui->setupUi(this);
  apiui = new ApiGui(api, this);
  ui->scrollAreaWidgetContents->layout()->addWidget(apiui);
  apiui->show();
}

MainGui::~MainGui() {}
