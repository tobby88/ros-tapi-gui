#include "maingui.hpp"
#include "ui_maingui.h"

namespace Tapi
{
// Constructor/Destructor

MainGui::MainGui(ros::NodeHandle* nh, QWidget* parent) : QMainWindow(parent), ui(new Ui::MainGui), nh(nh)
{
  api = new Tapi::Api(nh);
  ui->setupUi(this);
  QWidget::showMaximized();
  apiui = new Tapi::ApiGui(api, this);
  ui->scrollAreaWidgetContents->layout()->addWidget(apiui);
  apiui->show();
}

MainGui::~MainGui()
{
  delete ui;
}
}
