#include "maingui.hpp"
#include "ui_maingui.h"

namespace Tapi
{
// Constructor/Destructor

MainGui::MainGui(ros::NodeHandle* nh, QWidget* parent) : QMainWindow(parent), ui(new Ui::MainGui)
{
  ui->setupUi(this);
  QWidget::showMaximized();
  tapiGui = new Tapi::TapiGui(nh, this);
  ui->scrollAreaWidgetContents->layout()->addWidget(tapiGui);
  tapiGui->show();
}

MainGui::~MainGui()
{
  delete ui;
}
}
