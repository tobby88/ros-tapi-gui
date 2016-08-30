#include "maingui.hpp"
#include "ui_maingui.h"

namespace Tapi
{
// Constructor/Destructor

MainGui::MainGui(Tapi::Api* api, QWidget* parent) : QMainWindow(parent), ui(new Ui::MainGui), api(api)
{
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
