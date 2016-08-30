#ifndef MAINGUI_HPP
#define MAINGUI_HPP

#include <QMainWindow>
#include "api.hpp"
#include "apigui.hpp"

namespace Ui
{
class MainGui;
}

namespace Tapi
{
class MainGui : public QMainWindow
{
  Q_OBJECT

public:
  // Constructor/Destructor
  MainGui(Tapi::Api* api, QWidget* parent = 0);
  ~MainGui();

private:
  // Private member variables
  Tapi::Api* api;
  Tapi::ApiGui* apiui;
  Ui::MainGui* ui;
};
}

#endif  // MAINGUI_HPP
