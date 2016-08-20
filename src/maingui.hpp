#ifndef MAINGUI_HPP
#define MAINGUI_HPP

#include "api.hpp"
#include "apigui.hpp"
#include <QMainWindow>

namespace Ui
{
class MainGui;
}

class MainGui : public QMainWindow
{
  Q_OBJECT

public:
  MainGui(Api* api, QWidget* parent = 0);
  ~MainGui();

private:
  Ui::MainGui* ui;
  Api* api;
  ApiGui* apiui;
};

#endif // MAINGUI_HPP
