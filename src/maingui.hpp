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
  // Constructor/Destructor
  MainGui(Api* api, QWidget* parent = 0);
  ~MainGui();

private:
  // Private member variables
  Api* api;
  ApiGui* apiui;
  Ui::MainGui* ui;
};

#endif // MAINGUI_HPP
