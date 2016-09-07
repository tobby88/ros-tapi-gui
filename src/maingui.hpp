#ifndef MAINGUI_HPP
#define MAINGUI_HPP

#include <QMainWindow>
#include "ros/node_handle.h"
#include "tapigui.hpp"

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
  MainGui(ros::NodeHandle* nh, QWidget* parent = 0);
  ~MainGui();

private:
  // Private member variables
  Tapi::TapiGui* tapiGui;
  Ui::MainGui* ui;
};
}

#endif  // MAINGUI_HPP
