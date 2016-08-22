#ifndef APIGUI_HPP
#define APIGUI_HPP

#include <QMainWindow>
#include "api.hpp"
#include <QTimer>

namespace Ui
{
class ApiGui;
}

class ApiGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit ApiGui(Api* api, QWidget* parent = 0);
  ~ApiGui();

private:
  Ui::ApiGui* ui;
  Api* api;
  unsigned long int temp;
  QTimer *timer;

private slots:
  void checkApiForUpdate();
};

#endif // APIGUI_HPP
