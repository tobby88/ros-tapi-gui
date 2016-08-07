#ifndef APIGUI_HPP
#define APIGUI_HPP

#include <QMainWindow>

namespace Ui
{
class ApiGui;
}

class ApiGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit ApiGui(QWidget* parent = 0);
  ~ApiGui();

private:
  Ui::ApiGui* ui;
};

#endif // APIGUI_HPP
