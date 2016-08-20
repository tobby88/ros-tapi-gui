#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include "device.hpp"
#include "feature.hpp"
#include <QWidget>
#include <vector>

class GuiDevice : public QWidget
{
  Q_OBJECT
public:
  GuiDevice(QWidget* parent, Device* device);
  QPoint featureBoxPosition(Feature* feature);


  vector<Feature*> features; // TODO: private
protected:
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  Device* device;
  int connectbox_size;
  int header_end;
  int footer_height;
  int footer_start;
  int line_height;
  int line_width;
  int line_start;
  int line_end;
  int items;
};

#endif // GuiDevice_H
