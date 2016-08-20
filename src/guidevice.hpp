#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include "device.hpp"
#include "feature.hpp"
#include <QMouseEvent>
#include <QWidget>
#include <vector>

class GuiDevice : public QWidget
{
  Q_OBJECT

signals:
  void featureClicked(GuiDevice* guidevice, Feature* feature);

public:
  GuiDevice(QWidget* parent, Device* device);
  QPoint featureBoxPosition(Feature* feature);

  vector<Feature*> features; // TODO: private
  Device* device;            // TODO: getDevice() & private
protected:
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;

private:
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
