#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include <QMouseEvent>
#include <QWidget>
#include <vector>
#include "device.hpp"
#include "feature.hpp"

class GuiDevice : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor
  GuiDevice(QWidget* parent, Device* device);
  ~GuiDevice();

  // Public member functions
  QPoint FeatureBoxPosition(Feature* feature);
  Device* GetDevice();
  std::vector<Feature*> GetFeatures();

protected:
  // Protected member functions
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  int connectbox_size;
  Device* device;
  std::vector<Feature*> features;
  int footer_height;
  int footer_start;
  int header_end;
  int items;
  int line_end;
  int line_height;
  int line_start;
  int line_width;

signals:
  // Signals without implementation
  void featureClicked(GuiDevice* guidevice, Feature* feature);
};

#endif  // GuiDevice_H
