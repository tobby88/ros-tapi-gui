#include "guidevice.hpp"
#include <QPainter>
#include <QString>
#include "tobbyapi_msgs/HelloRequest.h"

using namespace std;

// Constructor/Destructor

GuiDevice::GuiDevice(QWidget* parent, Device* device) : QWidget(parent)
{
  this->device = device;
  connectbox_size = 10;
  header_end = 30;
  footer_height = 10;
  line_height = 20;
  line_start = connectbox_size;
  features = device->GetSortedFeatures();
  items = features.size();

  // min height
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);
}

GuiDevice::~GuiDevice()
{
}

// Public member functions

QPoint GuiDevice::FeatureBoxPosition(Feature* feature)
{
  int x, y, i;
  y = 0;
  if (device->GetType() == tobbyapi_msgs::HelloRequest::Type_ReceiverDevice)
    x = 0;
  else
    x = this->width() - 1;
  i = 0;
  for (auto it = features.begin(); it != features.end(); it++)
  {
    if (*it == feature)
      y = header_end + i * line_height + line_height / 2;
    i++;
  }
  return QPoint(x, y);
}

Device* GuiDevice::GetDevice()
{
  return device;
}

vector<Feature*> GuiDevice::GetFeatures()
{
  return features;
}

// Protected member functions

void GuiDevice::mouseReleaseEvent(QMouseEvent* event)
{
  int i = 0;
  QPoint p = event->pos();
  for (auto it = features.begin(); it != features.end(); it++)
  {
    int y = header_end + i * line_height;

    if (p.y() >= y && p.y() < y + line_height)
    {
      emit featureClicked(this, *it);
    }
    i++;
  }
}

void GuiDevice::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  footer_start = this->height() - footer_height;
  line_width = this->width() - 2 * connectbox_size - 1;
  line_end = line_start + line_width;

  // draw main box
  if (device->Active())
    painter.setBrush(Qt::white);
  else
    painter.setBrush(Qt::gray);
  painter.setPen(Qt::black);

  QSize a(line_width, this->height() - 1);
  painter.drawRoundedRect(QRect(QPoint(line_start, 0), a), 10, 10);

  // print heading
  painter.setFont(QFont("Arial", 11));
  painter.drawText(QRect(QPoint(line_start, 0), QPoint(line_end, header_end)), Qt::AlignCenter,
                   QString::fromStdString(device->GetName()));

  // Draw Features
  int i = 0;
  for (auto it = features.begin(); it != features.end(); it++)
  {
    int line_y = header_end + i * line_height;

    QColor color;
    switch ((*it)->GetType())
    {
      case tobbyapi_msgs::Feature::Type_AnalogValue:
        color = Qt::red;
        break;
      case tobbyapi_msgs::Feature::Type_Images:
        color = Qt::green;
        break;
      case tobbyapi_msgs::Feature::Type_Switch:
        color = Qt::blue;
        break;
      case tobbyapi_msgs::Feature::Type_Tristate:
        color = Qt::cyan;
        break;
      default:
        color = QColor("#f1aa00");
        break;
    }
    if (!device->Active())
      color = color.darker();
    painter.setBrush(color);

    painter.drawLine(QPoint(line_start, line_y), QPoint(line_end, line_y));

    // draw connect box
    QRect connect_box(QPoint(0, line_y + line_height / 2 - connectbox_size / 2),
                      QSize(connectbox_size, connectbox_size));

    if (device->GetType() == tobbyapi_msgs::HelloRequest::Type_SenderDevice)
    {
      connect_box.moveLeft(line_end);
    }
    painter.drawRect(connect_box);

    painter.setFont(QFont("Arial", 10));
    painter.drawText(QRect(QPoint(line_start, line_y), QPoint(line_end, line_y + line_height)), Qt::AlignCenter,
                     QString::fromStdString((*it)->GetName()));
    i++;
  }

  // footer line
  painter.drawLine(QPoint(line_start, footer_start), QPoint(line_end, footer_start));
}
