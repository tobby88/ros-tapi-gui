#include "apideviceswrapper.hpp"
#include <QBrush>
#include <QFont>

ApiDevicesWrapper::ApiDevicesWrapper(QObject* parent, Api* api)
    : QAbstractTableModel(parent)
{
  this->api = api;
}

ApiDevicesWrapper::~ApiDevicesWrapper() {}

int ApiDevicesWrapper::rowCount(const QModelIndex&) const
{
  //return api->GetDevices().size();
  return 10;
}

int ApiDevicesWrapper::columnCount(const QModelIndex&) const { return 3; }

QVariant ApiDevicesWrapper::data(const QModelIndex& index, int role) const
{
  int row = index.row();
  int col = index.column();

  switch (role)
  {
  case Qt::DisplayRole:
    if (row < api->GetDevices().size())
    {
      map<string, Device> devices = api->GetDevices();
      map<string, Device>::iterator it = devices.begin();
      for (int i = 0; i < row; i++)
      {
        it++;
      }
      return QString::fromStdString(it->second.getName());
    }
    break;
    /*case Qt::FontRole:
      if (row == 0 && col == 0) // change font only for cell(0,0)
      {
        QFont boldFont;
        boldFont.setBold(true);
        return boldFont;
      }
      break;
    case Qt::BackgroundRole:

      if (row == 1 && col == 2) // change background only for cell(1,2)
      {
        QBrush redBackground(Qt::red);
        return redBackground;
      }
      break;
    case Qt::CheckStateRole:

      if (row == 1 && col == 0) // add a checkbox to cell(1,0)
      {
        return Qt::Checked;
      }*/
  }
  return QVariant();
}
