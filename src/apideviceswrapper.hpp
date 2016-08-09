#ifndef APIDEVICESWRAPPER_HPP
#define APIDEVICESWRAPPER_HPP

#include <QAbstractTableModel>
#include "api.hpp"

class ApiDevicesWrapper : public QAbstractTableModel
{
  Q_OBJECT
public:
  ApiDevicesWrapper(QObject* parent, Api* api);
  ~ApiDevicesWrapper();
  int rowCount(const QModelIndex& parent = QModelIndex()) const Q_DECL_OVERRIDE;
  int columnCount(const QModelIndex& parent = QModelIndex()) const
      Q_DECL_OVERRIDE;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;

private:
  Api* api;
};

#endif // APIDEVICESWRAPPER_HPP
