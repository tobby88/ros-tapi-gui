#ifndef APIDEVICESWRAPPER_HPP
#define APIDEVICESWRAPPER_HPP

#include <QAbstractTableModel>

class ApiDevicesWrapper : public QAbstractTableModel
{
  Q_OBJECT
public:
  ApiDevicesWrapper(QObject* parent);
  ~ApiDevicesWrapper();
  int rowCount(const QModelIndex& parent = QModelIndex()) const Q_DECL_OVERRIDE;
  int columnCount(const QModelIndex& parent = QModelIndex()) const
      Q_DECL_OVERRIDE;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
};

#endif // APIDEVICESWRAPPER_HPP
