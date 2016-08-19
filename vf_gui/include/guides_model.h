#ifndef GUIDES_MODEL_H
#define GUIDES_MODEL_H

/// ROS
#include <ros/ros.h>
#include <mechanism_manager/MechanismManagerServices.h>

#include <QAbstractListModel>
#include <QTimer>

class GuidesModel : public QAbstractListModel
{
    Q_OBJECT
public:
    GuidesModel(ros::NodeHandle& nh, QObject *parent);
    int rowCount(const QModelIndex &parent = QModelIndex()) const ;
    //int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    bool removeRow(int row, const QModelIndex &parent = QModelIndex());
    bool insertRow(int row, const QModelIndex &parent = QModelIndex());
    bool saveRow(int row, const QModelIndex &parent = QModelIndex());
    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
    Qt::ItemFlags flags(const QModelIndex & index) const ;
    void updateList();
    bool isServerConnected();

protected:
    QStringList names_list_;
    ros::ServiceClient sc_;
    QTimer* refresh_timer_;
    bool new_guide_;



protected Q_SLOTS:
    void refreshTimerHit();


};

#endif // GUIDES_MODEL_H
