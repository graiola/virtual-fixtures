/**
 * @file   guides_model.h
 * @brief  Model associated to the list of guides.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    bool setMode(const QString& mode);
    Qt::ItemFlags flags(const QModelIndex & index) const ;
    void updateList();
    bool isServerConnected();
    bool setMergeTh(int merge_th);
    bool getMergeTh(int& merge_th);

protected:
    QStringList names_list_;
    ros::ServiceClient sc_;
    QTimer* refresh_timer_;
    bool new_guide_;

protected Q_SLOTS:
    void refreshTimerHit();

};

#endif // GUIDES_MODEL_H
