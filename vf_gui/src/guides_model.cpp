/**
 * @file   guides_model.cpp
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

#include "guides_model.h"

using namespace ros;
using namespace mechanism_manager;

GuidesModel::GuidesModel(NodeHandle& nh, QObject *parent)
    :QAbstractListModel(parent)
{

    sc_ = nh.serviceClient<MechanismManagerServices>("/mechanism_manager/mechanism_manager_interaction");

    //refresh_timer_ = new QTimer(this);
    //refresh_timer_->setInterval(15000); // Refresh the list every 15s
    //connect(refresh_timer_, SIGNAL(timeout()) , this, SLOT(refreshTimerHit()));
    //refresh_timer_->start();

    // True if the user tries to insert a new guide
    new_guide_ = false;

    updateList();
}

void GuidesModel::refreshTimerHit()
{   
    //updateList();
}

void GuidesModel::updateList()
{
    MechanismManagerServices srv;
    sc_.call(srv);
    QVector<QString> vec;
    QString tmp_qstring;

    for (unsigned int i = 0; i<srv.response.list_guides.size(); i++)
    {
        tmp_qstring = QString::fromStdString(srv.response.list_guides[i]);
        vec.append(tmp_qstring);
    }

    //names_list_.clear();

    beginInsertRows(QModelIndex(),0,srv.response.list_guides.size());
    names_list_ = QStringList::fromVector(vec);
    endInsertRows();
}

QVariant GuidesModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
       return names_list_[index.row()];
    }
    return QVariant();
}


int GuidesModel::rowCount(const QModelIndex & /*parent*/) const
{
   return names_list_.size();
}

bool GuidesModel::removeRow(int row, const QModelIndex & /*parent*/)
{
    beginRemoveRows(QModelIndex(),row,row);
    MechanismManagerServices srv;
    std::string command = "delete";
    srv.request.request_command = command;
    srv.request.selected_guide_idx = row;
    if(!sc_.call(srv))
    {
        //TODO Visualize the problem on the gui
        return false;
    }
    names_list_.removeAt(row);
    endRemoveRows();

    //updateList();

    return true;
}

bool GuidesModel::saveRow(int row, const QModelIndex & /*parent*/)
{
    MechanismManagerServices srv;
    std::string command = "save";
    srv.request.request_command = command;
    srv.request.selected_guide_idx = row;
    if(!sc_.call(srv))
    {
        // TODO Visualize the problem on the gui
        return false;
    }
    return true;
}

bool GuidesModel::insertRow(int row, const QModelIndex & /*parent*/)
{
    beginInsertRows(QModelIndex(),row,row);
    names_list_.insert(row,"");
    new_guide_ = true;
    endInsertRows();
    return true;
}

bool GuidesModel::setMode(const QString& mode)
{
    MechanismManagerServices srv;
    std::string command = "set_mode";
    srv.request.request_command = command;
    srv.request.selected_mode = mode.toStdString();
    if(!sc_.call(srv))
    {
        // TODO Visualize the problem on the gui
        return false;
    }
    return true;
}

bool GuidesModel::setMergeTh(int merge_th)
{
    MechanismManagerServices srv;
    std::string command = "set_merge_th";
    srv.request.request_command = command;
    srv.request.merge_th = merge_th;
    if(!sc_.call(srv))
    {
        // TODO Visualize the problem on the gui
        return false;
    }
    return true;
}

bool GuidesModel::getMergeTh(int& merge_th)
{
    MechanismManagerServices srv;
    std::string command = "get_merge_th";
    srv.request.request_command = command;
    if(!sc_.call(srv))
    {
        // TODO Visualize the problem on the gui
        return false;
    }
    merge_th = srv.response.merge_th;
    return true;

}

bool GuidesModel::isServerConnected()
{
    return sc_.exists();
}

bool GuidesModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
    if (role == Qt::EditRole)
    {
        MechanismManagerServices srv;
        std::string command;
        srv.request.selected_guide_name = value.toString().toStdString();
        if(new_guide_) // New guide
        {
            new_guide_ = false; // Reset
            command = "insert";
            srv.request.request_command = command;
            if(!sc_.call(srv))
            {
                // TODO Visualize the problem on the gui
                return false;
            }

        }
        else // Modify existing guide's name
        {
            command = "set_name";
            srv.request.request_command = command;
            srv.request.selected_guide_idx = index.row();
            if(!sc_.call(srv))
            {
                // TODO Visualize the problem on the gui
                return false;
            }
        }

        // Change the internal list
        names_list_[index.row()] = value.toString();
    }

    //updateList();

    return true;
}

Qt::ItemFlags GuidesModel::flags(const QModelIndex & /*index*/) const
{
    return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled ;
}
