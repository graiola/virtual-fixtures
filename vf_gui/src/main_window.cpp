/**
 * @file   main_window.cpp
 * @brief  View associated to the list of guides.
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

#include "main_window.h"
#include "ui_main_window.h"

using namespace ros;
//using namespace mechanism_manager;

MainWindow::MainWindow(NodeHandle& nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    startTimer(1000);

    // Create model
    guides_model_ = new GuidesModel(nh,this);

    ui->listView->setModel(guides_model_);

    // Add additional feature so that
    // we can manually modify the data in ListView
    // It may be triggered by hitting any key or double-click etc.
    ui->listView->setEditTriggers(QAbstractItemView::AnyKeyPressed |
                                  QAbstractItemView::DoubleClicked );

    // SlideBar
    ui->mergeSlider->setMinimum(0);
    ui->mergeSlider->setMaximum(100);
}


void MainWindow::timerEvent(QTimerEvent *event)
{

Q_UNUSED(event);

if (guides_model_->isServerConnected())
    ui->label->setText("connected");
else
    ui->label->setText("disconnected");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_quitButton_clicked()
{
    qApp->quit();
}

void MainWindow::on_deleteButton_clicked()
{
    guides_model_->removeRow(ui->listView->currentIndex().row());
}

void MainWindow::on_insertButton_clicked()
{
    // Get the position
    int row = guides_model_->rowCount();

    // Enable add one or more rows
    guides_model_->insertRow(row);

    // Get the row for Edit mode
    QModelIndex index = guides_model_->index(row);

    // Enable item selection and put it edit mode
    ui->listView->setCurrentIndex(index);
    ui->listView->edit(index);
}

void MainWindow::on_refreshButton_clicked()
{
    guides_model_->updateList();
}

void MainWindow::on_saveButton_clicked()
{
    guides_model_->saveRow(ui->listView->currentIndex().row());
}

void MainWindow::on_softRadioButton_clicked()
{
    QString mode = "SOFT";
    guides_model_->setMode(mode);
}

void MainWindow::on_hardRadioButton_clicked()
{
    QString mode = "HARD";
    guides_model_->setMode(mode);
}

void MainWindow::on_mergeSlider_sliderMoved(int position)
{
    guides_model_->setMergeTh(position);
}
