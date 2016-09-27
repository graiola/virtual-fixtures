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

    sub_ = new Subscriber(nh.subscribe("rosout", 1000, &MainWindow::loggerCallback, this));

    spinner_ptr_ = new AsyncSpinner(1); // Use one thread to keep the ros magic alive
    spinner_ptr_->start();

    // TextBox is read only
    ui->consoleText->setReadOnly(true);
    ui->consoleText->setTextInteractionFlags(0); //0 == Qt::TextInteractionFlag::NoTextInteraction

    //QColor color = QColorDialog::getColor(Qt::white,this); // in here your color pallete will open..

    QPalette p = ui->consoleText->palette(); // define pallete for textEdit..
    p.setColor(QPalette::Base, Qt::black); // set color "Red" for textedit base
    //p.setColor(QPalette::Text, Qt::white); // set text color which is selected from color pallete
    ui->consoleText->setPalette(p); // change textedit palette

    // Explanation of this hack:
    // Qt and Ros don't like each other, or better, their respective thread workers don't like each other.
    // You should not handle qt objects with ROS threads.
    // So we use the ros callback to trigger the qt callback :)
    QObject::connect(this, SIGNAL(requestUpdateConsole(const QString&,int)),
                     this, SLOT(updateConsole(const QString&,int)));

    // SlideBar
    slidebar_res_ = 100;
    ui->mergeSlider->setMinimum(0);
    ui->mergeSlider->setMaximum(slidebar_res_);

    //Refresh on start
    on_refreshButton_clicked(); // click a virtual button! :D
}

void MainWindow::loggerCallback(const rosgraph_msgs::Log::ConstPtr& msg)
{
    if(std::strcmp(msg->name.c_str(),"/mechanism_manager") == 0)
    {
         QString tmp_qstring;
         tmp_qstring = QString::fromStdString(msg->msg.data());
         emit requestUpdateConsole(tmp_qstring,msg->level);
    }
}

void MainWindow::updateConsole(const QString& data, int level)
{
    ui->consoleText->setTextBackgroundColor(Qt::black);

    switch(level)
    {
        case 4 :
            ui->consoleText->setTextColor(Qt::yellow);
            break;
        case 8 :
            ui->consoleText->setTextColor(Qt::red);
            break;
        default:
            ui->consoleText->setTextColor(Qt::white);
            break;
    }
    ui->consoleText->append(data);
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
    delete sub_;
    delete spinner_ptr_;
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
    /*guides_model_->updateList();
    int slider_value = 0;
    guides_model_->getMergeTh(slider_value);
    ui->mergeSlider->setValue(slider_value);
    QString mode;
    guides_model_->getMode(mode);
    if(mode == "SOFT")
        ui->softRadioButton->setChecked(true);
    else if(mode == "HARD")
         ui->hardRadioButton->setChecked(true);*/
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
    guides_model_->setMergeTh(static_cast<double>(position)/static_cast<double>(slidebar_res_));
}

void MainWindow::on_clearButton_clicked()
{
    ui->consoleText->clear();
}
