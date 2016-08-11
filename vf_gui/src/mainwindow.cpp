#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace ros;
using namespace mechanism_manager;

MainWindow::MainWindow(NodeHandle& nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    startTimer(1000);

    // Create model
    NamesModel_ = new QStringListModel(this);

    //NamesList_ << "Clair de Lune" << "Reverie" << "Prelude";

    // Populate our model
    NamesModel_->setStringList(NamesList_);

    ui->listView->setModel(NamesModel_);

    // Add additional feature so that
    // we can manually modify the data in ListView
    // It may be triggered by hitting any key or double-click etc.
    ui->listView->setEditTriggers(QAbstractItemView::AnyKeyPressed |
                                  QAbstractItemView::DoubleClicked);


    sc_ = nh.serviceClient<MechanismManagerServices>("/mechanism_manager/mechanism_manager_interaction",true); // Permanent connection
}


void MainWindow::timerEvent(QTimerEvent *event)
{

Q_UNUSED(event);

if (sc_.exists())
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
    MechanismManagerServices srv;
    std::string command = "delete";
    srv.request.request_command = command;
    srv.request.selected_guide = 0;
    if(!sc_.call(srv))
    {
      //ROS_INFO("OK, sent. Here is the answer:");
      //ROS_INFO(" - Response string: '%s'", srv.response.response_string.c_str());
    }

}

void MainWindow::on_insertButton_clicked()
{
    // Add button clicked
    // Adding at the end

    // Get the position
    int row = NamesModel_->rowCount();

    // Enable add one or more rows
    NamesModel_->insertRows(row,1);

    // Get the row for Edit mode
    QModelIndex index = NamesModel_->index(row);

    // Enable item selection and put it edit mode
    ui->listView->setCurrentIndex(index);
    ui->listView->edit(index);
    //mm_->Insert();
}
