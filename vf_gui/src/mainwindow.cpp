#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    ac_("mechanism_manager", true)
{
    ui->setupUi(this);

    startTimer(1000);

    ac_.waitForServer(ros::Duration(2.0));
}


void MainWindow::timerEvent(QTimerEvent *event)
{

Q_UNUSED(event);

if (ac_.isServerConnected())
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
    mechanism_manager::MechanismManagerGoal goal;

    goal.delete_guide_idx = 0;
    ac_.sendGoal(goal);
}

void MainWindow::on_insertButton_clicked()
{
    //mm_->Insert();
}
