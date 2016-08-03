#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);



    startTimer(1000);
}


void MainWindow::timerEvent(QTimerEvent *event)
{

Q_UNUSED(event);
/*if (mm_->isValid())
    ui->label->setText("connected");
else
    ui->label->setText("disconnected");

    */
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

    //mm_->Delete();

}

void MainWindow::on_insertButton_clicked()
{
    //mm_->Insert();
}
