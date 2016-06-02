#include "../include/qtest/testview.h"
#include "../include/qtest/qnode.h"
#include "ui_testview.h"
#include <QPainter>
testView::testView(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent), ui(new Ui::testView), qnode(new QNode(argc, argv))
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_PaintOutsidePaintEvent);
    layout()->setSizeConstraint(QLayout::SetFixedSize);
}

testView::~testView()
{
    delete ui;
}

void testView::on_start_clicked()
{
    //ui->chessBoard->setDisabled(true);
    ui->start->setDisabled(true);
    ui->Calibrate->setDisabled(true);
    ui->HoughCircles->setDisabled(true);
    qnode->init();
    //connect(qnode, SIGNAL(imageShow(QImage )), this, SLOT(imageShow(QImage )), Qt::DirectConnection);
    connect(qnode, SIGNAL(imageShow(QImage )), this, SLOT(imageShow(QImage )), Qt::BlockingQueuedConnection);
}


void testView::on_chessBoard_clicked()
{
    qDebug(">>>>>>>>>>on_chessBoard_clicked");
    ui->chessBoard->setDisabled(true);
    connect(qnode, SIGNAL(canCalibrate()), this, SLOT(canCalibrate()), Qt::BlockingQueuedConnection);
    qnode->checkBoard();
}

void testView::on_Calibrate_clicked()
{
    qDebug(">>>>>>>>>>on_Calibrate_clicked");
     ui->Calibrate->setDisabled(true);
     if(qnode->calibrate())
     {
         //read A/D from database
         ui->lineEdit->setText("00000000");
         ui->HoughCircles->setDisabled(false);
     }
}

void testView::on_HoughCircles_clicked()
{
    qDebug(">>>>>>>>>>on_HoughCircles_clicked");
     ui->HoughCircles->setDisabled(true);
    qnode->houghCircle();



}
void testView::on_stop_clicked()
{

}

void testView::imageShow(QImage q)
{
    qDebug(">>>>row:%d---col:%d", q.height(), q.width());
    //QRect v;
    //v = ui->verticalLayout_2->geometry();
   // qDebug(">>>>x:%d, y:%d", v.width(), v.height());
    //ui->verticalLayout_2->setGeometry(QRect(10, 10, v.width(), v.height()));
    map = QPixmap::fromImage(q);
    this->update();
}
void  testView::paintEvent(QPaintEvent *e)
{

    qDebug("draw............................................................");
    QPainter p(this);
    p.drawPixmap(QPoint(0,0), map) ;

}
void testView::canCalibrate()
{

     ui->Calibrate->setDisabled(false);

}


