#include "../include/qtest/testview.h"
#include "../include/qtest/qnode.h"
#include "ui_testview.h"
#include <QPainter>
#include <QFile>  
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
        // ui->lineEdit->setText("00000000");
         ui->HoughCircles->setDisabled(false);


         QFile file("/home/gxf/test_node/src/test_nodelet/tmp/calibrator.txt");  
         if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
            qDebug("Can't open the file!");  
         } 
         int i = 0; 
         while(!file.atEnd()) {  
            QByteArray line = file.readLine();  
            QString str(line);  
            qDebug(line.data()); 
            if(i==14){
               ui->lineEdit->setText(str);
            }else if(i==19){
               ui->lineEdit_2->setText(str);
            }else if(i==22){
               ui->lineEdit_3->setText(str);
            }
            i++;
       //     qDebug()<<str<<endl;  //调试时，在console中输出   
        }  
     }
}

void testView::on_HoughCircles_clicked()
{
    qDebug(">>>>>>>>>>on_HoughCircles_clicked");
     ui->HoughCircles->setDisabled(true);
    qnode->houghCircle();
    connect(qnode, SIGNAL(hasHoughed()), this, SLOT(hasHoughed()), Qt::BlockingQueuedConnection);

}
void testView::on_stop_clicked()
{

}

void testView::imageShow(QImage q)
{
    qDebug(">>>>row:%d---col:%d", q.height(), q.width());

    mapLocation = ui->frame->geometry();
    map = QPixmap::fromImage(q);
   ui->frame->setMinimumSize(map.width(), map.height());
   //this->adjustSize();
   this->update();
   
}
void  testView::paintEvent(QPaintEvent *e)
{

    qDebug("draw............................................................");
    QPainter p(this);
    p.drawPixmap(QPoint(mapLocation.x(), mapLocation.y()), map) ;

}
void testView::canCalibrate()
{
     qDebug("canCalibrate");  
     ui->Calibrate->setDisabled(false);

}
void testView::hasHoughed()
{ 
     qDebug("hasHoughed");  

      QFile file("/home/gxf/test_node/src/test_nodelet/tmp/mycircle.txt");  
         if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
            qDebug("Can't open the file!");  
         }  
         int i=0;
         while(!file.atEnd()) {  
            QByteArray line = file.readLine();  
            QString str(line);
            qDebug(line.data());  
            if(i==0){
                ui->lineEdit_5->setText(str);
            }else{
                ui->lineEdit_4->setText(str);
            } 
            i++;
            
       //     qDebug()<<str<<endl;  //调试时，在console中输出   
        }  

}


