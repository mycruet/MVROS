#ifndef TESTVIEW_H
#define TESTVIEW_H

#include <QMainWindow>
#include <QPaintEvent>
namespace Ui {
class testView;
}
class QNode;
class testView : public QMainWindow
{
    Q_OBJECT

public:

    explicit testView(int argc, char **argv, QWidget *parent = 0);
    ~testView();
public Q_SLOTS:
    void on_start_clicked();
    void on_chessBoard_clicked();
    void on_Calibrate_clicked();
    void on_HoughCircles_clicked();
    void on_stop_clicked();
    void imageShow(QImage q);
    void canCalibrate();

protected:
void paintEvent(QPaintEvent *);
private:
    Ui::testView *ui;
    QNode *qnode;
    QPixmap map;
    QRect mapLocation;
};

#endif // TESTVIEW_H
