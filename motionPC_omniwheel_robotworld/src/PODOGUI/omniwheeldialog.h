#ifndef OMNIWHEELDIALOG_H
#define OMNIWHEELDIALOG_H

#include <QDialog>

#include "CommonHeader.h"
namespace Ui {
class OmniWheelDialog;
}

class OmniWheelDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OmniWheelDialog(QWidget *parent = 0);
    ~OmniWheelDialog();

private slots:
    void DisplayUpdate();

    void on_OW_GOTO_DES_clicked();

    void on_OW_STOP_clicked();

    void on_OW_TRANSFORM_WALK_TO_WHEEL_clicked();

    void on_OW_SET_ZERO_clicked();

    void on_OW_NORMAL_WALKREADY_clicked();

    void on_OW_TRANSFORM_WHEEL_TO_WALK_clicked();

    //void on_OW_SEND_BOUNDARY_clicked();

    void on_OW_KNEE_GAIN_START_clicked();

    void on_OW_KNEE_GAIN_STOP_clicked();

    void on_SET_TO_REAL_clicked();

    void on_SET_TO_TEST_clicked();


    void on_BTN_ROS_MODE_clicked();

signals:
    void    SIG_OMNI_GET_VISION();
    void    SIG_OMNI_GOTO_POINT();
    void    SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST();

private:
    Ui::OmniWheelDialog *ui;
    QTimer				*displayTimer;

    int AlnumOmniWheel;
    int AlnumWalkReady;



//    int IMUnullFlag;
//    float IMUpitch;
//    int IMUcount;
//    float IMUsum;
};

#endif // OMNIWHEELDIALOG_H
