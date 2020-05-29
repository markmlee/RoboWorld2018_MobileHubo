#ifndef PANEL_MC_H
#define PANEL_MC_H

#include <QDialog>
#include <QTimer>

#include "RBMotorController.h"

namespace Ui {
class Panel_MC;
}

enum{
    OPMODE_DUTY = 0,
    OPMODE_VEL,
    OPMODE_POS
};

class Panel_MC : public QDialog
{
    Q_OBJECT

public:
    explicit Panel_MC(QWidget *parent, RBMotorController *mc);
    ~Panel_MC();

    RBMotorController   *dev_info;
    int     CurChannel;

    int     OpMode;
    int     DutyValue;
    int     VelValue;
    int     PosValue;

    void    NewAccess();

private slots:
    void Display();

    void on_RB_DUTY_clicked(bool checked);
    void on_RB_VEL_clicked(bool checked);
    void on_RB_POS_clicked(bool checked);

    void on_CB_CHANNEL_currentIndexChanged(int index);

    void on_BTN_FET_ON_clicked();
    void on_BTN_FET_OFF_clicked();
    void on_BTN_SRV_ON_clicked();
    void on_BTN_SRV_OFF_clicked();
    void on_BTN_HOME_clicked();
    void on_BTN_ENC_ZERO_clicked();

    void on_BTN_PLUS_clicked();

    void on_BTN_MINUS_clicked();

    void on_BTN_PARAMETER_SCAN_clicked();

    void on_BTN_SET_GAIN_clicked();

    void on_BTN_SET_DEADZONE_clicked();

    void on_BTN_SET_ENC_RES_clicked();

    void on_BTN_SET_HOME_INFO_clicked();

    void on_BTN_SET_HOME_INFO_2_clicked();

    void on_BTN_SET_LOWER_LIMIT_clicked();

    void on_BTN_SET_UPPER_LIMIT_clicked();

private:
    Ui::Panel_MC *ui;
    QTimer  *displayTimer;


    void    RefreshBoardInfo();
    void    RefreshOpValue();
};

#endif // PANEL_MC_H
