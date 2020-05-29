#ifndef CARTASKDIALOG_H
#define CARTASKDIALOG_H

#include <QDialog>

#include "CommonHeader.h"

namespace Ui {
class CarTaskDialog;
}

class CarTaskDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CarTaskDialog(QWidget *parent = 0, LANDialog *_lanData = 0);
    ~CarTaskDialog();

private slots:
    void DisplayUpdate();

    void on_BTN_CAR_GRAVITY_COMP_START_clicked();

    void on_BTN_CAR_PWM_clicked();

    void on_BTN_CAR_SAVE_clicked();

    void on_BTN_CAR_POS_MOVE_clicked();

    void on_BTN_CAR_POS_SAVE_clicked();

    void on_BTN_CAR_ZERO_GAIN_clicked();

    void on_BTN_CAR_GO_POSITION_MODE_clicked();

    void on_BTN_CAR_FORCE_CONTROL_START_clicked();

    void on_BTN_CAR_HYBRID_CONTROL_START_clicked();

    void on_BTN_CAR_HYBRID_REFRESH_clicked();

    void on_BTN_CAR_PRINT_CUR_REF_clicked();

    void on_BTN_CAR_TURNING_clicked();

    void on_BRN_CAR_POS5_clicked();

    void on_BTN_CAR_TURNING_POS_clicked();

    void on_BTN_CAR_RELEASE_clicked();

    void on_BTN_CAR_E_STOP_clicked();

    void on_BTN_CAR_PRINT_CUR_POS_clicked();

    void on_BTN_CAR_PRINT_WBIK_INFO_clicked();

    void on_BTN_CAR_SCANING_VIRTUAL_clicked();

    void on_BTN_CAR_GRAB_RH_clicked();

    void on_BTN_CAR_RH_GRAB_clicked();

    void on_BTN_CAR_RH_RELEASE_clicked();

    void on_BTN_CAR_LH_GRAB_clicked();

    void on_BTN_CAR_LH_RELEASE_clicked();

    void on_BTN_CAR_GO_DESCENDINNG_POS_clicked();

    void on_BTN_CAR_DESCENDING_STEP3_2_clicked();

    void on_BTN_CAR_MANUAL_HANDLE_ACCEL_clicked();

    void on_BTN_CAR_MANUAL_HANDLE_ACCEL_STOP_clicked();

    void on_BTN_CAR_LEFT_GAINOVER_clicked();

    void on_BTN_CAR_RIGHT_GAINOVER_clicked();

    void on_BTN_CAR_LEFT_GAINOVER_OFF_clicked();

    void on_BTN_CAR_RIGHT_GAINOVER_OFF_clicked();

    void on_BRN_CAR_POS6_clicked();

    void on_BTN_CAR_GRAVITY_COMP_START_LEFT_clicked();

    void on_BTN_CAR_PRINT_WB_POS_clicked();

    void on_BTN_POSITION_CTRL_REFRESH_clicked();

    void on_BTN_POSITION_CTRL_START_clicked();

    void on_BTN_CAR_STATIC_WALKING_clicked();

    void on_BTN_CAR_STATIC_WALKING_2_clicked();

    void on_BTN_CAR_HOME_POS_clicked();

    void on_BRN_CAR_RH_GRAB_POS_clicked();

    void on_BRN_CAR_HOME_POS_clicked();

    void on_BTN_CAR_JOINT_FORCE_R_clicked();

    void on_BTN_CAR_JOINT_FORCE_L_clicked();

    void on_BTN_CAR_JOINT_FORCE_IDLE_clicked();

    void on_BTN_CAR_STABILIZING_LH_clicked();

    void on_BTN_CAR_JLWFT_NULL_clicked();

    void on_BTN_CAR_RWFT_NULL_clicked();

    void on_BTN_CAR_RH_RE_GRAB_POS_clicked();

    void on_BTN_CAR_TEST_POS_clicked();

    void on_BTN_CAR_FORCE_CONTROL_DATA_SAVE_clicked();

private:
    LANDialog			*lanData;
    Ui::CarTaskDialog   *ui;
    QTimer				*displayTimer;

    int                 AlnumCarTask;
    int                 AlnumManual;
};

#endif // CARTASKDIALOG_H
