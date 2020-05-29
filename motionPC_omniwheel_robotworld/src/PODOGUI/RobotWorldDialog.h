#ifndef ROBOTWORLDDIALOG_H
#define ROBOTWORLDDIALOG_H

#include <QDialog>
#include "BasicFiles/PODOALDialog.h"
#include "../../../share/Headers/Command.h"
#include "CommonHeader.h"

namespace Ui {
class RobotWorldDialog;
}
enum{
    NO_HAND = 0, HAND_RIGHT, HAND_LEFT
};

typedef struct _HAND_INFO_
{
    double  HANDpos[3];
    double  HANDori[4];
    double  HANDelb;
}info_hand;

typedef QVector<info_hand> infos;

class RobotWorldDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RobotWorldDialog(QWidget *parent = 0);
    ~RobotWorldDialog();

private slots:
    void on_PB_WALK_READY_clicked();

    void on_PB_GO_TO_DES_clicked();

    void on_PB_STOP_clicked();

    void on_PB_SAVE_clicked();

    void on_PB_SAVE_START_clicked();

    void on_PB_GO_TO_DES_NEW_clicked();

    void on_PB_RHAND_GO_clicked();

    void on_PB_HAND_GRASP_clicked();

    void on_PB_HAND_STOP_clicked();

    void on_PB_HAND_OPEN_clicked();

    void on_PB_GRASP_clicked();

    void on_PB_ESTOP_clicked();

    void on_PB_PUT_clicked();

    void on_PB_WHEEL_MOVE_FRONT_clicked();

    void on_PB_WHEEL_MOVE_BACK_clicked();

    void UpdateData();
    void on_PB_RESET_ROBOTPOS_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::RobotWorldDialog *ui;
    int AlNumWalkReady;
    int AlNumRobotWorld;
};

#endif // ROBOTWORLDDIALOG_H
