#ifndef WALKINGDIALOG_H
#define WALKINGDIALOG_H

#include <QDialog>

#include "CommonHeader.h"

typedef struct DSPTask_t{
    double Left[3];
    double Right[3];
    double LYaw;
    double RYaw;
    double HTime;
    double COMz[2];

    //---pitch roll
    double LPitch;
    double RPitch;
    double LRoll;
    double RRoll;
}DSPTask;
typedef QVector<DSPTask>	DSPTasks;

const int FOOT_LEFT = 0;
const int FOOT_RIGHT = 1;

typedef enum{
    DIR_FORWARD = 0,
    DIR_BACKWARD,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_CW,
    DIR_CCW
}FOOT_DIRECTION;



namespace Ui {
class WalkingDialog;
}

class WalkingDialog : public QDialog
{
	Q_OBJECT

public:
    explicit WalkingDialog(QWidget *parent = 0);
	~WalkingDialog();

private:
	Ui::WalkingDialog	*ui;
	QTimer				*displayTimer;

	int					alNumFreeWalking;
	int					alNumWalkReady;


	QFont			tableFont;
	DSPTasks		DSPScheduler;

	void	SetControl(int num);
	void	SetTable(int row, DSPTask task);
	void	InsertTable(int row, DSPTask task);
	void	DeleteTable(int row);
	void	ChangeMaxSpinBox();
	void	Refresh();

	int		FootSelector(int direction = DIR_FORWARD);

private slots:
    void DisplayUpdate();
    void on_BTN_HOME_POS_clicked();
    void on_BTN_WALK_READY_POS_clicked();
    void on_TW_DSP_SCHEDULER_cellClicked(int row, int column);
    void on_SPIN_INDEX_valueChanged(int arg1);
    void on_BTN_CLEAR_ALL_clicked();
    void on_BTN_ADD_clicked();
    void on_BTN_DELETE_clicked();
    void on_BTN_APPLY_clicked();
    void on_BTN_FW_SAVE_clicked();
    void on_BTN_FW_LOAD_clicked();
    void on_BTN_FW_SAVE_PLAY_clicked();
    void on_BTN_FW_PLAY_SAVED_clicked();
    void on_BTN_INT_FORWARD_clicked();
    void on_BTN_INT_BACKWARD_clicked();
    void on_BTN_DATA_SAVE_clicked();
    void on_BTN_DATA_SAVE_START_clicked();
    void on_BTN_INT_RIGHT_clicked();
    void on_BTN_INT_LEFT_clicked();
    void on_BTN_INT_CW_clicked();
    void on_BTN_INT_CCW_clicked();
    void on_BTN_Control_On_clicked();
    void on_BTN_Control_Off_clicked();
    void on_BTN_FW_GO_clicked();
    void on_BTN_FILL_UP_STAIR_clicked();
    void on_BTN_FILL_1ST_DSP_clicked();
//    void on_BTN_FILL_DSP3_LAST();
    void on_BTN_FILL_DN_STAIR_clicked();
    void on_BTN_FILL_UP_STAIR_2_clicked();
    void on_BTN_FILL_DN_STAIR_2_clicked();
//    void fill_blanks(double rx[6],double ry[6],double rz[6],double lx[6],double ly[6],double lz[6],double com_z[6],double ryaw[6],double lyaw[6]);
    void fill_blanks(double rx[6],double ry[6],double rz[6],double lx[6],double ly[6],double lz[6],double com_z[6],double ryaw[6],double lyaw[6],double rroll[6],double lroll[6],double rpitch[6],double lpitch[6]);

    void on_BTN_FILL_UP_STAIR_3_clicked();
    void on_BTN_FILL_DN_STAIR_4_clicked();
    void on_BTN_FILL_UP_STAIR_4_clicked();
    void on_BTN_FILL_DN_STAIR_3_clicked();
    void on_BTN_FILL_1ST_DSP_2_clicked();
    void on_BTN_FILL_UP_TERRAIN_RF_1_clicked();
    void on_BTN_FILL_UP_TERRAIN_LF_1_clicked();
    void on_BTN_FILL_UP_TERRAIN_RF_2_clicked();
    void on_BTN_FILL_UP_TERRAIN_LF_2_clicked();
    void on_BTN_FILL_UP_TERRAIN_RF_3_clicked();
    void on_BTN_FILL_UP_TERRAIN_LF_3_clicked();
    void on_BTN_FILL_UP_TERRAIN_RF_4_clicked();
    void on_BTN_FILL_UP_TERRAIN_LF_4_clicked();
    void on_BTN_FILL_UP_TERRAIN_RF_5_clicked();
    void on_BTN_FILL_UP_TERRAIN_LF_5_clicked();
    void on_BTN_ENABLE_clicked();
    void on_BTN_FILL_UP_FROM_VISION_clicked();
    void on_BTN_FILL_UP_FROM_VISION_2_clicked();
    void on_BTN_INFINITY_clicked();
    void on_BTN_INFINITY_STOP_clicked();
    void on_BTN_STEP_TO_BE_WALK_READY_POS_clicked();
    void on_BTN_READY_clicked();
    void on_BTN_VISION_DIRECT_CONNECT_clicked();
    void on_BTN_VISION_DIRECT_REQUEST_clicked();
    void on_BTN_MOTION_CHECK_clicked();
    void on_BTN_FILL_UP_FROM_VISION_3_clicked();
    void on_BTN_FILL_UP_FROM_VISION_4_clicked();
    void on_BTN_FILL_1ST_DSP_3_clicked();
    void on_BTN_RF_2nd_clicked();
    void on_BTN_LF_2nd_clicked();
    void on_BTN_SET_FOOT_PRINT_clicked();
    void on_BTN_FILL_RF_clicked();
    void on_BTN_GOAL_GO_clicked();
    void on_BTN_INT_FORWARD_2_clicked();
    void on_BTN_INT_RIGHT_2_clicked();
    void on_BTN_INT_BACKWARD_2_clicked();
    void on_BTN_INT_LEFT_2_clicked();
    void on_BTN_INT_CW_2_clicked();
    void on_BTN_INT_CCW_2_clicked();
    void on_BTN_WALK_READY_POS_WITH_M180_clicked();
    void on_BTN_INITIALIZE_1step_clicked();
    void on_BTN_INT_INITIALIZE_clicked();
    void on_BTN_DATA_SAVE_2_clicked();
    void on_BTN_WALK_READY_POS_3_clicked();
    void on_BTN_Foot_up_clicked();
    void on_BTN_Knee_Down_clicked();
    void on_BTN_Foot_Down_clicked();
    void on_BTN_BOW_clicked();
    void on_BTN_WALK_READY_POS_4_clicked();
    void on_BTN_WALK_READY_TEST_clicked();
    void on_BTN_WALK_READY_TEST_2_clicked();
    void on_BTN_STOP_WALKING_clicked();
};

#endif // WALKINGDIALOG_H
