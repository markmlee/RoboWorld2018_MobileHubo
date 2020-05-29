#include "cartaskdialog.h"
#include "ui_cartaskdialog.h"

#include "BasicFiles/PODOALDialog.h"

enum CarTask_ALCOMMAND
{    
    CarTask_AL_NO_ACT = 100,
    CarTask_AL_UPPER_TASK_POSE,
    CarTask_AL_CAR_TASK_POSE,
    CarTask_AL_HAND,
    CarTask_AL_GAIN,
    CarTask_AL_CT_GAIN_TUNING,
    CarTask_AL_CT_GO_POS,
    CarTask_AL_SAVE_ENC,
    CarTask_AL_CT_START,
    CarTask_AL_CT_GRAVITY_COMP,
    CarTask_AL_CT_ZERO_GAIN,
    CarTask_AL_CT_GO_POSITION_MODE,
    CarTask_AL_CT_FORCE_CONTROL,
    CarTask_AL_CT_HYBRID_CONTROL,
    CarTask_AL_CT_REFRESH,
    CarTask_AL_CAR_TURNING,
    CarTask_AL_CAR_TURNING_POSE,
    CarTask_AL_CAR_RELEASE,
    CarTask_AL_GRAB_RH,
    CarTask_AL_GRAB_RH_VISION_DATA,
    CarTask_AL_HAND_GRAB,
    CarTask_AL_GO_DESCENDING_POS,
    CarTask_AL_START_DESCENDING,
    CarTask_AL_PRINT_STATE,
    CarTask_AL_CT_REFRESH_POSITION,
    CarTask_AL_CT_POSITION_CONTROL,
    CarTask_AL_STATIC_WALK,
    CarTask_AL_STABILIZING,
    CarTask_AL_STABILIZING_LEFT,
    CarTask_AL_WALKREADY_POS,
    CarTask_AL_RH_GRAB_POS,
    CarTask_AL_HOME_POS,
    CarTask_AL_JOINT_FORCE_CTRL_TEST,
    CarTask_AL_WRIST_FT_NULLING,
    CarTask_AL_RH_RE_GRAB_POS,
    CarTask_Al_AFTER_STATIC_WALK_POS,
    EgressTask_AL_EGRESS_POS,
    CarTask_AL_SEND_ROI,
    CarTask_AL_CHECK_RH_GRAB_POS_VISION,
    CarTask_AL_MODIFY_VISION_DATA,
    CarTask_AL_SAVE_DATA,
};
enum ManualMove_ALCOMMAND
{
    ManualMove_AL_NO_ACT = 100,
    ManualMove_AL_UPPER_TASK_POSE,
    ManualMove_AL_MANUAL_MODE_START,
    ManualMove_AL_MANUAL_MODE_STOP,
    ManualMove_AL_MANUAL_FOOT_MODE_START,
    ManualMove_AL_MANUAL_FOOT_MODE_STOP,
    ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START,
    ManualMove_AL_MANUAL_BOTH_FOOT_MODE_STOP,
    ManualMove_AL_MANUAL_BOTH_HAND_MODE_START,
    ManualMove_AL_MANUAL_BOTH_HAND_MODE_STOP,
    ManualMove_AL_HAND,
    ManualMove_AL_GAIN,
    ManualMove_AL_E_STOP,
    ManualMove_AL_DRIVE_MODE
};

CarTaskDialog::CarTaskDialog(QWidget *parent, LANDialog *_lanData) :
    QDialog(parent),lanData(_lanData),
    ui(new Ui::CarTaskDialog)
{
    ui->setupUi(this);

    AlnumCarTask = PODOALDialog::GetALNumFromFileName("CarTask");
    AlnumManual = PODOALDialog::GetALNumFromFileName("ManualMove");

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50
}

void CarTaskDialog::DisplayUpdate()
{
    ;
}


CarTaskDialog::~CarTaskDialog()
{
    delete ui;
}

void CarTaskDialog::on_BTN_CAR_GRAVITY_COMP_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GRAVITY_COMP;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // Right Arm
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}
void CarTaskDialog::on_BTN_CAR_GRAVITY_COMP_START_LEFT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GRAVITY_COMP;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Left Arm
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_PWM_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GAIN_TUNING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // current value
    cmd.COMMAND_DATA.USER_PARA_INT[1] = ui->LE_DUTY->text().toInt();
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GAIN_TUNING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2; //save
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_POS_MOVE_clicked()
{
    int hand = 0;
    if(ui->RB_LH_2 ->isChecked())
        hand = -1;
    else if(ui->RB_LH_2->isChecked())
        hand = 1;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GO_POS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_POS_NO->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = hand;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_POS_SAVE_clicked()
{
    QString str;
    int hand = 0;

    if(ui->RB_RH_2->isChecked())
        hand = -1;
    else if(ui->RB_LH_2->isChecked())
        hand = 1;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_SAVE_ENC;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_POS_NO->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_INT[1] = hand;
    cmd.COMMAND_TARGET = AlnumCarTask;

    str.setNum((cmd.COMMAND_DATA.USER_PARA_INT[0]+1)%50);
    ui->LE_POS_NO->setText(str);
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_ZERO_GAIN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_ZERO_GAIN;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_GO_POSITION_MODE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_GO_POSITION_MODE;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_FORCE_CONTROL_START_clicked()
{
    USER_COMMAND cmd;
    // RH FX
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->FORCE_CTRL_RH_FX->text().toDouble();
    // RH FY
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->FORCE_CTRL_RH_FY->text().toDouble();
    // RH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->FORCE_CTRL_RH_FZ->text().toDouble();

    // LH FX
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->FORCE_CTRL_LH_FX->text().toDouble();
    // LH FY
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->FORCE_CTRL_LH_FY->text().toDouble();
    // LH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->FORCE_CTRL_LH_FZ->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_FORCE_CONTROL;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_HYBRID_CONTROL_START_clicked()
{
    USER_COMMAND cmd;
    // RH X
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->HYBRID_CTRL_RH_X->text().toDouble();
    // RH Y
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->HYBRID_CTRL_RH_Y->text().toDouble();
    // RH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->HYBRID_CTRL_RH_FZ->text().toDouble();
    // RElb
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->HYBRID_CTRL_RELB->text().toDouble();

    // LH X
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->HYBRID_CTRL_LH_X->text().toDouble();
    // LH Y
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->HYBRID_CTRL_LH_Y->text().toDouble();
    // LH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->HYBRID_CTRL_LH_FZ->text().toDouble();
    // LElb
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->HYBRID_CTRL_LELB->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_HYBRID_CONTROL;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_HYBRID_REFRESH_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_REFRESH;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
    usleep(10*1000);
    QString str;
    ui->HYBRID_CTRL_RH_X->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.pRH[0]));
    ui->HYBRID_CTRL_RH_Y->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.pRH[1]));
    ui->HYBRID_CTRL_LH_X->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.pLH[0]));
    ui->HYBRID_CTRL_LH_Y->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.pLH[1]));
    ui->HYBRID_CTRL_RELB->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.Relb));
    ui->HYBRID_CTRL_LELB->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.Lelb));
//    std::cout<<"refresh Gui"<<std::endl;

}

void CarTaskDialog::on_BTN_CAR_PRINT_CUR_REF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_PRINT_STATE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // Current joint Referrence
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_PRINT_WBIK_INFO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_PRINT_STATE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Current joint Referrence
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_PRINT_CUR_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_PRINT_STATE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2; // Current Joint pos
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_PRINT_WB_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_PRINT_STATE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3; // Current WB pos //UB
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_TURNING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CAR_TURNING;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BRN_CAR_POS5_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CAR_TASK_POSE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // pos 4
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_TURNING_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CAR_TURNING_POSE;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RELEASE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CAR_RELEASE;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}


void CarTaskDialog::on_BTN_CAR_E_STOP_clicked() // ---> to ManualMove AL
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_E_STOP;
    cmd.COMMAND_TARGET = AlnumManual;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_SCANING_VIRTUAL_clicked()
{
    QString str;
    ui->EDIT_RH_GRAB_X->setText(str.sprintf("%.2f",0.227));
    ui->EDIT_RH_GRAB_Y->setText(str.sprintf("%.2f",-0.26));
    ui->EDIT_RH_GRAB_Z->setText(str.sprintf("%.2f",0.85));
}

void CarTaskDialog::on_BTN_CAR_GRAB_RH_clicked()
{
//    if(ui->EDIT_RH_GRAB_X->text().toDouble() == 0.0){
//        cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_NO_ACT;
//        cmd.COMMAND_TARGET = AlnumCarTask;
//        pLAN->SendCommand(cmd);
//    }
//    else{
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->EDIT_RH_GRAB_X->text().toDouble();
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->EDIT_RH_GRAB_Y->text().toDouble();
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->EDIT_RH_GRAB_Z->text().toDouble();
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GRAB_RH;
        cmd.COMMAND_TARGET = AlnumCarTask;
        pLAN->SendCommand(cmd);
//    }


}

void CarTaskDialog::on_BTN_CAR_RH_GRAB_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_HAND_GRAB;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // RH grab
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RH_RELEASE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_HAND_GRAB;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // RH release
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_LH_GRAB_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_HAND_GRAB;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2; // LH grab
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_LH_RELEASE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_HAND_GRAB;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3; // LH release
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_GO_DESCENDINNG_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GO_DESCENDING_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_DESCENDING_STEP3_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_START_DESCENDING;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_MANUAL_HANDLE_ACCEL_clicked()// ---> to ManualMove AL
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_DRIVE_MODE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; // start!
    cmd.COMMAND_TARGET = AlnumManual;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_MANUAL_HANDLE_ACCEL_STOP_clicked()// ---> to ManualMove AL
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_DRIVE_MODE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; // stop
    cmd.COMMAND_TARGET = AlnumManual;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_LEFT_GAINOVER_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GAIN;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // LEFT
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0; // LOW GAIN
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RIGHT_GAINOVER_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GAIN;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // RIGHT
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0; // LOW GAIN
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_LEFT_GAINOVER_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GAIN;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // LEFT
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // HIGH GAIN
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RIGHT_GAINOVER_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_GAIN;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // RIGHT
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // HIGH GAIN
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BRN_CAR_POS6_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CAR_TASK_POSE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // Pos3
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}



void CarTaskDialog::on_BTN_POSITION_CTRL_REFRESH_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_REFRESH_POSITION;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
    usleep(10*1000);
    QString str;
    ui->POSITION_CTRL_RH_X->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pRH[0]));
    ui->POSITION_CTRL_RH_Y->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pRH[1]));
    ui->POSITION_CTRL_RH_Z->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pRH[2]));
    ui->POSITION_CTRL_RH_ELB->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.Relb));
    ui->POSITION_CTRL_RH_ROLL->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qRH[0]*R2D)); //roll
    ui->POSITION_CTRL_RH_PITCH->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qRH[1]*R2D));//pitch
    ui->POSITION_CTRL_RH_YAW->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qRH[2]*R2D));  //yaw

    ui->POSITION_CTRL_LH_X->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pLH[0]));
    ui->POSITION_CTRL_LH_Y->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pLH[1]));
    ui->POSITION_CTRL_LH_Z->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.pLH[2]));
    ui->POSITION_CTRL_LH_ELB->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.Lelb));
    ui->POSITION_CTRL_LH_ROLL->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qLH[0]*R2D)); //roll
    ui->POSITION_CTRL_LH_PITCH->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qLH[1]*R2D));//pitch
    ui->POSITION_CTRL_LH_YAW->setText(str.sprintf("%.2f",PODO_DATA.UserM2G.qLH[2]*R2D));  //yaw
}

void CarTaskDialog::on_BTN_POSITION_CTRL_START_clicked()
{
    USER_COMMAND cmd;
    // RH X
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->POSITION_CTRL_RH_X->text().toDouble();
    // RH Y
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->POSITION_CTRL_RH_Y->text().toDouble();
    // RH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->POSITION_CTRL_RH_Z->text().toDouble();
    // RElb
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->POSITION_CTRL_RH_ELB->text().toDouble();
    // rpy
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->POSITION_CTRL_RH_ROLL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->POSITION_CTRL_RH_PITCH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->POSITION_CTRL_RH_YAW->text().toDouble();

    // LH X
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->POSITION_CTRL_LH_X->text().toDouble();
    // LH Y
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->POSITION_CTRL_LH_Y->text().toDouble();
    // LH FZ
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->POSITION_CTRL_LH_Z->text().toDouble();
    // LElb
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[10] = ui->POSITION_CTRL_LH_ELB->text().toDouble();
    // rpy
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[11] = ui->POSITION_CTRL_LH_ROLL->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[12] = ui->POSITION_CTRL_LH_PITCH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[13] = ui->POSITION_CTRL_LH_YAW->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_CT_POSITION_CONTROL;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}


void CarTaskDialog::on_BTN_CAR_STATIC_WALKING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_STATIC_WALK;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_STATIC_WALKING_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_STABILIZING;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_HOME_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_WALKREADY_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BRN_CAR_RH_GRAB_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_RH_GRAB_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BRN_CAR_HOME_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_HOME_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_JOINT_FORCE_R_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_JOINT_FORCE_CTRL_TEST;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_JOINT_FORCE_L_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_JOINT_FORCE_CTRL_TEST;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_JOINT_FORCE_IDLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_JOINT_FORCE_CTRL_TEST;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_STABILIZING_LH_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_STABILIZING_LEFT;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_JLWFT_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_WRIST_FT_NULLING;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RWFT_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_WRIST_FT_NULLING;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_RH_RE_GRAB_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_RH_RE_GRAB_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_TEST_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_Al_AFTER_STATIC_WALK_POS;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}

void CarTaskDialog::on_BTN_CAR_FORCE_CONTROL_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = CarTask_AL_SAVE_DATA;
    cmd.COMMAND_TARGET = AlnumCarTask;
    pLAN->SendCommand(cmd);
}
