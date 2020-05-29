#include "omniwheeldialog.h"
#include "ui_omniwheeldialog.h"
#include "CommonHeader.h"
//#include "BasicMath.h"
//#include "BasicMatrix.h"
#include <iostream>

#include "BasicFiles/PODOALDialog.h"

enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_GOTODES,
    OMNIWHEEL_AL_VELMODE,
    OMNIWHEEL_AL_CHANGEPOS,
    OMNIWHEEL_AL_CONTROL,
    OMNIWHEEL_AL_MANUAL,
    OMNIWHEEL_AL_RADIUS,
    OMNIWHEEL_AL_ROS
};

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD
};

float IMUnullFlag=0;
float IMUpitch=0.;
float IMUcount=0;
float IMUsum=0;

double pre_LU_point[3];
double pre_LD_point[3];
double pre_RD_point[3];
double pre_RU_point[3];

float next_LU_point[3];
float next_LD_point[3];
float next_RD_point[3];
float next_RU_point[3];

OmniWheelDialog *pOmni;

OmniWheelDialog::OmniWheelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OmniWheelDialog)
{
    ui->setupUi(this);
    pOmni = this;

    AlnumOmniWheel = PODOALDialog::GetALNumFromFileName("OmniWheel");
    AlnumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");


    connect(this, SIGNAL(SIG_OMNI_GOTO_POINT()), this, SLOT(on_OW_GOTO_DES_clicked()));
    //connect(this, SIGNAL(SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST()),this,SLOT(on_OW_SEND_BOUNDARY_clicked()));

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);
}

OmniWheelDialog::~OmniWheelDialog()
{
    delete ui;
}
void OmniWheelDialog::DisplayUpdate()
{

}
//----------------------------------------------------------------------------------------------------
// Moving Function
//----------------------------------------------------------------------------------------------------
void OmniWheelDialog::on_OW_GOTO_DES_clicked()
{
    // GOTO DESTINATION
    QString str_des_x = ui->OW_EDIT_X_POS->text();
    QString str_des_y = ui->OW_EDIT_Y_POS->text();
    QString str_des_a = ui->OW_EDIT_T_POS->text();
    QString str_des_v = ui->OW_EDIT_VEL->text();
    QString str_des_sec = ui->OW_EDIT_TIME->text();
    bool bSuccess1 = false;
    bool bSuccess2 = false;
    bool bSuccess3 = false;
    bool bSuccess4 = false;
    bool bSuccess5 = false;
    double des_x = 0.0;
    double des_y = 0.0;
    double des_a = 0.0;
    double des_v = 300;
    double des_sec = 0.0;
    des_x = str_des_x.toDouble(&bSuccess1);
    des_y = str_des_y.toDouble(&bSuccess2);
    des_a = str_des_a.toDouble(&bSuccess3);
    des_v = str_des_v.toDouble(&bSuccess4);
    des_sec = str_des_sec.toDouble(&bSuccess5);

    if((bSuccess1 == false)||(bSuccess2 == false)||(bSuccess3 == false)||(bSuccess4 == false)||(bSuccess5 == false)) return;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=des_x;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=des_y;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=des_a;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=des_v;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4]=des_sec;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_GOTODES;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);
}


void OmniWheelDialog::on_OW_STOP_clicked()
{
    // E Stop
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_VELMODE;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);
}

void OmniWheelDialog::on_OW_SET_ZERO_clicked()
{
    ui->OW_EDIT_X_POS->setText(QString().sprintf("%.3f", 0.));
    ui->OW_EDIT_Y_POS->setText(QString().sprintf("%.3f", 0.));
    ui->OW_EDIT_Z_POS->setText(QString().sprintf("%.3f", 0.));
    ui->OW_EDIT_M_POS_X->setText(QString().sprintf("%.3f", 0.6));
    ui->OW_EDIT_M_POS_Y->setText(QString().sprintf("%.3f", 0.35));
    ui->OW_EDIT_T_POS->setText(QString().sprintf("%.3f", 0.));
}
void OmniWheelDialog::on_OW_TRANSFORM_WALK_TO_WHEEL_clicked()
{
    // Transform from WALK -> WHEEL
//    ui->OW_TRANSFORM_WALK_TO_WHEEL->setEnabled(false);
//    ui->OW_TRANSFORM_WHEEL_TO_WALK->setEnabled(true);
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_CHANGEPOS;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);
}
void OmniWheelDialog::on_OW_TRANSFORM_WHEEL_TO_WALK_clicked()
{
//    ui->OW_TRANSFORM_WHEEL_TO_WALK->setEnabled(false);
//    ui->OW_TRANSFORM_WALK_TO_WHEEL->setEnabled(true);
    // Transform from WHEEL -> WALK
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=1;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_CHANGEPOS;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);
}


void OmniWheelDialog::on_OW_NORMAL_WALKREADY_clicked()
{
//    ui->OW_TRANSFORM_WALK_TO_WHEEL->setEnabled(true);
//    ui->OW_TRANSFORM_WHEEL_TO_WALK->setEnabled(true);
//    // normal walkready
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    pLAN->SendCommand(cmd);
}



//void OmniWheelDialog::on_OW_SEND_BOUNDARY_clicked()
//{
//    // Send next ROI bound point
//    // send:: next_LU_point
//    // send:: next_LD_point
//    // send:: next_RD_point
//    // send:: next_RU_point
//    char tosend[48];
//    memcpy(&tosend[0],next_LU_point,12);
//    memcpy(&tosend[12],next_LD_point,12);
//    memcpy(&tosend[24],next_RD_point,12);
//    memcpy(&tosend[36],next_RU_point,12);
//    QByteArray ttt(tosend,48);
//    pVC->server->RBSendData(ttt);
//}

void OmniWheelDialog::on_OW_KNEE_GAIN_START_clicked()
{
//    // Knee gain over-start
//    lanData->LANUserCommand->USER_PARA_CHAR[0]=2;
//    lanData->LANUserCommand->USER_COMMAND = OMNIWHEEL_AL_CONTROL;
//    lanData->LANUserCommand->COMMAND_TARGET = AlnumOmniWheel;
//    lanData->SendCommand();
}

void OmniWheelDialog::on_OW_KNEE_GAIN_STOP_clicked()
{
//    // Knee gain over-stop
//    lanData->LANUserCommand->USER_PARA_CHAR[0]=3;
//    lanData->LANUserCommand->USER_COMMAND = OMNIWHEEL_AL_CONTROL;
//    lanData->LANUserCommand->COMMAND_TARGET = AlnumOmniWheel;
//    lanData->SendCommand();
}


void OmniWheelDialog::on_SET_TO_REAL_clicked()
{
//    // Set to Real
//    lanData->LANUserCommand->USER_PARA_CHAR[0]=10;
//    lanData->LANUserCommand->USER_PARA_CHAR[1]=1;
//    lanData->LANUserCommand->USER_COMMAND = OMNIWHEEL_AL_CONTROL;
//    lanData->LANUserCommand->COMMAND_TARGET = AlnumOmniWheel;
//    lanData->SendCommand();
}

void OmniWheelDialog::on_SET_TO_TEST_clicked()
{
//    // Set to Test
//    lanData->LANUserCommand->USER_PARA_CHAR[0]=10;
//    lanData->LANUserCommand->USER_PARA_CHAR[1]=0;
//    lanData->LANUserCommand->USER_COMMAND = OMNIWHEEL_AL_CONTROL;
//    lanData->LANUserCommand->COMMAND_TARGET = AlnumOmniWheel;
//    lanData->SendCommand();
}

void OmniWheelDialog::on_BTN_ROS_MODE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_ROS;
    pLAN->SendCommand(cmd);
}
