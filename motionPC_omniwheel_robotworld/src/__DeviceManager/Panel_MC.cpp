#include "Panel_MC.h"
#include "ui_Panel_MC.h"


extern int     _SrvOnFlag;
extern int     _HomeStartFlag;
extern int     _EncoderZeroFlag;
extern int     _HomeSequence;


Panel_MC::Panel_MC(QWidget *parent, RBMotorController *mc) :
    QDialog(parent),
    ui(new Ui::Panel_MC)
{
    ui->setupUi(this);

    OpMode = OPMODE_DUTY;
    ui->RB_DUTY->setChecked(true);
    CurChannel = 0;

    dev_info = mc;
    for(int i=0; i<dev_info->MOTOR_CHANNEL; i++){
        ui->CB_CHANNEL->addItem(QString().sprintf("%d", i));
    }
    DutyValue = 10;
    VelValue = 10;
    PosValue = 4000;

    RefreshBoardInfo();


    displayTimer = new QTimer();
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(Display()));
    displayTimer->start(100);
}

Panel_MC::~Panel_MC()
{
    delete ui;
}

void Panel_MC::Display(){
    // encoder display
    ui->LE_ENCODER->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].EncoderValue));

    // status display
    if(dev_info->Joints[CurChannel].CurrentStatus.b.HIP == 1)
        ui->LE_FET->setStyleSheet("background-color: green");
    else
        ui->LE_FET->setStyleSheet("background-color: red");

    if(dev_info->Joints[CurChannel].CurrentStatus.b.RUN == 1)
        ui->LE_RUN->setStyleSheet("background-color: green");
    else
        ui->LE_RUN->setStyleSheet("background-color: red");

    if(dev_info->Joints[CurChannel].CurrentStatus.b.LIM == 1)
        ui->LE_LIMIT->setStyleSheet("background-color: green");
    else
        ui->LE_LIMIT->setStyleSheet("background-color: red");
}


void Panel_MC::on_RB_DUTY_clicked(bool checked){
    if(checked){
        OpMode = OPMODE_DUTY;
        ui->LB_MODE->setText("D:");
    }
    RefreshBoardInfo();
}
void Panel_MC::on_RB_VEL_clicked(bool checked){
    if(checked){
        OpMode = OPMODE_VEL;
        ui->LB_MODE->setText("V:");
    }
    RefreshBoardInfo();
}
void Panel_MC::on_RB_POS_clicked(bool checked){
    if(checked){
        OpMode = OPMODE_POS;
        ui->LB_MODE->setText("P:");
    }
    RefreshBoardInfo();
}


void Panel_MC::on_CB_CHANNEL_currentIndexChanged(int index){
    CurChannel = index;
    RefreshBoardInfo();
}


void Panel_MC::RefreshBoardInfo(){
    QString str;
    str.sprintf("%2d - %d", dev_info->BOARD_ID, CurChannel);
    switch(OpMode){
    case OPMODE_DUTY:
        str += " D";
        break;
    case OPMODE_VEL:
        str += " V";
        break;
    case OPMODE_POS:
        str += " P";
        break;
    }
    ui->LB_BOARD_INFO->setText(str);
    RefreshOpValue();
}


void Panel_MC::NewAccess(){
    for(int i=0; i<dev_info->MOTOR_CHANNEL; i++){
        dev_info->RBJoint_EnableFeedbackControl(i+1, false);
        dev_info->RBJoint_EnableFETDriver(i+1, false);
    }
    on_BTN_PARAMETER_SCAN_clicked();
}



void Panel_MC::on_BTN_FET_ON_clicked(){
    dev_info->RBJoint_EnableFETDriver(CurChannel+1, true);
}
void Panel_MC::on_BTN_FET_OFF_clicked(){
    dev_info->RBJoint_EnableFETDriver(CurChannel+1, false);
}
void Panel_MC::on_BTN_SRV_ON_clicked(){
    _SrvOnFlag = true;
    //dev_info->RBJoint_EnableFeedbackControl(CurChannel+1, true);
}
void Panel_MC::on_BTN_SRV_OFF_clicked(){
    dev_info->RBJoint_EnableFeedbackControl(CurChannel+1, false);
}
void Panel_MC::on_BTN_HOME_clicked(){
    _HomeSequence = 0;
    _HomeStartFlag = true;
    //dev_info->RBJoint_FindHome(CurChannel+1);
}
void Panel_MC::on_BTN_ENC_ZERO_clicked(){
    _EncoderZeroFlag = true;
    //dev_info->RBJoint_ResetEncoder(CurChannel+1);
}


void Panel_MC::on_BTN_PLUS_clicked(){
    switch(OpMode){
    case OPMODE_DUTY:
        DutyValue += 1;
        if(DutyValue > 100) DutyValue = 100;
        break;
    case OPMODE_VEL:
        VelValue += 1;
        if(VelValue > 100) VelValue = 100;
        break;
    case OPMODE_POS:
        PosValue += 1000;
        if(PosValue > 20000) PosValue = 20000;
        break;
    }
    RefreshOpValue();
}

void Panel_MC::on_BTN_MINUS_clicked(){
    switch(OpMode){
    case OPMODE_DUTY:
        DutyValue -= 1;
        if(DutyValue < 1) DutyValue = 1;
        break;
    case OPMODE_VEL:
        VelValue -= 1;
        if(VelValue < 1) VelValue = 1;
        break;
    case OPMODE_POS:
        PosValue -= 1000;
        if(PosValue < 1000) PosValue = 1000;
        break;
    }
    RefreshOpValue();
}

void Panel_MC::RefreshOpValue(){
    switch(OpMode){
    case OPMODE_DUTY:
        ui->LE_MODE_VALUE->setText(QString().sprintf("%d", DutyValue));
        break;
    case OPMODE_VEL:
        ui->LE_MODE_VALUE->setText(QString().sprintf("%d", VelValue));
        break;
    case OPMODE_POS:
        ui->LE_MODE_VALUE->setText(QString().sprintf("%d", PosValue));
        break;
    }
}

void Panel_MC::on_BTN_PARAMETER_SCAN_clicked(){
    dev_info->RBJoint_LoadParameter(CurChannel+1);

    ui->LE_KP->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].GainKp));
    ui->LE_KD->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].GainKd));
    ui->LE_KI->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].GainKi));

    ui->LE_ENC_RES->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].EncoderResolution));
    ui->LE_ENC_AUTOSCALE->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].AutoScale));
    ui->LE_ENC_DIR->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].PositiveDirection));

    ui->LE_DEADZONE->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].Deadzone));

    ui->LE_HOME_MODE->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].HomeSearchMode));
    ui->LE_HOME_DIR->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].SearchDirection));
    ui->LE_HOME_REV->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].LimitRevolution));
    ui->LE_HOME_OFFSET->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].OffsetAngle));

    ui->LE_HOME_ACC->setText(QString().sprintf("%.1f", dev_info->Joints[CurChannel].MaxAccelerationDuringHome/100.0));
    ui->LE_HOME_VEL->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].MaxVelocityDuringHome));
    ui->LE_HOME_OFFSET_VEL->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].MaxVelocityDuringGoOffset));

    ui->LE_LOWER_LIMIT->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].LowerPositionLimit));
    ui->LE_UPPER_LIMIT->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].UpperPositionLimit));
    ui->LE_LOWER_USE->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].UseLowerLimit));
    ui->LE_UPPER_USE->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].UseUpperLimit));

    ui->LE_MAX_ACC->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].MaxAcceleration));
    ui->LE_MAX_VEL->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].MaxVelocity));

    ui->LE_JAM_DUTY->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].JAMDuty));
    ui->LE_JAM_TIME->setText(QString().sprintf("%.1f", dev_info->Joints[CurChannel].JAMmsTime/10.0));
    ui->LE_PWM_DUTY->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].PWMDuty));
    ui->LE_PWM_TIME->setText(QString().sprintf("%.1f", dev_info->Joints[CurChannel].PWMmsTime/10.0));

    ui->LE_INP_ERROR->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].InputErrorLimitValue));
    ui->LE_BIG_ERROR->setText(QString().sprintf("%d", dev_info->Joints[CurChannel].BigErrorLimitValue));
}

void Panel_MC::on_BTN_SET_GAIN_clicked(){
    if(CurChannel == 0){
        dev_info->RBBoard_SetMotorPositionGain0(ui->LE_KP->text().toInt(), ui->LE_KI->text().toInt(), ui->LE_KD->text().toInt());
    }else if(CurChannel == 1){
        dev_info->RBBoard_SetMotorPositionGain1(ui->LE_KP->text().toInt(), ui->LE_KI->text().toInt(), ui->LE_KD->text().toInt());
    }
}

void Panel_MC::on_BTN_SET_DEADZONE_clicked(){
    dev_info->RBJoint_SetDeadzone(CurChannel+1, ui->LE_DEADZONE->text().toInt());
}

void Panel_MC::on_BTN_SET_ENC_RES_clicked(){
    dev_info->RBJoint_SetEncoderResolution(CurChannel+1, ui->LE_ENC_RES->text().toInt(), ui->LE_ENC_AUTOSCALE->text().toInt(), ui->LE_ENC_DIR->text().toInt());
}

void Panel_MC::on_BTN_SET_HOME_INFO_clicked(){
    dev_info->RBJoint_SetHomeSearchParameter(CurChannel+1, ui->LE_HOME_REV->text().toInt(), ui->LE_HOME_DIR->text().toInt(), ui->LE_HOME_OFFSET->text().toInt());
}

void Panel_MC::on_BTN_SET_HOME_INFO_2_clicked(){
    dev_info->RBJoint_SetMaxAccVelForHomeSearch(CurChannel+1, ui->LE_HOME_ACC->text().toInt(), ui->LE_HOME_VEL->text().toInt(), ui->LE_HOME_OFFSET_VEL->text().toInt(),
                                                ui->LE_HOME_MODE->text().toInt(), ui->LE_PWM_DUTY->text().toInt());
}

void Panel_MC::on_BTN_SET_LOWER_LIMIT_clicked(){
    dev_info->RBJoint_SetLowerPosLimit(CurChannel+1, ui->LE_LOWER_LIMIT->text().toInt(), ui->LE_LOWER_USE->text().toInt()+2);
}
void Panel_MC::on_BTN_SET_UPPER_LIMIT_clicked(){
    dev_info->RBJoint_SetUpperPosLimit(CurChannel+1, ui->LE_UPPER_LIMIT->text().toInt(), ui->LE_UPPER_USE->text().toInt()+2);
}
