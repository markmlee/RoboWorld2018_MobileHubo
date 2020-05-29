#include "JointDialog.h"
#include "ui_JointDialog.h"


inline void DisplayJointReference(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentReference));
}
inline void DisplayJointEncoder(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSEN.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentPosition));
}

JointDialog::JointDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointDialog)
{
    ui->setupUi(this);

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateJoints()));
}

JointDialog::~JointDialog()
{
    delete ui;
}

void JointDialog::UpdateJoints(){
    QString str;

    if(ui->RB_JOINT_REFERENCE->isChecked()){
//        DisplayJointReference(RHY  , ui->LE_JOINT_RHY);
//        DisplayJointReference(RHR  , ui->LE_JOINT_RHR);
//        DisplayJointReference(RHP  , ui->LE_JOINT_RHP);
//        DisplayJointReference(RKN  , ui->LE_JOINT_RKN);
//        DisplayJointReference(RAP  , ui->LE_JOINT_RAP);
//        DisplayJointReference(RAR  , ui->LE_JOINT_RAR);

        DisplayJointReference(RSP  , ui->LE_JOINT_RSP);
        DisplayJointReference(RSR  , ui->LE_JOINT_RSR);
        DisplayJointReference(RSY  , ui->LE_JOINT_RSY);
        DisplayJointReference(REB  , ui->LE_JOINT_REB);
        DisplayJointReference(RWY  , ui->LE_JOINT_RWY);
        DisplayJointReference(RWP  , ui->LE_JOINT_RWP);
        DisplayJointReference(RWY2 , ui->LE_JOINT_RWY2);
        DisplayJointReference(RHAND, ui->LE_JOINT_RHAND);

//        DisplayJointReference(LHY  , ui->LE_JOINT_LHY);
//        DisplayJointReference(LHR  , ui->LE_JOINT_LHR);
//        DisplayJointReference(LHP  , ui->LE_JOINT_LHP);
//        DisplayJointReference(LKN  , ui->LE_JOINT_LKN);
//        DisplayJointReference(LAP  , ui->LE_JOINT_LAP);
//        DisplayJointReference(LAR  , ui->LE_JOINT_LAR);

        DisplayJointReference(LSP  , ui->LE_JOINT_LSP);
        DisplayJointReference(LSR  , ui->LE_JOINT_LSR);
        DisplayJointReference(LSY  , ui->LE_JOINT_LSY);
        DisplayJointReference(LEB  , ui->LE_JOINT_LEB);
        DisplayJointReference(LWY  , ui->LE_JOINT_LWY);
        DisplayJointReference(LWP  , ui->LE_JOINT_LWP);
        DisplayJointReference(LWY2 , ui->LE_JOINT_LWY2);
        DisplayJointReference(LHAND, ui->LE_JOINT_LHAND);

        DisplayJointReference(WST  , ui->LE_JOINT_WST);
        DisplayJointReference(RWH  , ui->LE_JOINT_RWH);
        DisplayJointReference(LWH  , ui->LE_JOINT_LWH);
        DisplayJointReference(BWH  , ui->LE_JOINT_BWH);

//        DisplayJointReference(NKP1 , ui->LE_JOINT_NKP1);
//        DisplayJointReference(NKP2 , ui->LE_JOINT_NKP2);
//        DisplayJointReference(NKY  , ui->LE_JOINT_NKY);
//        DisplayJointReference(NKR  , ui->LE_JOINT_NKR);

//        DisplayJointReference(RF1  , ui->LE_JOINT_RF1);
//        DisplayJointReference(RF2  , ui->LE_JOINT_RF2);
//        DisplayJointReference(RF3  , ui->LE_JOINT_RF3);
//        DisplayJointReference(RF4  , ui->LE_JOINT_RF4);

//        DisplayJointReference(LF1  , ui->LE_JOINT_LF1);
//        DisplayJointReference(LF2  , ui->LE_JOINT_LF2);
//        DisplayJointReference(LF3  , ui->LE_JOINT_LF3);
//        DisplayJointReference(LF4  , ui->LE_JOINT_LF4);
    }else{
//        DisplayJointEncoder(RHY  , ui->LE_JOINT_RHY);
//        DisplayJointEncoder(RHR  , ui->LE_JOINT_RHR);
//        DisplayJointEncoder(RHP  , ui->LE_JOINT_RHP);
//        DisplayJointEncoder(RKN  , ui->LE_JOINT_RKN);
//        DisplayJointEncoder(RAP  , ui->LE_JOINT_RAP);
//        DisplayJointEncoder(RAR  , ui->LE_JOINT_RAR);

        DisplayJointEncoder(RSP  , ui->LE_JOINT_RSP);
        DisplayJointEncoder(RSR  , ui->LE_JOINT_RSR);
        DisplayJointEncoder(RSY  , ui->LE_JOINT_RSY);
        DisplayJointEncoder(REB  , ui->LE_JOINT_REB);
        DisplayJointEncoder(RWY  , ui->LE_JOINT_RWY);
        DisplayJointEncoder(RWP  , ui->LE_JOINT_RWP);
        DisplayJointEncoder(RWY2 , ui->LE_JOINT_RWY2);
        DisplayJointEncoder(RHAND, ui->LE_JOINT_RHAND);

//        DisplayJointEncoder(LHY  , ui->LE_JOINT_LHY);
//        DisplayJointEncoder(LHR  , ui->LE_JOINT_LHR);
//        DisplayJointEncoder(LHP  , ui->LE_JOINT_LHP);
//        DisplayJointEncoder(LKN  , ui->LE_JOINT_LKN);
//        DisplayJointEncoder(LAP  , ui->LE_JOINT_LAP);
//        DisplayJointEncoder(LAR  , ui->LE_JOINT_LAR);

        DisplayJointEncoder(LSP  , ui->LE_JOINT_LSP);
        DisplayJointEncoder(LSR  , ui->LE_JOINT_LSR);
        DisplayJointEncoder(LSY  , ui->LE_JOINT_LSY);
        DisplayJointEncoder(LEB  , ui->LE_JOINT_LEB);
        DisplayJointEncoder(LWY  , ui->LE_JOINT_LWY);
        DisplayJointEncoder(LWP  , ui->LE_JOINT_LWP);
        DisplayJointEncoder(LWY2 , ui->LE_JOINT_LWY2);
        DisplayJointEncoder(LHAND, ui->LE_JOINT_LHAND);

        DisplayJointEncoder(WST  , ui->LE_JOINT_WST);
        DisplayJointEncoder(RWH  , ui->LE_JOINT_RWH);
        DisplayJointEncoder(LWH  , ui->LE_JOINT_LWH);
        DisplayJointEncoder(BWH  , ui->LE_JOINT_BWH);

//        DisplayJointEncoder(NKP1 , ui->LE_JOINT_NKP1);
//        DisplayJointEncoder(NKP2 , ui->LE_JOINT_NKP2);
//        DisplayJointEncoder(NKY  , ui->LE_JOINT_NKY);
//        DisplayJointEncoder(NKR  , ui->LE_JOINT_NKR);

//        DisplayJointEncoder(RF1  , ui->LE_JOINT_RF1);
//        DisplayJointEncoder(RF2  , ui->LE_JOINT_RF2);
//        DisplayJointEncoder(RF3  , ui->LE_JOINT_RF3);
//        DisplayJointEncoder(RF4  , ui->LE_JOINT_RF4);

//        DisplayJointEncoder(LF1  , ui->LE_JOINT_LF1);
//        DisplayJointEncoder(LF2  , ui->LE_JOINT_LF2);
//        DisplayJointEncoder(LF3  , ui->LE_JOINT_LF3);
//        DisplayJointEncoder(LF4  , ui->LE_JOINT_LF4);
    }


//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RHY)][MC_GetCH(RHY)].b.GOV)  ui->LE_JOINT_RHY->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RHY->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RHR)][MC_GetCH(RHR)].b.GOV)  ui->LE_JOINT_RHR->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RHR->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RHP)][MC_GetCH(RHP)].b.GOV)  ui->LE_JOINT_RHP->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RHP->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RKN)][MC_GetCH(RKN)].b.GOV)  ui->LE_JOINT_RKN->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RKN->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RAP)][MC_GetCH(RAP)].b.GOV)  ui->LE_JOINT_RAP->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RAP->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RAR)][MC_GetCH(RAR)].b.GOV)  ui->LE_JOINT_RAR->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_RAR->setStyleSheet("background-color: white");

//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LHY)][MC_GetCH(LHY)].b.GOV)  ui->LE_JOINT_LHY->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LHY->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LHR)][MC_GetCH(LHR)].b.GOV)  ui->LE_JOINT_LHR->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LHR->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LHP)][MC_GetCH(LHP)].b.GOV)  ui->LE_JOINT_LHP->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LHP->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LKN)][MC_GetCH(LKN)].b.GOV)  ui->LE_JOINT_LKN->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LKN->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LAP)][MC_GetCH(LAP)].b.GOV)  ui->LE_JOINT_LAP->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LAP->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LAR)][MC_GetCH(LAR)].b.GOV)  ui->LE_JOINT_LAR->setStyleSheet("background-color: yellow");
//    else                                                                ui->LE_JOINT_LAR->setStyleSheet("background-color: white");

    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RSP)][MC_GetCH(RSP)].b.GOV)  ui->LE_JOINT_RSP->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_RSP->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RSR)][MC_GetCH(RSR)].b.GOV)  ui->LE_JOINT_RSR->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_RSR->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RSY)][MC_GetCH(RSY)].b.GOV)  ui->LE_JOINT_RSY->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_RSY->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(REB)][MC_GetCH(REB)].b.GOV)  ui->LE_JOINT_REB->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_REB->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RWY)][MC_GetCH(RWY)].b.GOV)  ui->LE_JOINT_RWY->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_RWY->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RWP)][MC_GetCH(RWP)].b.GOV)  ui->LE_JOINT_RWP->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_RWP->setStyleSheet("background-color: white");

    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LSP)][MC_GetCH(LSP)].b.GOV)  ui->LE_JOINT_LSP->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LSP->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LSR)][MC_GetCH(LSR)].b.GOV)  ui->LE_JOINT_LSR->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LSR->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LSY)][MC_GetCH(LSY)].b.GOV)  ui->LE_JOINT_LSY->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LSY->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LEB)][MC_GetCH(LEB)].b.GOV)  ui->LE_JOINT_LEB->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LEB->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LWY)][MC_GetCH(LWY)].b.GOV)  ui->LE_JOINT_LWY->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LWY->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LWP)][MC_GetCH(LWP)].b.GOV)  ui->LE_JOINT_LWP->setStyleSheet("background-color: yellow");
    else                                                                ui->LE_JOINT_LWP->setStyleSheet("background-color: white");

    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(WST)][MC_GetCH(WST)].b.GOV)      ui->LE_JOINT_WST->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_WST->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RWY2)][MC_GetCH(RWY2)].b.GOV)    ui->LE_JOINT_RWY2->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_RWY2->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RHAND)][MC_GetCH(RHAND)].b.GOV)  ui->LE_JOINT_RHAND->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_RHAND->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LWY2)][MC_GetCH(LWY2)].b.GOV)    ui->LE_JOINT_LWY2->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_LWY2->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LHAND)][MC_GetCH(LHAND)].b.GOV)  ui->LE_JOINT_LHAND->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_LHAND->setStyleSheet("background-color: white");

    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(RWH)][MC_GetCH(RWH)].b.GOV)      ui->LE_JOINT_RWH->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_RWH->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(LWH)][MC_GetCH(LWH)].b.GOV)      ui->LE_JOINT_LWH->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_LWH->setStyleSheet("background-color: white");
    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(BWH)][MC_GetCH(BWH)].b.GOV)      ui->LE_JOINT_BWH->setStyleSheet("background-color: yellow");
    else                                                                    ui->LE_JOINT_BWH->setStyleSheet("background-color: white");

//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(NKP1)][MC_GetCH(NKP1)].b.GOV)    ui->LE_JOINT_NKP1->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_NKP1->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(NKY)][MC_GetCH(NKY)].b.GOV)      ui->LE_JOINT_NKY->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_NKY->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(NKP2)][MC_GetCH(NKP2)].b.GOV)    ui->LE_JOINT_NKP2->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_NKP2->setStyleSheet("background-color: white");
//    if(PODO_DATA.CoreSEN.MCStatus[MC_GetID(NKR)][MC_GetCH(NKR)].b.GOV)      ui->LE_JOINT_NKR->setStyleSheet("background-color: yellow");
//    else                                                                    ui->LE_JOINT_NKR->setStyleSheet("background-color: white");

}

void JointDialog::on_BTN_ENC_ENABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JointDialog::on_BTN_ENC_DISABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // off
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
