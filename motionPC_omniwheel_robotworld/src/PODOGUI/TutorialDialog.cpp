#include "TutorialDialog.h"
#include "ui_TutorialDialog.h"

#include "BasicFiles/PODOALDialog.h"


enum DebrisALCOMMAND
{
    DemoFireAL_NO_ACT = 100,
    DemoFireAL_READYPOS,
    DemoFireAL_MOTIONSTART,
    DemoFireAL_TEST,
    DemoFireAL_SET,
    DemoFireAL_SW_READY,
    DemoFireAL_SW_SET,
    DemoFireAL_SW_GRASP,
    DemoFireAL_SW_HOLD,
    DemoFireAL_SW_BACK
};


enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    TUTORIAL_FINGER_CONTROL
};


TutorialDialog::TutorialDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TutorialDialog)
{
    ui->setupUi(this);
    ALNum_Tuto = PODOALDialog::GetALNumFromFileName("ALTutorial");
    ALNum_Demo = PODOALDialog::GetALNumFromFileName("DemoFireAL");

    EXF_R = EXF_L = 0;  // default gripper

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSomethings()));
}

TutorialDialog::~TutorialDialog()
{
    delete ui;
}


void TutorialDialog::UpdateSomethings(){
    int changed_flag = false;
    if(PODO_DATA.CoreSEN.EXF_R_Enabled){
        if(EXF_R == 0)  changed_flag = true;
        EXF_R = 1;
        ui->LB_RF_MODE->setText("Right: 5Ch Finger Mode");
    }else{
        if(EXF_R == 1)  changed_flag = true;
        EXF_R = 0;
        ui->LB_RF_MODE->setText("Right: 1Ch Gripper Mode");
    }

    if(PODO_DATA.CoreSEN.EXF_L_Enabled){
        if(EXF_L == 0)  changed_flag = true;
        EXF_L = 1;
        ui->LB_LF_MODE->setText("Left: 5Ch Finger Mode");
    }else{
        if(EXF_L == 1)  changed_flag = true;
        EXF_L = 0;
        ui->LB_LF_MODE->setText("Left: 1Ch Gripper Mode");
    }


    // if finger status changed --> enable & disable buttons
    if(changed_flag == true){
        if(EXF_R == 0){
            //ui->BTN_R_GRASP0->setEnabled(false);
            ui->BTN_R_GRASP1->setEnabled(false);
            ui->BTN_R_GRASP2->setEnabled(false);
            ui->BTN_R_GRASP3->setEnabled(false);
            ui->BTN_R_GRASP4->setEnabled(false);
            ui->BTN_R_GRASP_ALL->setEnabled(false);
            //ui->BTN_R_RELEASE0->setEnabled(false);
            ui->BTN_R_RELEASE1->setEnabled(false);
            ui->BTN_R_RELEASE2->setEnabled(false);
            ui->BTN_R_RELEASE3->setEnabled(false);
            ui->BTN_R_RELEASE4->setEnabled(false);
            ui->BTN_R_RELEASE_ALL->setEnabled(false);
        }else{
            //ui->BTN_R_GRASP0->setEnabled(true);
            ui->BTN_R_GRASP1->setEnabled(true);
            ui->BTN_R_GRASP2->setEnabled(true);
            ui->BTN_R_GRASP3->setEnabled(true);
            ui->BTN_R_GRASP4->setEnabled(true);
            ui->BTN_R_GRASP_ALL->setEnabled(true);
            //ui->BTN_R_RELEASE0->setEnabled(true);
            ui->BTN_R_RELEASE1->setEnabled(true);
            ui->BTN_R_RELEASE2->setEnabled(true);
            ui->BTN_R_RELEASE3->setEnabled(true);
            ui->BTN_R_RELEASE4->setEnabled(true);
            ui->BTN_R_RELEASE_ALL->setEnabled(true);
        }

        if(EXF_L == 0){
            //ui->BTN_L_GRASP0->setEnabled(false);
            ui->BTN_L_GRASP1->setEnabled(false);
            ui->BTN_L_GRASP2->setEnabled(false);
            ui->BTN_L_GRASP3->setEnabled(false);
            ui->BTN_L_GRASP4->setEnabled(false);
            ui->BTN_L_GRASP_ALL->setEnabled(false);
            //ui->BTN_L_RELEASE0->setEnabled(false);
            ui->BTN_L_RELEASE1->setEnabled(false);
            ui->BTN_L_RELEASE2->setEnabled(false);
            ui->BTN_L_RELEASE3->setEnabled(false);
            ui->BTN_L_RELEASE4->setEnabled(false);
            ui->BTN_L_RELEASE_ALL->setEnabled(false);
        }else{
            //ui->BTN_L_GRASP0->setEnabled(true);
            ui->BTN_L_GRASP1->setEnabled(true);
            ui->BTN_L_GRASP2->setEnabled(true);
            ui->BTN_L_GRASP3->setEnabled(true);
            ui->BTN_L_GRASP4->setEnabled(true);
            ui->BTN_L_GRASP_ALL->setEnabled(true);
            //ui->BTN_L_RELEASE0->setEnabled(true);
            ui->BTN_L_RELEASE1->setEnabled(true);
            ui->BTN_L_RELEASE2->setEnabled(true);
            ui->BTN_L_RELEASE3->setEnabled(true);
            ui->BTN_L_RELEASE4->setEnabled(true);
            ui->BTN_L_RELEASE_ALL->setEnabled(true);
        }
    }

    if(modiCnt < 20){
        ui->LE_MODI_R0->setReadOnly(true);
        ui->LE_MODI_R1->setReadOnly(true);
        ui->LE_MODI_R2->setReadOnly(true);
        ui->LE_MODI_R3->setReadOnly(true);
        ui->LE_MODI_R4->setReadOnly(true);

        ui->LE_MODI_L0->setReadOnly(true);
        ui->LE_MODI_L1->setReadOnly(true);
        ui->LE_MODI_L2->setReadOnly(true);
        ui->LE_MODI_L3->setReadOnly(true);
        ui->LE_MODI_L4->setReadOnly(true);

        ui->LE_MODI_R0->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_R_Modifier[0]));
        ui->LE_MODI_R1->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_R_Modifier[1]));
        ui->LE_MODI_R2->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_R_Modifier[2]));
        ui->LE_MODI_R3->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_R_Modifier[3]));
        ui->LE_MODI_R4->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_R_Modifier[4]));

        ui->LE_MODI_L0->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_L_Modifier[0]));
        ui->LE_MODI_L1->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_L_Modifier[1]));
        ui->LE_MODI_L2->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_L_Modifier[2]));
        ui->LE_MODI_L3->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_L_Modifier[3]));
        ui->LE_MODI_L4->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.EXF_L_Modifier[4]));

        modiCnt++;
    }else if(modiCnt <25){
        ui->LE_MODI_R0->setReadOnly(false);
        ui->LE_MODI_R1->setReadOnly(false);
        ui->LE_MODI_R2->setReadOnly(false);
        ui->LE_MODI_R3->setReadOnly(false);
        ui->LE_MODI_R4->setReadOnly(false);

        ui->LE_MODI_L0->setReadOnly(false);
        ui->LE_MODI_L1->setReadOnly(false);
        ui->LE_MODI_L2->setReadOnly(false);
        ui->LE_MODI_L3->setReadOnly(false);
        ui->LE_MODI_L4->setReadOnly(false);

        modiCnt++;
    }
}

void TutorialDialog::on_BTN_TEST_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 999;
    cmd.COMMAND_TARGET = ALNum_Tuto;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SCAN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    pLAN->G2MData->ros_cmd.param_f[0] = ui->LE_SCAN_START->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.param_f[1] = ui->LE_SCAN_END->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.param_f[2] = ui->LE_SCAN_TIME->text().toFloat();
    pLAN->G2MData->ros_cmd.cmd = 1;

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_GOTO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    pLAN->G2MData->ros_cmd.param_f[0] = ui->LE_GOTO_ANGLE->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.cmd = 2;

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_READY;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_SET;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_SW_PX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_SW_PY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_SW_PZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_SW_WZ->text().toDouble();

    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_GRASP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_GRASP;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_HOLD_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_HOLD;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_BACK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_BACK;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_GET_POS_clicked()
{
    ui->LE_SW_PX->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[0]));
    ui->LE_SW_PY->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[1]));
    ui->LE_SW_PZ->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[2]));
}

void TutorialDialog::on_NRLALButton_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 999;

    cmd.COMMAND_TARGET = PODOALDialog::GetALNumFromFileName("ALNRL");; // NRL AL number
    pLAN->SendCommand(cmd);
}




void TutorialDialog::on_BTN_R_GRASP0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}





void TutorialDialog::on_BTN_L_GRASP0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}



void TutorialDialog::on_BTN_INIT_RHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;      // right hand
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_INIT_LHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;      // left hand
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_INIT_ALLHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 3;      // both hand
    pLAN->SendCommand(cmd);
}



void TutorialDialog::on_BTN_READ_MODI_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_SET_FINGER_MODIFIER;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);

    modiCnt = 0;
}

void TutorialDialog::on_BTN_SET_MODI_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_SET_FINGER_MODIFIER;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;

    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_MODI_R0->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_MODI_R1->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[2] = ui->LE_MODI_R2->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[3] = ui->LE_MODI_R3->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[4] = ui->LE_MODI_R4->text().toFloat();

    cmd.COMMAND_DATA.USER_PARA_FLOAT[5] = ui->LE_MODI_L0->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[6] = ui->LE_MODI_L1->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[7] = ui->LE_MODI_L2->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[8] = ui->LE_MODI_L3->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[9] = ui->LE_MODI_L4->text().toFloat();

    pLAN->SendCommand(cmd);
}
