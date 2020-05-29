#include "RobotWorldDialog.h"
#include "ui_RobotWorldDialog.h"

RobotWorldDialog::RobotWorldDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobotWorldDialog)
{
    ui->setupUi(this);
    AlNumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
    AlNumRobotWorld = PODOALDialog::GetALNumFromFileName("RobotWorld");
//    ui->PB_WHEEL_MOVE_BACK->setEnabled(false);
    connect(pLAN,SIGNAL(NewPODOData()),this,SLOT(UpdateData()));
}

RobotWorldDialog::~RobotWorldDialog()
{
    delete ui;
}

void RobotWorldDialog::UpdateData()
{
    QString str;
    ui->LE_ROBOT_POS_X->setText(str.sprintf("%.3f", PODO_DATA.UserM2G.OMNIRobotPosX));
    ui->LE_ROBOT_POS_Y->setText(str.sprintf("%.3f", PODO_DATA.UserM2G.OMNIRobotPosY));
    ui->LE_ROBOT_POS_YAW->setText(str.sprintf("%.3f", PODO_DATA.UserM2G.OMNIRobotPosYaw*R2D));


    ui->LE_ROBOT_POS_X_SLAM->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.robot_pos.pos_x));
    ui->LE_ROBOT_POS_Y_SLAM->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.robot_pos.pos_y));
    ui->LE_ROBOT_POS_YAW_SLAM->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.robot_pos.ori_w));

    ui->LE_OBJECT_POS_X->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.Objpos[0].pos_x));
    ui->LE_OBJECT_POS_Y->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.Objpos[0].pos_y));
    ui->LE_OBJECT_POS_Z->setText(str.sprintf("%.3f", PODO_DATA.UserR2M.Objpos[0].pos_z));
}

void RobotWorldDialog::on_PB_WALK_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_GO_TO_DES_clicked()
{
    double des_x = ui->LE_OMNI_X->text().toDouble();
    double des_y = ui->LE_OMNI_Y->text().toDouble();
    double des_rot = ui->LE_OMNI_ROT->text().toDouble();
    double des_time = ui->LE_OMNI_TIME->text().toDouble();


    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_OMNIWHEEL_MOVE;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = des_x;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = des_y;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = des_rot;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = des_time;

    if(ui->RB_WST_180->isChecked() == true)
    {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] == 1;
    }
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_OMNIWHEEL_MOVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_GO_TO_DES_NEW_clicked()
{
    double des_x = ui->LE_OMNI_X->text().toDouble();
    double des_y = ui->LE_OMNI_Y->text().toDouble();
    double des_rot = ui->LE_OMNI_ROT->text().toDouble();
    double des_time = ui->LE_OMNI_TIME->text().toDouble();

    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_OMNIWHEEL_MOVE;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = des_x;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = des_y;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = des_rot;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = des_time;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_RHAND_GO_clicked()
{
    USER_COMMAND cmd;
    double pos[3], ori[3],quat[4], Elb_deg;

    pos[0] = ui->LE_HANDPOS_X->text().toDouble();
    pos[1] = ui->LE_HANDPOS_Y->text().toDouble();
    pos[2] = ui->LE_HANDPOS_Z->text().toDouble();
    Elb_deg = ui->LE_HAND_ELB->text().toDouble();

    quat[0] = ui->LE_HANDORI_w->text().toDouble();
    quat[1] = ui->LE_HANDORI_x->text().toDouble();
    quat[2] = ui->LE_HANDORI_y->text().toDouble();
    quat[3] = ui->LE_HANDORI_z->text().toDouble();

    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_HAND_GO;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;//right

    for(int i=0;i<3;i++)
    {
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i] = pos[i];
    }

    for(int i=0;i<4;i++)
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[3+i] = quat[i];

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = Elb_deg;


    printf("Move RHAND pos = [%f,%f,%f] ori = [%f,%f,%f] elb = [%f]\n",pos[0],pos[1],pos[1],ori[0],ori[1],ori[2], Elb_deg);

    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_HAND_GRASP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_HAND_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_HAND_OPEN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_GRASPING;
    if(ui->RB_HANDboth->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 0;
    if(ui->RB_HANDr->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    if(ui->RB_HANDl->isChecked() == true)    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_GRASP_clicked()
{
    double x = ui->LE_VPOS_X->text().toDouble();
    double y = ui->LE_VPOS_Y->text().toDouble();
    double z = ui->LE_VPOS_Z->text().toDouble();
    double elb = ui->LE_HAND_ELB_2->text().toDouble();
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_VISION_GRASP;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = x;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = y;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = z;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = elb;
    pLAN->SendCommand(cmd);

}

void RobotWorldDialog::on_PB_ESTOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_VISION_GRASP;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    pLAN->SendCommand(cmd);

}

void RobotWorldDialog::on_PB_PUT_clicked()
{
    double x = ui->LE_VPOS_X->text().toDouble();
    double y = ui->LE_VPOS_Y->text().toDouble();
    double z = ui->LE_VPOS_Z->text().toDouble();
    double elb = ui->LE_HAND_ELB_2->text().toDouble();
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_VISION_GRASP;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = x;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = y;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = z;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = elb;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_PB_WHEEL_MOVE_FRONT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_OMNIMOVE;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    pLAN->SendCommand(cmd);
//    ui->PB_WHEEL_MOVE_FRONT->set/*Enable*/d(false);
//    ui->PB_WHEEL_MOVE_BACK->setEnabled(true);
}

void RobotWorldDialog::on_PB_WHEEL_MOVE_BACK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_OMNIMOVE;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    pLAN->SendCommand(cmd);
//    ui->PB_WHEEL_MOVE_FRONT->setEnabled(true);
//    ui->PB_WHEEL_MOVE_BACK->setEnabled(false);
}

void RobotWorldDialog::on_PB_RESET_ROBOTPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlNumRobotWorld;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_RESET;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_pushButton_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_TEST;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    pLAN->SendCommand(cmd);
}

void RobotWorldDialog::on_pushButton_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ROBOTWORLD_AL_TEST;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    pLAN->SendCommand(cmd);
}
