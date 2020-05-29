#include "walkingdialog.h"
#include "ui_walkingdialog.h"

#include <math.h>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QDir>
#include <iostream>


#include "BasicFiles/PODOALDialog.h"


const double L_PEL2PEL = 0.2;
const double Offset_to_WR = 0.06;
double _RF_POS[3],_RF_ORI[3],_LF_POS[3],_LF_ORI[3],_G2M_block_edge[4][12];

using namespace std;
enum DEMOFLAG
{
    FOOTUP=100,
    KNEEDOWN,
    FOOTDOWN,
    BOW,
    HIT_HIT,
    HIT_READY,
    HIT_RETURN
};
enum WALKREADYCOMMAND
{
	WALKREADY_NO_ACT = 100,
	WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};
enum FREEWALKCOMMAND
{
    FREEWALK_NO_ACT = 100,
    FREEWALK_CUR_DSP,
    FREEWALK_WALK,
    FREEWALK_SAVE,
    FREEWALK_CONTROL_TEST,
    FREEWALK_PRE_WALK,
    FREEWALK_MOTION_CHECK,
    FREEWALK_INITIALIZE,
    FREEWALK_TERRAIN,
    FREEWALK_ADDMASS,
    FREEWALK_WIDE,
    FREEWALK_DSP_HOLD_WALK,
    FREEWALK_STOP
};

enum{
    NORMAL_WALKING =0,
    TERRAIN_WALKING,
    TERRAIN_WALKING_ONE_STEP,
    LADDER_WALKING,
    GOAL_WALKING
};
enum Inside_OutsideMode
{
    INSIDE_WALKING = 0,
    OUTSIDE_WALKING
};
enum WalkingModeCommand{
    FORWARD_WALKING =0,
    BACKWARD_WALKING,
    RIGHTSIDE_WALKING,
    LEFTSIDE_WALKING,
    CWROT_WALKING,
    CCWROT_WALKING,
    GOTOWR_WALKING
};
enum WalkingStopModeCommand{
    COMPLETE_STOP_WALKING = 0,
    SPREAD_STOP_WALKING
};

enum WalkingGoalGoModeCommand
{
    GOALGO_NORMAL = 0,
    GOALGO_FOR_SIDE,
    GOALGO_SIDE_FOR,
    GOALGO_DIAGONAL,
};

enum TerrainMode
{
    GUI_TERRAIN =0,
    FIELD_TERRAIN,
    OCS_TERRAIN
};


// Save & Load the DSPSchedule Data
class SaveDSPScheduleData{
public:
    SaveDSPScheduleData()	{}
    double		left[3];
    double		right[3];
    double		lyaw;
    double		ryaw;
    double      htime;
    double      COMz[2];

    double      rroll;
    double      rpitch;
    double      lroll;
    double      lpitch;
};
QDataStream &operator<<(QDataStream &out, const QVector<SaveDSPScheduleData> &vec){
    out << vec.size();
    for(int i=0; i<vec.size(); i++){
        out << vec[i].left[0] << vec[i].left[1] << vec[i].left[2] << vec[i].lyaw << vec[i].lroll << vec[i].lpitch
            << vec[i].right[0] << vec[i].right[1] << vec[i].right[2] << vec[i].ryaw << vec[i].rroll << vec[i].rpitch << vec[i].htime << vec[i].COMz[0] << vec[i].COMz[1];
    }
    return out;
}
QDataStream &operator>>(QDataStream &in, QVector<SaveDSPScheduleData> &vec){
    int size;
    in >> size;
    for(int i=0; i<size; i++){
        SaveDSPScheduleData sdsp;
        in >> sdsp.left[0] >> sdsp.left[1] >> sdsp.left[2] >> sdsp.lyaw >> sdsp.lroll >> sdsp.lpitch
                >>sdsp.right[0] >> sdsp.right[1] >> sdsp.right[2] >> sdsp.ryaw >> sdsp.rroll >> sdsp.rpitch >> sdsp.htime >> sdsp.COMz[0] >> sdsp.COMz[1];
        vec.push_back(sdsp);
    }
    return in;
}


WalkingDialog::WalkingDialog(QWidget *parent) :
    QDialog(parent),
	ui(new Ui::WalkingDialog)
{
	ui->setupUi(this);

    alNumWalkReady      = PODOALDialog::GetALNumFromFileName("WalkReady");
    alNumFreeWalking    = PODOALDialog::GetALNumFromFileName("FreeWalking");


    tableFont.setPointSize(8);
    ui->TW_DSP_SCHEDULER->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->TW_DSP_SCHEDULER->setColumnWidth(0, 30);
    ui->TW_DSP_SCHEDULER->setColumnWidth(1, 170);
    ui->TW_DSP_SCHEDULER->setColumnWidth(2, 170);
    ui->TW_DSP_SCHEDULER->setColumnWidth(3, 60);


    DSPScheduler.clear();
    DSPTask initialPos;
    initialPos.Left[0] = 0.0;
    initialPos.Left[1] = L_PEL2PEL/2.;
    initialPos.Left[2] = 0.0;

    initialPos.Right[0] = 0.0;
    initialPos.Right[1] = -L_PEL2PEL/2.;
    initialPos.Right[2] = 0.0;
    initialPos.HTime = 0.0;


	displayTimer = new QTimer(this);
	connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
	displayTimer->start(50);

}

WalkingDialog::~WalkingDialog(){
	delete ui;
}

void WalkingDialog::DisplayUpdate(){
	QString str;	
    ui->LE_CUR_LEFT_X->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[0]));
    ui->LE_CUR_LEFT_Y->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[1]));
    ui->LE_CUR_LEFT_Z->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[2]));
    ui->LE_CUR_LEFT_YAW->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[3]));
    ui->LE_CUR_LEFT_ROLL->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[4]));
    ui->LE_CUR_LEFT_PITCH->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootL[5]));

    ui->LE_CUR_RIGHT_X->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[0]));
    ui->LE_CUR_RIGHT_Y->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[1]));
    ui->LE_CUR_RIGHT_Z->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[2]));
    ui->LE_CUR_RIGHT_YAW->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[3]));
    ui->LE_CUR_RIGHT_ROLL->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[4]));
    ui->LE_CUR_RIGHT_PITCH->setText(str.sprintf("%.4f", PODO_DATA.UserM2G.curFootR[5]));

    double temp_lower_foot;

    temp_lower_foot = min(PODO_DATA.UserM2G.curFootL[2],PODO_DATA.UserM2G.curFootR[2]);
    ui->LE_CUR_COM_Z_OFFSET->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.curZMP[2]));
//        ui->LE_CUR_ZMP_X->setText(str.sprintf("%.4f",pLAN->LANPODOData->GUI_PARA_FLOAT[9]));
//        ui->LE_CUR_ZMP_Y->setText(str.sprintf("%.4f",pLAN->LANPODOData->GUI_PARA_FLOAT[10]));

    ui->LE_CUR_ZMP_X->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.curZMP[0]));
    ui->LE_CUR_ZMP_Y->setText(str.sprintf("%.4f",PODO_DATA.UserM2G.curZMP[1]));

    double angle1,angle2,yaw,roll,pitch;
    double VISION_RMAT[3][3],offset[3];
    double VISION_OFFSET[3];

//    VisionData.offset[0] = 0.163-0.1843+0.079+0.02;//-0.118;////(0.732-0.54);
//    VisionData.offset[1] = -0.024-0.008+0.018+0.004f;//(-0.288-(-0.273));
//    VisionData.offset[2] = 0.5101 + 0.036 +0.015 + 0.0376 +0.005f;// - 0.0046 + 0.0049 -0.0066+0.0035-0.0015+0.0018;//(-0.749-(-1.251));

////    VISION_OFFSET[0] = (0.732-0.54);
////    VISION_OFFSET[1] = (-0.288-(-0.273));
////    VISION_OFFSET[2] = (0.0-(-1.251));

//    angle1 = atan2(-VisionData.nX,-VisionData.nZ)*180/PI;
//    angle2 = atan2(-VisionData.nY,-VisionData.nZ)*180/PI;


//    VISION_RMAT[0][0] = VisionData.xX;
//    VISION_RMAT[1][0] = VisionData.xY;
//    VISION_RMAT[2][0] = VisionData.xZ;

//    VISION_RMAT[0][1] = VisionData.yX;
//    VISION_RMAT[1][1] = VisionData.yY;
//    VISION_RMAT[2][1] = VisionData.yZ;

//    VISION_RMAT[0][2] = VisionData.nX;
//    VISION_RMAT[1][2] = VisionData.nY;
//    VISION_RMAT[2][2] = VisionData.nZ;

//    roll    = asin(VISION_RMAT[2][1])*R2D;
//    yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//    pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//    ui->LB_TEMP->setText(QString().sprintf("X:%.4f, Y:%.4f, Z:%.4f\n NX:%.4f, NY:%.4f, NZ:%.4f\n Y:%.4f, R:%.4f, P:%.4f\n %.4f,%4f,%4f\n, %.4f,%4f,%4f\n, %.4f,%4f,%4f\n",
//                                           VisionData.mX,VisionData.mY,VisionData.mZ,
//                                           VisionData.nX,VisionData.nY,VisionData.nZ,
//                                           yaw,roll,pitch,
//                                           VISION_RMAT[0][0],VISION_RMAT[0][1],VISION_RMAT[0][2],VISION_RMAT[1][0],VISION_RMAT[1][1],VISION_RMAT[1][2],VISION_RMAT[2][0],VISION_RMAT[2][1],VISION_RMAT[2][2]));

//    if(visionInfo->dataReceived.size()>0)
//    {
//        visionInfo->DataReceived();
//    }
}



void WalkingDialog::on_BTN_HOME_POS_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_HOMEPOS;
    pLAN->SendCommand(cmd);
}
void WalkingDialog::on_BTN_WALK_READY_POS_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}
void WalkingDialog::on_BTN_INFINITY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

    pLAN->SendCommand(cmd);
}
void WalkingDialog::on_BTN_INFINITY_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

    pLAN->SendCommand(cmd);
}



void WalkingDialog::SetControl(int num){
    QString str;
    DSPTask task = DSPScheduler[num];
    ui->LE_LEFT_X->setText(str.sprintf("%.4f", task.Left[0]));
    ui->LE_LEFT_Y->setText(str.sprintf("%.4f", task.Left[1]));
    ui->LE_LEFT_Z->setText(str.sprintf("%.4f", task.Left[2]));
    ui->LE_LEFT_YAW->setText(str.sprintf("%.4f", task.LYaw));
    ui->LE_LEFT_ROLL->setText(str.sprintf("%.4f", task.LRoll));
    ui->LE_LEFT_PITCH->setText(str.sprintf("%.4f", task.LPitch));

    ui->LE_RIGHT_X->setText(str.sprintf("%.4f", task.Right[0]));
    ui->LE_RIGHT_Y->setText(str.sprintf("%.4f", task.Right[1]));
    ui->LE_RIGHT_Z->setText(str.sprintf("%.4f", task.Right[2]));
    ui->LE_RIGHT_YAW->setText(str.sprintf("%.4f", task.RYaw));
    ui->LE_RIGHT_ROLL->setText(str.sprintf("%.4f", task.RRoll));
    ui->LE_RIGHT_PITCH->setText(str.sprintf("%.4f", task.RPitch));

    ui->LE_HOLD_TIME->setText(str.sprintf("%.4f", task.HTime));
    //------------comz
    ui->LE_LEFT_COM_Z->setText(str.sprintf("%.4f",task.COMz[0]));
    ui->LE_RIGHT_COM_Z->setText(str.sprintf("%.4f",task.COMz[1]));

}
void WalkingDialog::SetTable(int row, DSPTask task){
    QString str;
    QTableWidgetItem *tempItem;

    ui->TW_DSP_SCHEDULER->setRowHeight(row, 25);
    tempItem = new QTableWidgetItem();
    tempItem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    tempItem->setFont(tableFont);
    tempItem->setText(str.sprintf("%d", row));
    ui->TW_DSP_SCHEDULER->setItem(row, 0, tempItem);

    tempItem = new QTableWidgetItem();
    tempItem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    tempItem->setFont(tableFont);
    tempItem->setText(str.sprintf("(%.4f, %.4f, %.4f) (%.4f, %.4f,%.4f)", task.Left[0], task.Left[1], task.Left[2], task.LYaw, task.LRoll, task.LPitch));
    ui->TW_DSP_SCHEDULER->setItem(row, 1, tempItem);

    tempItem = new QTableWidgetItem();
    tempItem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    tempItem->setFont(tableFont);
    tempItem->setText(str.sprintf("(%.4f, %.4f, %.4f) (%.4f, %.4f,%.4f)", task.Right[0], task.Right[1], task.Right[2], task.RYaw, task.RRoll, task.RPitch));
    ui->TW_DSP_SCHEDULER->setItem(row, 2, tempItem);

    tempItem = new QTableWidgetItem();
    tempItem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    tempItem->setFont(tableFont);
    tempItem->setText(str.sprintf("%.4f", task.HTime));
    ui->TW_DSP_SCHEDULER->setItem(row, 3, tempItem);

    //-------------------comz
    tempItem = new QTableWidgetItem();
    tempItem->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
    tempItem->setFont(tableFont);
    tempItem->setText(str.sprintf("%.4f, %.4f", task.COMz[0],task.COMz[1]));
    ui->TW_DSP_SCHEDULER->setItem(row, 4, tempItem);

}

void WalkingDialog::InsertTable(int row, DSPTask task){
    ui->TW_DSP_SCHEDULER->insertRow(row);
    SetTable(row, task);
    ChangeMaxSpinBox();
}
void WalkingDialog::DeleteTable(int row){
    ui->TW_DSP_SCHEDULER->removeRow(row);
    ui->TW_DSP_SCHEDULER->selectRow(row);
    ChangeMaxSpinBox();
}
void WalkingDialog::ChangeMaxSpinBox(){ui->SPIN_INDEX->setMaximum(DSPScheduler.size()-1);}
void WalkingDialog::on_TW_DSP_SCHEDULER_cellClicked(int row, int column){
    ui->SPIN_INDEX->setValue(row);
    ui->TW_DSP_SCHEDULER->selectRow(row);
}
void WalkingDialog::on_SPIN_INDEX_valueChanged(int arg1){
    if(arg1 < 0) return;
    ui->TW_DSP_SCHEDULER->selectRow(arg1);
    SetControl(arg1);
}

void WalkingDialog::on_BTN_ADD_clicked(){
    DSPTask task;
    int size = DSPScheduler.size();
    if(size == 0){
        task.Left[0] = ui->LE_CUR_LEFT_X->text().toDouble();
        task.Left[1] = ui->LE_CUR_LEFT_Y->text().toDouble();
        task.Left[2] = ui->LE_CUR_LEFT_Z->text().toDouble();
        task.LYaw = ui->LE_CUR_LEFT_YAW->text().toDouble();
        task.LRoll = ui->LE_CUR_LEFT_ROLL->text().toDouble();
        task.LPitch = ui->LE_CUR_LEFT_PITCH->text().toDouble();

        task.Right[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
        task.Right[1] = ui->LE_CUR_RIGHT_Y->text().toDouble();
        task.Right[2] = ui->LE_CUR_RIGHT_Z->text().toDouble();
        task.RYaw = ui->LE_CUR_RIGHT_YAW->text().toDouble();
        task.RRoll = ui->LE_CUR_RIGHT_ROLL->text().toDouble();
        task.RPitch = ui->LE_CUR_RIGHT_PITCH->text().toDouble();

        task.HTime = ui->LE_HOLD_TIME->text().toDouble();
        //---------------comz
//        task.COMz[0] = ui->LE_CUR_LEFT_COM_Z->text().toDouble();
//        task.COMz[1] = ui->LE_CUR_RIGHT_COM_Z->text().toDouble();
    }else{
        task.Left[0] = DSPScheduler[size-1].Left[0];
        task.Left[1] = DSPScheduler[size-1].Left[1];
        task.Left[2] = DSPScheduler[size-1].Left[2];
        task.LYaw = DSPScheduler[size-1].LYaw;
        task.LRoll = DSPScheduler[size-1].LRoll;
        task.LPitch = DSPScheduler[size-1].LPitch;

        task.Right[0] = DSPScheduler[size-1].Right[0];
        task.Right[1] = DSPScheduler[size-1].Right[1];
        task.Right[2] = DSPScheduler[size-1].Right[2];
        task.RYaw = DSPScheduler[size-1].RYaw;
        task.RRoll = DSPScheduler[size-1].RRoll;
        task.RPitch = DSPScheduler[size-1].RPitch;

        task.HTime = DSPScheduler[size-1].HTime;
        //--------------comz
        task.COMz[0] = DSPScheduler[size-1].COMz[0];
        task.COMz[1] = DSPScheduler[size-1].COMz[1];
    }
    DSPScheduler.push_back(task);
    InsertTable(ui->TW_DSP_SCHEDULER->rowCount(), task);

    on_TW_DSP_SCHEDULER_cellClicked(ui->TW_DSP_SCHEDULER->rowCount()-1, 0);
    Refresh();
}
void WalkingDialog::on_BTN_DELETE_clicked(){
    int row = ui->TW_DSP_SCHEDULER->currentRow();
    if(row <= 0)
        return;
    if(DSPScheduler.size() < 1)
        return;
    DSPScheduler.remove(row);
    DeleteTable(row);
    Refresh();
}
void WalkingDialog::Refresh(){
    int size = DSPScheduler.size();
    if(size == 0)
        return;

    for(int i=0; i<size; i++){
        SetTable(i, DSPScheduler[i]);
    }
    SetControl(ui->TW_DSP_SCHEDULER->currentRow());
}
void WalkingDialog::on_BTN_APPLY_clicked(){
    int pos = ui->SPIN_INDEX->value();
    DSPScheduler[pos].Left[0]   = ui->LE_LEFT_X->text().toDouble();
    DSPScheduler[pos].Left[1]   = ui->LE_LEFT_Y->text().toDouble();
    DSPScheduler[pos].Left[2]   = ui->LE_LEFT_Z->text().toDouble();
    DSPScheduler[pos].LYaw      = ui->LE_LEFT_YAW->text().toDouble();
    DSPScheduler[pos].LRoll     = ui->LE_LEFT_ROLL->text().toDouble();
    DSPScheduler[pos].LPitch    = ui->LE_LEFT_PITCH->text().toDouble();

    DSPScheduler[pos].Right[0]  = ui->LE_RIGHT_X->text().toDouble();
    DSPScheduler[pos].Right[1]  = ui->LE_RIGHT_Y->text().toDouble();
    DSPScheduler[pos].Right[2]  = ui->LE_RIGHT_Z->text().toDouble();
    DSPScheduler[pos].RYaw      = ui->LE_RIGHT_YAW->text().toDouble();
    DSPScheduler[pos].RRoll     = ui->LE_RIGHT_ROLL->text().toDouble();
    DSPScheduler[pos].RPitch    = ui->LE_RIGHT_PITCH->text().toDouble();

    DSPScheduler[pos].HTime = ui->LE_HOLD_TIME->text().toDouble();
    DSPScheduler[pos].COMz[0] = ui->LE_LEFT_COM_Z->text().toDouble();
    DSPScheduler[pos].COMz[1] = ui->LE_RIGHT_COM_Z->text().toDouble();
    Refresh();
}
void WalkingDialog::on_BTN_CLEAR_ALL_clicked(){
//    WriteConsole("Command: SENSOR_FOG_NULL");
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
//    cmd.COMMAND_DATA.USER_COMMAND = SENSOR_IMU_NULL;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
    pLAN->G2MData->StepTime = 1.f;
    for(int i=DSPScheduler.size(); i>=0; i--){
        DeleteTable(i);
    }
    DSPScheduler.clear();

    if(ui->RB_COMPLETE->isChecked() == false)
        pLAN->G2MData->WalkingStopModeCommand = SPREAD_STOP_WALKING;
    else
        pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
}

void WalkingDialog::on_BTN_FW_SAVE_clicked(){
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save DSP Schedule Data"), "", tr("DSPScheduleData (*.dsp);;AllFiles (*)"));

    if(fileName.isEmpty())
        return;
    else{
        int pointIndex;
        if((pointIndex = fileName.indexOf(".")) < 0)
            fileName += ".dsp";

        QFile file(fileName);
        if(!file.open(QIODevice::WriteOnly)){
            QMessageBox::information(this, tr("Unable to open file..!!"), file.errorString());
            return;
        }
        QDataStream out(&file);
        out.setVersion(QDataStream::Qt_5_1);

        QVector<SaveDSPScheduleData> SDSPVector;

        for(int i=0; i<DSPScheduler.size(); i++){
            SaveDSPScheduleData sdsp;
            sdsp.left[0] = DSPScheduler[i].Left[0];
            sdsp.left[1] = DSPScheduler[i].Left[1];
            sdsp.left[2] = DSPScheduler[i].Left[2];
            sdsp.lyaw = DSPScheduler[i].LYaw;
            sdsp.lroll = DSPScheduler[i].LRoll;
            sdsp.lpitch = DSPScheduler[i].LPitch;

            sdsp.right[0] = DSPScheduler[i].Right[0];
            sdsp.right[1] = DSPScheduler[i].Right[1];
            sdsp.right[2] = DSPScheduler[i].Right[2];
            sdsp.ryaw = DSPScheduler[i].RYaw;
            sdsp.rroll = DSPScheduler[i].RRoll;
            sdsp.rpitch = DSPScheduler[i].RPitch;

            sdsp.htime = DSPScheduler[i].HTime;
            sdsp.COMz[0] = DSPScheduler[i].COMz[0];
            sdsp.COMz[1] = DSPScheduler[i].COMz[1];
            SDSPVector.push_back(sdsp);
        }
        out << SDSPVector;

        file.close();
    }
}

void WalkingDialog::on_BTN_FW_LOAD_clicked(){
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open DSP Schedule Data"), "", tr("DSPScheduleData (*.dsp);;AllFiles (*)"));
    if(fileName.isEmpty())
        return;
    else{
        QFile file(fileName);
        if(!file.open(QIODevice::ReadOnly)){
            QMessageBox::information(this, tr("Unable to open file..!!"), file.errorString());
            return;
        }
        QDataStream in(&file);
        in.setVersion(QDataStream::Qt_5_1);

        QVector<SaveDSPScheduleData> SDSPVector;
        in >> SDSPVector;

        file.close();

        for(int i=DSPScheduler.size(); i>=0; i--){
            DeleteTable(i);
        }
        DSPScheduler.clear();
        for(int i=0; i<SDSPVector.size(); i++){
            DSPTask task;
            task.Left[0] = SDSPVector[i].left[0];
            task.Left[1] = SDSPVector[i].left[1];
            task.Left[2] = SDSPVector[i].left[2];
            task.LYaw = SDSPVector[i].lyaw;
            task.LRoll = SDSPVector[i].lroll;
            task.LPitch = SDSPVector[i].lpitch;

            task.Right[0] = SDSPVector[i].right[0];
            task.Right[1] = SDSPVector[i].right[1];
            task.Right[2] = SDSPVector[i].right[2];
            task.RYaw = SDSPVector[i].ryaw;
            task.RRoll = SDSPVector[i].rroll;
            task.RPitch = SDSPVector[i].rpitch;

            task.HTime = SDSPVector[i].htime;
            //--------------------comz
            task.COMz[0] = SDSPVector[i].COMz[0];
            task.COMz[1] = SDSPVector[i].COMz[1];
            DSPScheduler.push_back(task);
            ui->TW_DSP_SCHEDULER->insertRow(i);
            SetTable(i, task);
        }
        ui->TW_DSP_SCHEDULER->selectRow(0);
        ChangeMaxSpinBox();
        Refresh();
    }
}

void WalkingDialog::on_BTN_FW_SAVE_PLAY_clicked(){
    QString fileName = "Default.dsp";

    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly)){
        QMessageBox::information(this, tr("Unable to open file..!!"), file.errorString());
        return;
    }
    QDataStream out(&file);
    out.setVersion(QDataStream::Qt_5_1);

    QVector<SaveDSPScheduleData> SDSPVector;

    for(int i=0; i<DSPScheduler.size(); i++){
        SaveDSPScheduleData sdsp;
        sdsp.left[0] = DSPScheduler[i].Left[0];
        sdsp.left[1] = DSPScheduler[i].Left[1];
        sdsp.left[2] = DSPScheduler[i].Left[2];
        sdsp.lyaw = DSPScheduler[i].LYaw;
        sdsp.lroll = DSPScheduler[i].LRoll;
        sdsp.lpitch = DSPScheduler[i].LPitch;

        sdsp.right[0] = DSPScheduler[i].Right[0];
        sdsp.right[1] = DSPScheduler[i].Right[1];
        sdsp.right[2] = DSPScheduler[i].Right[2];
        sdsp.ryaw = DSPScheduler[i].RYaw;
        sdsp.rroll = DSPScheduler[i].RRoll;
        sdsp.rpitch = DSPScheduler[i].RPitch;

        sdsp.htime = DSPScheduler[i].HTime;
        //--------------------comz
        sdsp.COMz[0] = DSPScheduler[i].COMz[0];
        sdsp.COMz[1] = DSPScheduler[i].COMz[1];
        SDSPVector.push_back(sdsp);
    }
    out << SDSPVector;

    file.close();

    QFileInfo fileInfo;
    fileInfo.setFile(QDir().currentPath(), fileName);


//    if(ui->RB_task_terrain->isChecked() == true)
//    {// terrain
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = TERRAIN_WALKING;
//    }else if(ui->RB_task_ladder->isChecked() == true)
//    {// ladder
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = LADDER_WALKING;
//    }else if(ui->RB_task_normal_walking->isChecked() == true)
//    {// normal
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
//    }

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    // Send File Name (before 2014/10/30)
//    memset(cmd.COMMAND_DATA.USER_PARA_CHAR, 0, 100);
//    memcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, fileInfo.absoluteFilePath().toStdString().data(), fileInfo.absoluteFilePath().size());

    // Send All Data (after 2014/10/30)

    cmd.COMMAND_DATA.USER_PARA_INT[5] = SDSPVector.size();
    for(int i=0; i<SDSPVector.size(); i++){
        pLAN->G2MData->walkingDSP[i*14] = SDSPVector[i].left[0];
        pLAN->G2MData->walkingDSP[i*14+1] = SDSPVector[i].left[1];
        pLAN->G2MData->walkingDSP[i*14+2] = SDSPVector[i].left[2];
        pLAN->G2MData->walkingDSP[i*14+3] = SDSPVector[i].lyaw;
        pLAN->G2MData->walkingDSP[i*14+4] = SDSPVector[i].lroll;   //roll
        pLAN->G2MData->walkingDSP[i*14+5] = SDSPVector[i].lpitch;   //pitch

        pLAN->G2MData->walkingDSP[i*14+6] = SDSPVector[i].right[0];
        pLAN->G2MData->walkingDSP[i*14+7] = SDSPVector[i].right[1];
        pLAN->G2MData->walkingDSP[i*14+8] = SDSPVector[i].right[2];
        pLAN->G2MData->walkingDSP[i*14+9] = SDSPVector[i].ryaw;
        pLAN->G2MData->walkingDSP[i*14+10] = SDSPVector[i].rroll;   //roll
        pLAN->G2MData->walkingDSP[i*14+11] = SDSPVector[i].rpitch;   //pitch

        pLAN->G2MData->walkingDSP[i*14+12] = SDSPVector[i].COMz[0];
        pLAN->G2MData->walkingDSP[i*14+13] = 0;//SDSPVector[i].COMz[1];

//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14] = SDSPVector[i].left[0];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+1] = SDSPVector[i].left[1];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+2] = SDSPVector[i].left[2];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+3] = SDSPVector[i].lyaw;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+4] = SDSPVector[i].lroll;   //roll
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+5] = SDSPVector[i].lpitch;   //pitch

//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+6] = SDSPVector[i].right[0];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+7] = SDSPVector[i].right[1];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+8] = SDSPVector[i].right[2];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+9] = SDSPVector[i].ryaw;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+10] = SDSPVector[i].rroll;   //roll
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+11] = SDSPVector[i].rpitch;   //pitch

//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+12] = SDSPVector[i].COMz[0];
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[i*14+13] = 0;//SDSPVector[i].COMz[1];
    }
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
    QString str;
    str.sprintf("%s", cmd.COMMAND_DATA.USER_PARA_CHAR);
    cout << str.toStdString().data() << endl;


}

void WalkingDialog::on_BTN_FW_PLAY_SAVED_clicked(){
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open DSP Schedule Data"), "", tr("DSPScheduleData (*.dsp);;AllFiles (*)"));
    USER_COMMAND cmd;
    if(fileName.isEmpty())
        return;
    else{
        QFile file(fileName);
        if(!file.open(QIODevice::ReadOnly)){
            QMessageBox::information(this, tr("Unable to open file..!!"), file.errorString());
            return;
        }
        QVector<SaveDSPScheduleData> SDSPVector;
         QDataStream in(&file);
         in.setVersion(QDataStream::Qt_5_1);
         in >> SDSPVector;
         file.close();

         cmd.COMMAND_DATA.USER_PARA_INT[9] = TERRAIN_WALKING;

         cmd.COMMAND_DATA.USER_PARA_INT[5] = SDSPVector.size();
         for(int i=0; i<SDSPVector.size(); i++){
             pLAN->G2MData->walkingDSP[i*14] = SDSPVector[i].left[0];
             pLAN->G2MData->walkingDSP[i*14+1] = SDSPVector[i].left[1];
             pLAN->G2MData->walkingDSP[i*14+2] = SDSPVector[i].left[2];
             pLAN->G2MData->walkingDSP[i*14+3] = SDSPVector[i].lyaw;
             pLAN->G2MData->walkingDSP[i*14+4] = SDSPVector[i].lroll;   //roll
             pLAN->G2MData->walkingDSP[i*14+5] = SDSPVector[i].lpitch;   //pitch

             pLAN->G2MData->walkingDSP[i*14+6] = SDSPVector[i].right[0];
             pLAN->G2MData->walkingDSP[i*14+7] = SDSPVector[i].right[1];
             pLAN->G2MData->walkingDSP[i*14+8] = SDSPVector[i].right[2];
             pLAN->G2MData->walkingDSP[i*14+9] = SDSPVector[i].ryaw;
             pLAN->G2MData->walkingDSP[i*14+10] = SDSPVector[i].rroll;   //roll
             pLAN->G2MData->walkingDSP[i*14+11] = SDSPVector[i].rpitch;   //pitch

             pLAN->G2MData->walkingDSP[i*14+12] = SDSPVector[i].COMz[0];
             pLAN->G2MData->walkingDSP[i*14+13] = 0;//SDSPVector[i].COMz[1];

         }

//        if(ui->RB_task_terrain->isChecked() == true)
//        {
//            cmd.COMMAND_DATA.USER_PARA_INT[0] = 100;

//        }else if(ui->RB_task_ladder->isChecked() == true)
//        {
//            cmd.COMMAND_DATA.USER_PARA_INT[0] = 101;
//        }else
//        {
//            cmd.COMMAND_DATA.USER_PARA_INT[0] = 102;
//        }

//        memset(cmd.COMMAND_DATA.USER_PARA_CHAR, 0, 100);
//        memcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, fileName.toStdString().data(), fileName.size());
        cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
        cmd.COMMAND_TARGET = alNumFreeWalking;
        pLAN->SendCommand(cmd);
    }
}



int WalkingDialog::FootSelector(int direction){
    double errTh = 0.00001;
    int pos = DSPScheduler.size()-1;
    double lx = DSPScheduler[pos].Left[0];
    double ly = DSPScheduler[pos].Left[1];
    double rx = DSPScheduler[pos].Right[0];
    double ry = DSPScheduler[pos].Right[1];
    double lth = DSPScheduler[pos].LYaw;
    double rth = DSPScheduler[pos].RYaw;

    double theta = (DSPScheduler[pos].LYaw+DSPScheduler[pos].RYaw)/2.;
    double lx_n,lx_nn,ly_n,ly_nn;
    double rx_n,rx_nn,ry_n,ry_nn;

    lx_n = lx-(rx+lx)/2.;
    ly_n = ly-(ry+ly)/2.;
    rx_n = rx-(rx+lx)/2.;
    ry_n = ry-(ry+ly)/2.;

    lx_nn =  lx_n*cos(theta*D2R) + ly_n*sin(theta*D2R);
    ly_nn = -lx_n*sin(theta*D2R) + ly_n*cos(theta*D2R);

    rx_nn =  rx_n*cos(theta*D2R) + ry_n*sin(theta*D2R);
    ry_nn = -rx_n*sin(theta*D2R) + ry_n*cos(theta*D2R);

    if(direction == DIR_FORWARD){
        if(fabs(lx_nn-rx_nn) < errTh)
            return FOOT_RIGHT;
        if(lx_nn > rx_nn)
            return FOOT_RIGHT;
        return FOOT_LEFT;
    }else if(direction == DIR_BACKWARD){
        if(fabs(lx_nn-rx_nn) < errTh)
            return FOOT_RIGHT;
        if(lx_nn > rx_nn)
            return FOOT_LEFT;
        return FOOT_RIGHT;
    }else if(direction == DIR_RIGHT){
//        if(fabs(fabs(ly_nn-ry_nn)-L_PEL2PEL) > 0.01){
//            return FOOT_LEFT;
//        }else{
            return FOOT_RIGHT;
//        }
    }else if(direction == DIR_LEFT){
//        if(fabs(fabs(ly_nn-ry_nn)-L_PEL2PEL) > 0.01){
//            return FOOT_RIGHT;
//        }else{
            return FOOT_LEFT;
//        }
    }else if(direction == DIR_CW){
        if(lth!=rth)
            return FOOT_LEFT;
        else
            return FOOT_RIGHT;
    }else if(direction == DIR_CCW){
        if(lth!=rth)
            return FOOT_RIGHT;
        else
            return FOOT_LEFT;
    }
}

void WalkingDialog::on_BTN_INT_FORWARD_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = FORWARD_WALKING;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;

    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;

    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_BACKWARD_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = BACKWARD_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;

    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;

    pLAN->SendCommand(cmd);

}

void WalkingDialog::on_BTN_INT_RIGHT_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = RIGHTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;

    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_LEFT_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = LEFTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;
    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}



void WalkingDialog::on_BTN_DATA_SAVE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_SAVE;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_DATA_SAVE_START_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_SAVE;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}



void WalkingDialog::on_BTN_INT_CW_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = CWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;
    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_CCW_clicked(){
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = CCWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
    if(pLAN->G2MData->StepNum>30) return;
    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}




void WalkingDialog::on_BTN_Control_On_clicked()
{
//    cmd.COMMAND_DATA.USER_PARA_INT[3] = 1;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_P_GAIN->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_D_GAIN->text().toFloat();

    cmd.COMMAND_DATA.USER_PARA_FLOAT[2] = ui->LE_P_GAIN_2->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[3] = ui->LE_D_GAIN_2->text().toFloat();

    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_CONTROL_TEST;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_Control_Off_clicked()
{
//    cmd.COMMAND_DATA.USER_PARA_INT[3] = 0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_CONTROL_TEST;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_FW_GO_clicked()
{

//    WriteConsole("Command: SENSOR_FOG_NULL");
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
//    cmd.COMMAND_DATA.USER_COMMAND = SENSOR_IMU_NULL;
//    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
//    pLAN->SendCommand(cmd);
    USER_COMMAND cmd;
    if(ui->CB_OUTSIDE->isChecked())
        cmd.COMMAND_DATA.USER_PARA_INT[4] = OUTSIDE_WALKING;
    else
        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    int r = 1;

    int temp_step_num = ui->LE_FW_STEP_NUM->text().toInt();

    cmd.COMMAND_DATA.USER_PARA_INT[3] = GUI_TERRAIN;

    cmd.COMMAND_DATA.USER_PARA_INT[5] = temp_step_num;
    //===============================1st DSP========================================
    pLAN->G2MData->walkingDSP[0] = ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble();
    pLAN->G2MData->walkingDSP[1] = ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble();
    pLAN->G2MData->walkingDSP[2] = ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble();
    pLAN->G2MData->walkingDSP[3] = ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble();
    pLAN->G2MData->walkingDSP[4] = ui->LE_STEP_LENGTH_LEFT_ROLL_1->text().toDouble();   //roll
    pLAN->G2MData->walkingDSP[5] = ui->LE_STEP_LENGTH_LEFT_PITCH_1->text().toDouble();   //pitch

    pLAN->G2MData->walkingDSP[6] = ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble();
    pLAN->G2MData->walkingDSP[7] = ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble();
    pLAN->G2MData->walkingDSP[8] = ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble();
    pLAN->G2MData->walkingDSP[9] = ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble();
    pLAN->G2MData->walkingDSP[10] = ui->LE_STEP_LENGTH_RIGHT_ROLL_1->text().toDouble();   //roll
    pLAN->G2MData->walkingDSP[11] = ui->LE_STEP_LENGTH_RIGHT_PITCH_1->text().toDouble();   //pitch

    pLAN->G2MData->walkingDSP[12] = ui->LE_STEP_LEFT_COM_Z1->text().toDouble();
    pLAN->G2MData->walkingDSP[13] = ui->LE_STEP_LEFT_COM_Z1_2->text().toDouble();
    pLAN->G2MData->walkingDSP[13+r] = ui->LE_STEP_LEFT_COM_Z1->text().toDouble();
    //===============================2nd DSP===============================
    pLAN->G2MData->walkingDSP[14+r] = ui->LE_STEP_LENGTH_LEFT_X2->text().toDouble();
    pLAN->G2MData->walkingDSP[15+r] = ui->LE_STEP_LENGTH_LEFT_Y2->text().toDouble();
    pLAN->G2MData->walkingDSP[16+r] = ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble();
    pLAN->G2MData->walkingDSP[17+r] = ui->LE_STEP_LENGTH_LEFT_YAW_2->text().toDouble();
    pLAN->G2MData->walkingDSP[18+r] = ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble();
    pLAN->G2MData->walkingDSP[19+r] = ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble();

    pLAN->G2MData->walkingDSP[20+r] = ui->LE_STEP_LENGTH_RIGHT_X2->text().toDouble();
    pLAN->G2MData->walkingDSP[21+r] = ui->LE_STEP_LENGTH_RIGHT_Y2->text().toDouble();
    pLAN->G2MData->walkingDSP[22+r] = ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble();
    pLAN->G2MData->walkingDSP[23+r] = ui->LE_STEP_LENGTH_RIGHT_YAW_2->text().toDouble();
    pLAN->G2MData->walkingDSP[24+r] = ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble();
    pLAN->G2MData->walkingDSP[25+r] = ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble();

    pLAN->G2MData->walkingDSP[26+r] = ui->LE_STEP_LEFT_COM_Z2->text().toDouble();
    pLAN->G2MData->walkingDSP[27+r] = ui->LE_STEP_LEFT_COM_Z2_2->text().toDouble();
    pLAN->G2MData->walkingDSP[28+r] = ui->LE_STEP_LEFT_COM_Z1_2->text().toDouble() ;//+ (double)0.03f;
    //===============================3rd DSP===============================
    r = 2;
    pLAN->G2MData->walkingDSP[28+r] = ui->LE_STEP_LENGTH_LEFT_X3->text().toDouble();
    pLAN->G2MData->walkingDSP[29+r] = ui->LE_STEP_LENGTH_LEFT_Y3->text().toDouble();
    pLAN->G2MData->walkingDSP[30+r] = ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble();
    pLAN->G2MData->walkingDSP[31+r] = ui->LE_STEP_LENGTH_LEFT_YAW_3->text().toDouble();
    pLAN->G2MData->walkingDSP[32+r] =  ui->LE_STEP_LENGTH_LEFT_ROLL_3->text().toDouble();
    pLAN->G2MData->walkingDSP[33+r] = ui->LE_STEP_LENGTH_LEFT_PITCH_3->text().toDouble();

    pLAN->G2MData->walkingDSP[34+r] = ui->LE_STEP_LENGTH_RIGHT_X3->text().toDouble();
    pLAN->G2MData->walkingDSP[35+r] = ui->LE_STEP_LENGTH_RIGHT_Y3->text().toDouble();
    pLAN->G2MData->walkingDSP[36+r] = ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble();
    pLAN->G2MData->walkingDSP[37+r] = ui->LE_STEP_LENGTH_RIGHT_YAW_3->text().toDouble();
    pLAN->G2MData->walkingDSP[38+r] =  ui->LE_STEP_LENGTH_RIGHT_ROLL_3->text().toDouble();
    pLAN->G2MData->walkingDSP[39+r] = ui->LE_STEP_LENGTH_RIGHT_PITCH_3->text().toDouble();

    pLAN->G2MData->walkingDSP[40+r] = ui->LE_STEP_LEFT_COM_Z3->text().toDouble();
    pLAN->G2MData->walkingDSP[41+r] = ui->LE_STEP_LEFT_COM_Z3_2->text().toDouble();
    pLAN->G2MData->walkingDSP[42+r] = ui->LE_STEP_LEFT_COM_Z2_2->text().toDouble();//+ (double)0.03f;
//    //===============================4th DSP===============================
    r =3;
    pLAN->G2MData->walkingDSP[42+r] = ui->LE_STEP_LENGTH_LEFT_X4->text().toDouble();
    pLAN->G2MData->walkingDSP[43+r] = ui->LE_STEP_LENGTH_LEFT_Y4->text().toDouble();
    pLAN->G2MData->walkingDSP[44+r] = ui->LE_STEP_LENGTH_LEFT_Z4->text().toDouble();
    pLAN->G2MData->walkingDSP[45+r] = ui->LE_STEP_LENGTH_LEFT_YAW_4->text().toDouble();
    pLAN->G2MData->walkingDSP[46+r] =  ui->LE_STEP_LENGTH_LEFT_ROLL_4->text().toDouble();
    pLAN->G2MData->walkingDSP[47+r] = ui->LE_STEP_LENGTH_LEFT_PITCH_4->text().toDouble();

    pLAN->G2MData->walkingDSP[48+r] = ui->LE_STEP_LENGTH_RIGHT_X4->text().toDouble();
    pLAN->G2MData->walkingDSP[49+r] = ui->LE_STEP_LENGTH_RIGHT_Y4->text().toDouble();
    pLAN->G2MData->walkingDSP[50+r] = ui->LE_STEP_LENGTH_RIGHT_Z4->text().toDouble();
    pLAN->G2MData->walkingDSP[51+r] = ui->LE_STEP_LENGTH_RIGHT_YAW_4->text().toDouble();
    pLAN->G2MData->walkingDSP[52+r] = ui->LE_STEP_LENGTH_RIGHT_ROLL_4->text().toDouble();
    pLAN->G2MData->walkingDSP[53+r] =ui->LE_STEP_LENGTH_RIGHT_PITCH_4->text().toDouble();

    pLAN->G2MData->walkingDSP[54+r] = ui->LE_STEP_LEFT_COM_Z4->text().toDouble();
    pLAN->G2MData->walkingDSP[55+r] = ui->LE_STEP_LEFT_COM_Z4_2->text().toDouble();
    pLAN->G2MData->walkingDSP[56+r] = ui->LE_STEP_LEFT_COM_Z3_2->text().toDouble();//+ (double)0.03f;
//    //===============================5th DSP===============================
//    pLAN->G2MData->walkingDSP[56] = ui->LE_STEP_LENGTH_LEFT_X5->text().toDouble();
//    pLAN->G2MData->walkingDSP[57] = ui->LE_STEP_LENGTH_LEFT_Y5->text().toDouble();
//    pLAN->G2MData->walkingDSP[58] = ui->LE_STEP_LENGTH_LEFT_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[59] = ui->LE_STEP_LENGTH_LEFT_YAW_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[60] =  ui->LE_STEP_LENGTH_LEFT_ROLL_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[61] =  ui->LE_STEP_LENGTH_LEFT_PITCH_5->text().toDouble();

//    pLAN->G2MData->walkingDSP[62] = ui->LE_STEP_LENGTH_RIGHT_X5->text().toDouble();
//    pLAN->G2MData->walkingDSP[63] = ui->LE_STEP_LENGTH_RIGHT_Y5->text().toDouble();
//    pLAN->G2MData->walkingDSP[64] = ui->LE_STEP_LENGTH_RIGHT_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[65] = ui->LE_STEP_LENGTH_RIGHT_YAW_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[66] = ui->LE_STEP_LENGTH_RIGHT_ROLL_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[67] = ui->LE_STEP_LENGTH_RIGHT_PITCH_5->text().toDouble();

//    pLAN->G2MData->walkingDSP[68] = ui->LE_STEP_LEFT_COM_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[69] = 0.;
//    //===============================6th DSP===============================
//    pLAN->G2MData->walkingDSP[70] = ui->LE_STEP_LENGTH_LEFT_X6->text().toDouble();
//    pLAN->G2MData->walkingDSP[71] = ui->LE_STEP_LENGTH_LEFT_Y6->text().toDouble();
//    pLAN->G2MData->walkingDSP[72] = ui->LE_STEP_LENGTH_LEFT_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[73] = ui->LE_STEP_LENGTH_LEFT_YAW_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[74] =  ui->LE_STEP_LENGTH_LEFT_ROLL_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[75] =  ui->LE_STEP_LENGTH_LEFT_PITCH_6->text().toDouble();

//    pLAN->G2MData->walkingDSP[76] = ui->LE_STEP_LENGTH_RIGHT_X6->text().toDouble();
//    pLAN->G2MData->walkingDSP[77] = ui->LE_STEP_LENGTH_RIGHT_Y6->text().toDouble();
//    pLAN->G2MData->walkingDSP[78] = ui->LE_STEP_LENGTH_RIGHT_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[79] = ui->LE_STEP_LENGTH_RIGHT_YAW_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[80] = ui->LE_STEP_LENGTH_RIGHT_ROLL_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[81] = ui->LE_STEP_LENGTH_RIGHT_PITCH_6->text().toDouble();

//    pLAN->G2MData->walkingDSP[82] = ui->LE_STEP_LEFT_COM_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[83] = 0.;
//    //===============================7th DSP===============================
//    pLAN->G2MData->walkingDSP[84] = ui->LE_STEP_LENGTH_LEFT_X7->text().toDouble();
//    pLAN->G2MData->walkingDSP[85] = ui->LE_STEP_LENGTH_LEFT_Y7->text().toDouble();
//    pLAN->G2MData->walkingDSP[86] = ui->LE_STEP_LENGTH_LEFT_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[87] = ui->LE_STEP_LENGTH_LEFT_YAW_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[88] =  ui->LE_STEP_LENGTH_LEFT_ROLL_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[89] =  ui->LE_STEP_LENGTH_LEFT_PITCH_7->text().toDouble();

//    pLAN->G2MData->walkingDSP[90] = ui->LE_STEP_LENGTH_RIGHT_X7->text().toDouble();
//    pLAN->G2MData->walkingDSP[91] = ui->LE_STEP_LENGTH_RIGHT_Y7->text().toDouble();
//    pLAN->G2MData->walkingDSP[92] = ui->LE_STEP_LENGTH_RIGHT_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[93] = ui->LE_STEP_LENGTH_RIGHT_YAW_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[94] = ui->LE_STEP_LENGTH_RIGHT_ROLL_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[95] = ui->LE_STEP_LENGTH_RIGHT_PITCH_7->text().toDouble();

//    pLAN->G2MData->walkingDSP[96] = ui->LE_STEP_LEFT_COM_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[97] = 0.;
//    //===============================8th DSP===============================
//    pLAN->G2MData->walkingDSP[98] = ui->LE_STEP_LENGTH_LEFT_X8->text().toDouble();
//    pLAN->G2MData->walkingDSP[99] = ui->LE_STEP_LENGTH_LEFT_Y8->text().toDouble();
//    pLAN->G2MData->walkingDSP[100] = ui->LE_STEP_LENGTH_LEFT_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[101] = ui->LE_STEP_LENGTH_LEFT_YAW_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[102] =  ui->LE_STEP_LENGTH_LEFT_ROLL_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[103] =  ui->LE_STEP_LENGTH_LEFT_PITCH_8->text().toDouble();

//    pLAN->G2MData->walkingDSP[104] = ui->LE_STEP_LENGTH_RIGHT_X8->text().toDouble();
//    pLAN->G2MData->walkingDSP[105] = ui->LE_STEP_LENGTH_RIGHT_Y8->text().toDouble();
//    pLAN->G2MData->walkingDSP[106] = ui->LE_STEP_LENGTH_RIGHT_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[107] = ui->LE_STEP_LENGTH_RIGHT_YAW_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[108] = ui->LE_STEP_LENGTH_RIGHT_ROLL_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[109] = ui->LE_STEP_LENGTH_RIGHT_PITCH_8->text().toDouble();

//    pLAN->G2MData->walkingDSP[110] = ui->LE_STEP_LEFT_COM_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[111] = 0.;

//    if(ui->RB_task_terrain->isChecked() == true)
//    {// terrain
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = TERRAIN_WALKING;
//    }

            cmd.COMMAND_DATA.USER_PARA_INT[9] = TERRAIN_WALKING;
//            cmd.COMMAND_DATA.USER_PARA_INT[9] = LADDER_WALKING;

//    else if(ui->RB_task_ladder->isChecked() == true)
//    {// ladder
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = LADDER_WALKING;
//    }else if(ui->RB_task_normal_walking->isChecked() == true)
//    {// normal
//        cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
//    }

    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_TERRAIN;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
    QString str;
    str.sprintf("%s", cmd.COMMAND_DATA.USER_PARA_CHAR);
    cout << str.toStdString().data() << endl;
}

void WalkingDialog::on_BTN_FILL_1ST_DSP_clicked()
{
    QString str;
    ui->LE_STEP_LENGTH_LEFT_X1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_X->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_Y->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Z1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_Z->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_YAW_1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_YAW->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_ROLL_1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_ROLL->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_PITCH_1->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_PITCH->text().toDouble()));

    ui->LE_STEP_LENGTH_RIGHT_X1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_X->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Y1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_Y->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Z1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_Z->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_YAW_1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_YAW->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_ROLL->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_1->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_PITCH->text().toDouble()));

    ui->LE_STEP_LEFT_COM_Z1->setText(str.sprintf("%.4f", ui->LE_CUR_COM_Z_OFFSET->text().toDouble()));

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_X->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Y2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_Y->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_Z->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_YAW->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_ROLL->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f", ui->LE_CUR_LEFT_PITCH->text().toDouble()));

    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_X->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_Y->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_Z->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_YAW->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_ROLL->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f", ui->LE_CUR_RIGHT_PITCH->text().toDouble()));

    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f", ui->LE_CUR_COM_Z_OFFSET->text().toDouble()));

}
void WalkingDialog::on_BTN_FILL_1ST_DSP_3_clicked()
{
    QString str;
    ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_X2->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_Y2->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_YAW_2->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble()));
    ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble()));

    ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_X2->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Y2->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_YAW_2->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble()));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble()));

    ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LEFT_COM_Z2->text().toDouble()));

}
//void WalkingDialog::on_BTN_FILL_DSP3_LAST()
//{
//    QString str;
//    ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_X2->text().toDouble()));
//    ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_Y2->text().toDouble()));
//    ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble()));
//    ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_YAW_2->text().toDouble()));
//    ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble()));
//    ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble()));

//    ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_X2->text().toDouble()));
//    ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Y2->text().toDouble()));
//    ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble()));
//    ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_YAW_2->text().toDouble()));
//    ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble()));
//    ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble()));

//    ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f", ui->LE_STEP_LEFT_COM_Z2->text().toDouble()));

//}


void WalkingDialog::on_BTN_FILL_UP_STAIR_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0] + L;      lx[1] = lx[0];
    ry[1] = ry[0];          ly[1] = ly[0];
    rz[1] = rz[0];          lz[1] = lz[0];
    com_z[1] = 0.0;

    rx[2] = rx[1];          lx[2] = lx[1] + L;
    ry[2] = ry[1];          ly[2] = ly[1];
    rz[2] = rz[1];          lz[2] = lz[1] + H;
    com_z[2] = 0.05;

    rx[3] = rx[2] + L;      lx[3] = lx[2];
    ry[3] = ry[2];          ly[3] = ly[2];
    rz[3] = rz[2] + H;      lz[3] = lz[2];
    com_z[3] = 0.0;

    rx[4] = rx[3];          lx[4] = lx[3] + L;
    ry[4] = ry[3];          ly[4] = ly[3];
    rz[4] = rz[3];          lz[4] = lz[3] + H;
    com_z[4] = 0.05;

    rx[5] = rx[4] + L;      lx[5] = lx[4];
    ry[5] = ry[4];          ly[5] = ly[4];
    rz[5] = rz[4] + H;      lz[5] = lz[4];
    com_z[5] = 0.00;
    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}


void WalkingDialog::on_BTN_FILL_DN_STAIR_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = lx[0] + L;
    ry[1] = ry[0];          ly[1] = ly[0];
    rz[1] = rz[0];          lz[1] = lz[0] - H;
    com_z[1] = -H;

    rx[2] = rx[1] + L;      lx[2] = lx[1];
    ry[2] = ry[1];          ly[2] = ly[1];
    rz[2] = rz[1] - H;      lz[2] = lz[1];
    com_z[2] = 0.0;

    rx[3] = rx[2];          lx[3] = lx[2] + L;
    ry[3] = ry[2];          ly[3] = ly[2];
    rz[3] = rz[2];          lz[3] = lz[2] - H;
    com_z[3] = -H;

    rx[4] = rx[3] + L;      lx[4] = lx[3];
    ry[4] = ry[3];          ly[4] = ly[3];
    rz[4] = rz[3] - H;      lz[4] = lz[3];
    com_z[4] = 0.0;

    rx[5] = rx[4];          lx[5] = lx[4] + L;
    ry[5] = ry[4];          ly[5] = ly[4];
    rz[5] = rz[4];          lz[5] = lz[4];
    com_z[5] = 0.00;
    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_UP_STAIR_2_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = lx[0] - L;
    ry[1] = ry[0];          ly[1] = ly[0];
    rz[1] = rz[0];          lz[1] = lz[0];
    com_z[1] = 0.0;

    rx[2] = rx[1] -L;       lx[2] = lx[1];
    ry[2] = ry[1];          ly[2] = ly[1];
    rz[2] = rz[1] + H;      lz[2] = lz[1];
    com_z[2] = 0.05;

    rx[3] = rx[2];          lx[3] = lx[2] - L;
    ry[3] = ry[2];          ly[3] = ly[2];
    rz[3] = rz[2];          lz[3] = lz[2] + H;
    com_z[3] = 0.0;

    rx[4] = rx[3] - L;      lx[4] = lx[3];
    ry[4] = ry[3];          ly[4] = ly[3];
    rz[4] = rz[3] + H;      lz[4] = lz[3];
    com_z[4] = 0.05;

    rx[5] = rx[4];          lx[5] = lx[4] - L;
    ry[5] = ry[4];          ly[5] = ly[4];
    rz[5] = rz[4];          lz[5] = lz[4] + H;
    com_z[5] = 0.00;
    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_DN_STAIR_2_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0] - L;      lx[1] = lx[0];
    ry[1] = ry[0];          ly[1] = ly[0];
    rz[1] = rz[0] - H;      lz[1] = lz[0];
    com_z[1] = -H;

    rx[2] = rx[1];          lx[2] = lx[1] - L;
    ry[2] = ry[1];          ly[2] = ly[1];
    rz[2] = rz[1];          lz[2] = lz[1] - H;
    com_z[2] = 0.0;

    rx[3] = rx[2] - L;      lx[3] = lx[2];
    ry[3] = ry[2];          ly[3] = ly[2];
    rz[3] = rz[2] - H;      lz[3] = lz[2];
    com_z[3] = -H;

    rx[4] = rx[3];          lx[4] = lx[3] - L;
    ry[4] = ry[3];          ly[4] = ly[3];
    rz[4] = rz[3];          lz[4] = lz[3] -H;
    com_z[4] = 0.0;

    rx[5] = rx[4] - L;      lx[5] = lx[4];
    ry[5] = ry[4];          ly[5] = ly[4];
    rz[5] = rz[4];          lz[5] = lz[4];
    com_z[5] = 0.00;

    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}
void WalkingDialog::fill_blanks(double rx[6],double ry[6],double rz[6],double lx[6],double ly[6],double lz[6],double com_z[6],double ryaw[6],double lyaw[6],double rroll[6],double lroll[6],double rpitch[6],double lpitch[6])
{
    QString str;
    ui->LE_STEP_LENGTH_RIGHT_X1->setText(str.sprintf("%.4f", rx[0]));       ui->LE_STEP_LENGTH_LEFT_X1->setText(str.sprintf("%.4f", lx[0]));
    ui->LE_STEP_LENGTH_RIGHT_Y1->setText(str.sprintf("%.4f", ry[0]));       ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ly[0]));
    ui->LE_STEP_LENGTH_RIGHT_Z1->setText(str.sprintf("%.4f", rz[0]));       ui->LE_STEP_LENGTH_LEFT_Z1->setText(str.sprintf("%.4f", lz[0]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_1->setText(str.sprintf("%.4f", ryaw[0]));     ui->LE_STEP_LENGTH_LEFT_YAW_1->setText(str.sprintf("%.4f", lyaw[0]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_1->setText(str.sprintf("%.4f", rroll[0]));     ui->LE_STEP_LENGTH_LEFT_ROLL_1->setText(str.sprintf("%.4f", lroll[0]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_1->setText(str.sprintf("%.4f", rpitch[0]));     ui->LE_STEP_LENGTH_LEFT_PITCH_1->setText(str.sprintf("%.4f", lpitch[0]));
    ui->LE_STEP_LEFT_COM_Z1->setText(str.sprintf("%.4f", com_z[0]));

    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", rx[1]));       ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", lx[1]));
    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", ry[1]));       ui->LE_STEP_LENGTH_LEFT_Y2->setText(str.sprintf("%.4f", ly[1]));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", rz[1]));       ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", lz[1]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f", ryaw[1]));     ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f", lyaw[1]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f", rroll[1]));     ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f", lroll[1]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f", rpitch[1]));     ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f", lpitch[1]));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f", com_z[1]));

    ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", rx[2]));       ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", lx[2]));
    ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", ry[2]));       ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", ly[2]));
    ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", rz[2]));       ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", lz[2]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f", ryaw[2]));     ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f", lyaw[2]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f", rroll[2]));     ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f", lroll[2]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f", rpitch[2]));     ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f", lpitch[2]));
    ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f", com_z[2]));

    ui->LE_STEP_LENGTH_RIGHT_X4->setText(str.sprintf("%.4f", rx[3]));       ui->LE_STEP_LENGTH_LEFT_X4->setText(str.sprintf("%.4f", lx[3]));
    ui->LE_STEP_LENGTH_RIGHT_Y4->setText(str.sprintf("%.4f", ry[3]));       ui->LE_STEP_LENGTH_LEFT_Y4->setText(str.sprintf("%.4f", ly[3]));
    ui->LE_STEP_LENGTH_RIGHT_Z4->setText(str.sprintf("%.4f", rz[3]));       ui->LE_STEP_LENGTH_LEFT_Z4->setText(str.sprintf("%.4f", lz[3]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_4->setText(str.sprintf("%.4f", ryaw[3]));     ui->LE_STEP_LENGTH_LEFT_YAW_4->setText(str.sprintf("%.4f", lyaw[3]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_4->setText(str.sprintf("%.4f", rroll[3]));     ui->LE_STEP_LENGTH_LEFT_ROLL_4->setText(str.sprintf("%.4f", lroll[3]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_4->setText(str.sprintf("%.4f", rpitch[3]));     ui->LE_STEP_LENGTH_LEFT_PITCH_4->setText(str.sprintf("%.4f", lpitch[3]));
    ui->LE_STEP_LEFT_COM_Z4->setText(str.sprintf("%.4f", com_z[3]));

    ui->LE_STEP_LENGTH_RIGHT_X5->setText(str.sprintf("%.4f", rx[4]));       ui->LE_STEP_LENGTH_LEFT_X5->setText(str.sprintf("%.4f", lx[4]));
    ui->LE_STEP_LENGTH_RIGHT_Y5->setText(str.sprintf("%.4f", ry[4]));       ui->LE_STEP_LENGTH_LEFT_Y5->setText(str.sprintf("%.4f", ly[4]));
    ui->LE_STEP_LENGTH_RIGHT_Z5->setText(str.sprintf("%.4f", rz[4]));       ui->LE_STEP_LENGTH_LEFT_Z5->setText(str.sprintf("%.4f", lz[4]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_5->setText(str.sprintf("%.4f", ryaw[4]));     ui->LE_STEP_LENGTH_LEFT_YAW_5->setText(str.sprintf("%.4f", lyaw[4]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_5->setText(str.sprintf("%.4f", rroll[4]));     ui->LE_STEP_LENGTH_LEFT_ROLL_5->setText(str.sprintf("%.4f", lroll[4]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_5->setText(str.sprintf("%.4f", rpitch[4]));     ui->LE_STEP_LENGTH_LEFT_PITCH_5->setText(str.sprintf("%.4f", lpitch[4]));
    ui->LE_STEP_LEFT_COM_Z5->setText(str.sprintf("%.4f", com_z[4]));

    ui->LE_STEP_LENGTH_RIGHT_X6->setText(str.sprintf("%.4f", rx[5]));       ui->LE_STEP_LENGTH_LEFT_X6->setText(str.sprintf("%.4f", lx[5]));
    ui->LE_STEP_LENGTH_RIGHT_Y6->setText(str.sprintf("%.4f", ry[5]));       ui->LE_STEP_LENGTH_LEFT_Y6->setText(str.sprintf("%.4f", ly[5]));
    ui->LE_STEP_LENGTH_RIGHT_Z6->setText(str.sprintf("%.4f", rz[5]));       ui->LE_STEP_LENGTH_LEFT_Z6->setText(str.sprintf("%.4f", lz[5]));
    ui->LE_STEP_LENGTH_RIGHT_YAW_6->setText(str.sprintf("%.4f", ryaw[5]));     ui->LE_STEP_LENGTH_LEFT_YAW_6->setText(str.sprintf("%.4f", lyaw[5]));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_6->setText(str.sprintf("%.4f", rroll[5]));     ui->LE_STEP_LENGTH_LEFT_ROLL_6->setText(str.sprintf("%.4f", lroll[5]));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_6->setText(str.sprintf("%.4f", rpitch[5]));     ui->LE_STEP_LENGTH_LEFT_PITCH_6->setText(str.sprintf("%.4f", lpitch[5]));
    ui->LE_STEP_LEFT_COM_Z6->setText(str.sprintf("%.4f", com_z[5]));
}

void WalkingDialog::on_BTN_FILL_UP_STAIR_3_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();
    lroll[0] = ui->LE_CUR_LEFT_ROLL->text().toDouble();
    lpitch[0] = ui->LE_CUR_LEFT_PITCH->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();
    rroll[0] = ui->LE_CUR_RIGHT_ROLL->text().toDouble();
    rpitch[0] = ui->LE_CUR_RIGHT_PITCH->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = rx[0]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);
    ry[1] = ry[0];          ly[1] = ry[0]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);
    rz[1] = rz[0];          lz[1] = lz[0] + H;
    com_z[1] = 0.06;

    lroll[1] = ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble();
    lpitch[1] = ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble();

    rroll[1] = ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble();
    rpitch[1] = ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble();


    rx[2] = lx[1] + L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);          lx[2] = lx[1];
    ry[2] = ly[1] - L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);          ly[2] = ly[1];
    rz[2] = rz[1] + 2.*H;    lz[2] = lz[1];
    com_z[2] = 0.06;


    lroll[2] = ui->LE_STEP_LENGTH_LEFT_ROLL_3->text().toDouble();
    lpitch[2] = ui->LE_STEP_LENGTH_LEFT_PITCH_3->text().toDouble();

    rroll[2] = ui->LE_STEP_LENGTH_RIGHT_ROLL_3->text().toDouble();
    rpitch[2] = ui->LE_STEP_LENGTH_RIGHT_PITCH_3->text().toDouble();



    rx[3] = rx[2];          lx[3] = rx[2]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R);
    ry[3] = ry[2];          ly[3] = ry[2]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R);
    rz[3] = rz[2];          lz[3] = lz[2] + H;
    com_z[3] = 0.0;


    lroll[3] = ui->LE_STEP_LENGTH_LEFT_ROLL_4->text().toDouble();
    lpitch[3] = ui->LE_STEP_LENGTH_LEFT_PITCH_4->text().toDouble();

    rroll[3] = ui->LE_STEP_LENGTH_RIGHT_ROLL_4->text().toDouble();
    rpitch[3] = ui->LE_STEP_LENGTH_RIGHT_PITCH_4->text().toDouble();



    ryaw[1] = ryaw[2] = ryaw[3] = ryaw[0];
    lyaw[1] = lyaw[2] = lyaw[3] = lyaw[0];


    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_DN_STAIR_4_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

//    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
//    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
//    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
//    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();

//    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
//    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
//    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
//    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();


    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();
    lroll[0] = ui->LE_CUR_LEFT_ROLL->text().toDouble();
    lpitch[0] = ui->LE_CUR_LEFT_PITCH->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();
    rroll[0] = ui->LE_CUR_RIGHT_ROLL->text().toDouble();
    rpitch[0] = ui->LE_CUR_RIGHT_PITCH->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = rx[0] - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);
    ry[1] = ry[0];          ly[1] = ry[0] + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);
    rz[1] = rz[0];          lz[1] = lz[0] - H;
    com_z[1] = -0.15;


    lroll[1] = ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble();
    lpitch[1] = ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble();

    rroll[1] = ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble();
    rpitch[1] = ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble();



    rx[2] = lx[1] + L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);          lx[2] = lx[1];
    ry[2] = ly[1] - L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);          ly[2] = ly[1];
    rz[2] = rz[1] - 2*H;    lz[2] = lz[1];
    com_z[2] = -0.15;


    lroll[2] = ui->LE_STEP_LENGTH_LEFT_ROLL_3->text().toDouble();
    lpitch[2] = ui->LE_STEP_LENGTH_LEFT_PITCH_3->text().toDouble();

    rroll[2] = ui->LE_STEP_LENGTH_RIGHT_ROLL_3->text().toDouble();
    rpitch[2] = ui->LE_STEP_LENGTH_RIGHT_PITCH_3->text().toDouble();


    rx[3] = rx[2];          lx[3] = rx[2]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R);
    ry[3] = ry[2];          ly[3] = ry[2]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R);
    rz[3] = rz[2];          lz[3] = lz[2] - H;
    com_z[3] = 0.0;


    lroll[3] = ui->LE_STEP_LENGTH_LEFT_ROLL_4->text().toDouble();
    lpitch[3] = ui->LE_STEP_LENGTH_LEFT_PITCH_4->text().toDouble();

    rroll[3] = ui->LE_STEP_LENGTH_RIGHT_ROLL_4->text().toDouble();
    rpitch[3] = ui->LE_STEP_LENGTH_RIGHT_PITCH_4->text().toDouble();


    ryaw[1] = ryaw[2] = ryaw[3] = ryaw[0];
    lyaw[1] = lyaw[2] = lyaw[3] = lyaw[0];

    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_UP_STAIR_4_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = -ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

//    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
//    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
//    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
//    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();

//    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
//    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
//    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
//    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();
    lroll[0] = ui->LE_CUR_LEFT_ROLL->text().toDouble();
    lpitch[0] = ui->LE_CUR_LEFT_PITCH->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();
    rroll[0] = ui->LE_CUR_RIGHT_ROLL->text().toDouble();
    rpitch[0] = ui->LE_CUR_RIGHT_PITCH->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = rx[0]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);
    ry[1] = ry[0];          ly[1] = ry[0]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);
    rz[1] = rz[0];          lz[1] = lz[0] + H;
    com_z[1] = 0.03;

    rx[2] = lx[1] + L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);          lx[2] = lx[1];
    ry[2] = ly[1] - L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);          ly[2] = ly[1];
    rz[2] = rz[1] + 2.*H;    lz[2] = lz[1];
    com_z[2] = 0.03;

    rx[3] = rx[2];          lx[3] = rx[2]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R);
    ry[3] = ry[2];          ly[3] = ry[2]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R);
    rz[3] = rz[2];          lz[3] = lz[2] + H;
    com_z[3] = 0.0;

    ryaw[1] = ryaw[2] = ryaw[3] = ryaw[0];
    lyaw[1] = lyaw[2] = lyaw[3] = lyaw[0];

    rroll[1] = rroll[2] = rroll[3] = rroll[0];
    lroll[1] = lroll[2] = lroll[3] = lroll[0];

    rpitch[1] = rpitch[2] = rpitch[3] = rpitch[0];
    lpitch[1] = lpitch[2] = lpitch[3] = lpitch[0];

    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_DN_STAIR_3_clicked()
{
    double rx[6],ry[6],rz[6],lx[6],ly[6],lz[6],com_z[6],lyaw[6],ryaw[6],rpitch[6],lpitch[6],rroll[6],lroll[6];
    double H,L;
    H = ui->LE_FW_TERRAIN_HEIGT->text().toDouble();
    L = -ui->LE_FW_TERRAIN_LENGTH->text().toDouble();

//    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
//    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
//    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
//    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();

//    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
//    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
//    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
//    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();

    lx[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    ly[0] = ui->LE_CUR_LEFT_Y->text().toDouble();
    lz[0] = ui->LE_CUR_LEFT_Z->text().toDouble();
    lyaw[0]  = ui->LE_CUR_LEFT_YAW->text().toDouble();
    lroll[0] = ui->LE_CUR_LEFT_ROLL->text().toDouble();
    lpitch[0] = ui->LE_CUR_LEFT_PITCH->text().toDouble();

    rx[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    ry[0] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    rz[0] = ui->LE_CUR_RIGHT_Z->text().toDouble();
    ryaw[0]  = ui->LE_CUR_RIGHT_YAW->text().toDouble();
    rroll[0] = ui->LE_CUR_RIGHT_ROLL->text().toDouble();
    rpitch[0] = ui->LE_CUR_RIGHT_PITCH->text().toDouble();

    com_z[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

    rx[1] = rx[0];          lx[1] = rx[0] - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);
    ry[1] = ry[0];          ly[1] = ry[0] + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);
    rz[1] = rz[0];          lz[1] = lz[0] - H;
    com_z[1] = -0.1;

    rx[2] = lx[1] + L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R) + L*cos((lyaw[0] + ryaw[0])/2.*D2R);          lx[2] = lx[1];
    ry[2] = ly[1] - L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R) + L*sin((lyaw[0] + ryaw[0])/2.*D2R);          ly[2] = ly[1];
    rz[2] = rz[1] - 2*H;    lz[2] = lz[1];
    com_z[2] = -0.1;

    rx[3] = rx[2];          lx[3] = rx[2]  - L_PEL2PEL*sin((lyaw[0] + ryaw[0])/2.*D2R);
    ry[3] = ry[2];          ly[3] = ry[2]  + L_PEL2PEL*cos((lyaw[0] + ryaw[0])/2.*D2R);
    rz[3] = rz[2];          lz[3] = lz[2] - H;
    com_z[3] = 0.0;

    ryaw[1] = ryaw[2] = ryaw[3] = ryaw[0];
    lyaw[1] = lyaw[2] = lyaw[3] = lyaw[0];

    rroll[1] = rroll[2] = rroll[3] = rroll[0];
    lroll[1] = lroll[2] = lroll[3] = lroll[0];

    rpitch[1] = rpitch[2] = rpitch[3] = rpitch[0];
    lpitch[1] = lpitch[2] = lpitch[3] = lpitch[0];

    fill_blanks(rx,ry,rz,lx,ly,lz,com_z,ryaw,lyaw,rroll,lroll,rpitch,lpitch) ;
}

void WalkingDialog::on_BTN_FILL_1ST_DSP_2_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
}
float th1 = -15.4;
float th2 = -15.3;
float th3 = 15;

float th4 = 15.1;
float th5 = 15.5;
float th6 = -15;
float H = 0.15,L=0.39,PEL=0.26,FOOT=0.24;

float xi = 0.22;

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_RF_1_clicked()
{
    ui->BTN_FILL_UP_TERRAIN_RF_1->setEnabled(false);
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -L));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",0. ));

    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", 0.));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_LF_1_clicked()
{
    ui->BTN_FILL_UP_TERRAIN_RF_1->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_1->setEnabled(false);
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;

    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi-H*sin(fabs(D2R*th1)) + L/2.*cos(fabs(D2R*th1)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", H*cos(fabs(D2R*th1))+L/2*sin(fabs(D2R*th1))));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f", -th1));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_RF_2_clicked()
{
    ui->BTN_FILL_UP_TERRAIN_LF_1->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_2->setEnabled(false);
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + H*sin(fabs(D2R*th4)) + FOOT/2.*cos(fabs(D2R*th4)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", H*cos(fabs(D2R*th4))+(L-FOOT/2.)*sin(fabs(D2R*th4))));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f", -th4));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_LF_2_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    ui->BTN_FILL_UP_TERRAIN_RF_2->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_2->setEnabled(false);
    QString str;
    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L/2.)));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", H + L*sin(fabs(D2R*th2))-PEL/2.*tan(fabs(D2R*th2)) + H/cos(fabs(D2R*th2))));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",-th2 ));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_RF_3_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_LF_2->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_3->setEnabled(false);

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + H*sin(fabs(D2R*th4)) + L*cos(fabs(D2R*th4)) + L/2.)));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", H + L*sin(fabs(D2R*th5)) - PEL/2.*tan(fabs(D2R*th5)) + H/cos(fabs(D2R*th5))));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",-th5 ));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",-0.03));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_LF_3_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_RF_3->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_3->setEnabled(false);

    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", H*cos(fabs(D2R*th3)) + L/2.*sin(fabs(D2R*th3))));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",-th3));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",-0.15));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_RF_4_clicked()
{

    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_LF_3->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_4->setEnabled(false);

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + H*sin(fabs(D2R*th4)) + L*cos(fabs(D2R*th4)) + L + (L - FOOT/2.)*cos(fabs(D2R*th6)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", H*cos(fabs(D2R*th6)) + (L - FOOT/2.)*sin(fabs(D2R*th6))));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",-th6));

    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.02));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_LF_4_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_RF_4->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_4->setEnabled(false);

    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",-0.15));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_RF_5_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_LF_4->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_5->setEnabled(false);

    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -(L + xi + L*cos(fabs(D2R*th1)) + H*sin(fabs(D2R*th4)) + L*cos(fabs(D2R*th4)) + L + (L - FOOT/2.)*cos(fabs(D2R*th6)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",0. ));
    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",0.));

    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.00));
}

void WalkingDialog::on_BTN_FILL_UP_TERRAIN_LF_5_clicked()
{
    on_BTN_FILL_1ST_DSP_clicked();
    QString str;
    ui->BTN_FILL_UP_TERRAIN_RF_4->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_4->setEnabled(false);

    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(L + xi + L*cos(fabs(D2R*th1)) + H*sin(fabs(D2R*th4)) + L*cos(fabs(D2R*th4)) + L + (L - FOOT/2.)*cos(fabs(D2R*th6)))));
    //ui->LE_STEP_LENGTH_LEFT_Y1->setText(str.sprintf("%.4f", ));
    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", 0.));
    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",0.));
    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
}

void WalkingDialog::on_BTN_ENABLE_clicked()
{
    ui->BTN_FILL_UP_TERRAIN_RF_1->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_2->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_3->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_4->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_RF_5->setEnabled(true);

    ui->BTN_FILL_UP_TERRAIN_LF_1->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_2->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_3->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_4->setEnabled(true);
    ui->BTN_FILL_UP_TERRAIN_LF_5->setEnabled(true);
}

void WalkingDialog::on_BTN_FILL_UP_FROM_VISION_clicked()
{
//    on_BTN_FILL_1ST_DSP_clicked();
//    QString str;

////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());

//     double ORI_[3],X_[3],Y_[3],Z_[3];
//     double VISION_RMAT[3][3],roll,pitch,yaw;
//     double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.;


////     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + (-ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() - ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble())/2.;
////     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + (-ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() - ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble())/2.;

//     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//     ORI_[2] = VisionData.mZ + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
//     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
//     X_[2] = VisionData.xZ;

//     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
//     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
//     Y_[2] = VisionData.yZ;

//     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
//     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
//     Z_[2] = VisionData.nZ;

//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
//    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(ORI_[0])));
//    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", -(ORI_[1])));
//    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", (ORI_[2])));
//    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",yaw));
//    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",-roll));
//    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",-pitch));
//    //ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
////
//    if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
//    {
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
//    }
//    else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
//    {
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
//    }

}

void WalkingDialog::on_BTN_FILL_UP_FROM_VISION_2_clicked()
{
//    on_BTN_FILL_1ST_DSP_clicked();
//    QString str;

////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());

//     double ORI_[3],X_[3],Y_[3],Z_[3];
//     double VISION_RMAT[3][3],roll,pitch,yaw;
//     double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.0;


//     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//     ORI_[2] = VisionData.mZ + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];
//     //printf("Vision.offset[2] = %f,pLAN->LANPODOData->GUI_PARA_FLOAT[16] = %f\n ",VisionData.offset[2],pLAN->LANPODOData->GUI_PARA_FLOAT[16]);
//     cout << "vision offset[2]"<<VisionData.offset[2]<<"PEL Z"<<PODO_DATA.UserM2G.curPEL[2] <<endl;
//     //ORI_[2] = VisionData.mZ + VisionData.offset[2] + ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

//     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
//     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
//     X_[2] = VisionData.xZ;

//     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
//     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
//     Y_[2] = VisionData.yZ;

//     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
//     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
//     Z_[2] = VisionData.nZ;

//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
//    ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", -(ORI_[0])));
//    ui->LE_STEP_LENGTH_LEFT_Y2->setText(str.sprintf("%.4f", -(ORI_[1])));
//    ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", (ORI_[2])));
//    ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",yaw));
//    ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",-roll));
//    ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",-pitch));
////    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
//    if(ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble()));
//    else if(ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble()));
}



void WalkingDialog::on_BTN_STEP_TO_BE_WALK_READY_POS_clicked()
{
    on_BTN_CLEAR_ALL_clicked();

    DSPTask task;
    task.Left[0] = ui->LE_CUR_LEFT_X->text().toDouble();
    task.Left[1] = ui->LE_CUR_LEFT_Y->text().toDouble();
    task.Left[2] = ui->LE_CUR_LEFT_Z->text().toDouble();
    task.LYaw = ui->LE_CUR_LEFT_YAW->text().toDouble();
    task.Right[0] = ui->LE_CUR_RIGHT_X->text().toDouble();
    task.Right[1] = ui->LE_CUR_RIGHT_Y->text().toDouble();
    task.Right[2] = ui->LE_CUR_RIGHT_Z->text().toDouble();
    task.RYaw = ui->LE_CUR_RIGHT_YAW->text().toDouble();
    task.COMz[0] = ui->LE_CUR_COM_Z_OFFSET->text().toDouble();
    task.HTime = 0.0;

    DSPScheduler.push_back(task);
    InsertTable(ui->TW_DSP_SCHEDULER->rowCount(), task);

    double length = pLAN->G2MData->StepLength = 0.00001;

        for(int i=0; i<1; i++)
        {
            int foot = FootSelector(DIR_FORWARD);
            int pos = DSPScheduler.size()-1;
            if(foot == FOOT_RIGHT)
            {
                task.Left[0] = DSPScheduler[pos].Left[0];
                task.Left[1] = DSPScheduler[pos].Left[1];
                task.Left[2] = DSPScheduler[pos].Left[2];

                task.Right[0] = DSPScheduler[pos].Left[0] + (L_PEL2PEL-Offset_to_WR)*sin((DSPScheduler[pos].LYaw + DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[1] = DSPScheduler[pos].Left[1] - (L_PEL2PEL-Offset_to_WR)*cos((DSPScheduler[pos].LYaw + DSPScheduler[pos].RYaw)/2.*D2R);
                task.Right[2] = DSPScheduler[pos].Left[2];

                task.LYaw = DSPScheduler[pos].LYaw;
                task.RYaw = DSPScheduler[pos].RYaw;
                task.HTime = 0.0;
                DSPScheduler.push_back(task);
                InsertTable(ui->TW_DSP_SCHEDULER->rowCount(), task);
            }else
            {
                task.Left[0] = DSPScheduler[pos].Right[0] - (L_PEL2PEL-Offset_to_WR)*sin((DSPScheduler[pos].LYaw + DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[1] = DSPScheduler[pos].Right[1] + (L_PEL2PEL-Offset_to_WR)*cos((DSPScheduler[pos].LYaw + DSPScheduler[pos].RYaw)/2.*D2R);
                task.Left[2] = DSPScheduler[pos].Right[2];

                task.Right[0] = DSPScheduler[pos].Right[0];
                task.Right[1] = DSPScheduler[pos].Right[1];
                task.Right[2] = DSPScheduler[pos].Right[2];

                task.LYaw = DSPScheduler[pos].LYaw;
                task.RYaw = DSPScheduler[pos].RYaw;
                task.HTime = 0.0;
                DSPScheduler.push_back(task);
                InsertTable(ui->TW_DSP_SCHEDULER->rowCount(), task);
            }
        }



    on_TW_DSP_SCHEDULER_cellClicked(ui->TW_DSP_SCHEDULER->rowCount()-1, 0);
    Refresh();

    on_BTN_FW_SAVE_PLAY_clicked();
}


void WalkingDialog::on_BTN_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_PRE_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_VISION_DIRECT_CONNECT_clicked()
{
//    if(visionInfo->RBConnectionState == RBLAN_CS_CONNECTED){
//         visionInfo->RBDisconnect();
//        ui->BTN_VISION_DIRECT_CONNECT->setText("Direct Connect");
//    }else{
//         visionInfo->RBConnect(ui->LE_VISION_DIRECT_IP->text(), ui->LE_VISION_DIRECT_PORT->text().toInt());
//        ui->BTN_VISION_DIRECT_CONNECT->setText("Direct Disconnect");
//    }
}

void WalkingDialog::on_BTN_VISION_DIRECT_REQUEST_clicked()
{
//    QByteArray tosend;
//    uniInt head, type, size, tail, tosendData;
//    head.iData = DATA_HEADER;
//    type.iData = 30051;
//    size.iData = 4;
//    tosendData.iData = 0;
//    tail.iData = DATA_TAIL;
//    tosend.append(head.cData,4);
//    tosend.append(type.cData,4);
//    tosend.append(size.cData,4);
//    tosend.append(tosendData.cData,4);
//    tosend.append(tail.cData,4);
//    visionInfo->RBSendData(tosend);
}

void WalkingDialog::on_BTN_MOTION_CHECK_clicked()
{

//    int temp_step_num = ui->LE_FW_STEP_NUM->text().toInt();

//    cmd.COMMAND_DATA.USER_PARA_INT[5] = temp_step_num;
//    //===============================1st DSP========================================
//    pLAN->G2MData->walkingDSP[0] = ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble();
//    pLAN->G2MData->walkingDSP[1] = ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble();
//    pLAN->G2MData->walkingDSP[2] = ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble();
//    pLAN->G2MData->walkingDSP[3] = ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble();
//    pLAN->G2MData->walkingDSP[4] = ui->LE_STEP_LENGTH_LEFT_ROLL_1->text().toDouble();   //roll
//    pLAN->G2MData->walkingDSP[5] = ui->LE_STEP_LENGTH_LEFT_PITCH_1->text().toDouble();   //pitch

//    pLAN->G2MData->walkingDSP[6] = ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble();
//    pLAN->G2MData->walkingDSP[7] = ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble();
//    pLAN->G2MData->walkingDSP[8] = ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble();
//    pLAN->G2MData->walkingDSP[9] = ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble();
//    pLAN->G2MData->walkingDSP[10] = ui->LE_STEP_LENGTH_RIGHT_ROLL_1->text().toDouble();   //roll
//    pLAN->G2MData->walkingDSP[11] = ui->LE_STEP_LENGTH_RIGHT_PITCH_1->text().toDouble();   //pitch

//    pLAN->G2MData->walkingDSP[12] = ui->LE_STEP_LEFT_COM_Z1->text().toDouble();
//    pLAN->G2MData->walkingDSP[13] = 0.;
//    //===============================2nd DSP===============================
//    pLAN->G2MData->walkingDSP[14] = ui->LE_STEP_LENGTH_LEFT_X2->text().toDouble();
//    pLAN->G2MData->walkingDSP[15] = ui->LE_STEP_LENGTH_LEFT_Y2->text().toDouble();
//    pLAN->G2MData->walkingDSP[16] = ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble();
//    pLAN->G2MData->walkingDSP[17] = ui->LE_STEP_LENGTH_LEFT_YAW_2->text().toDouble();
//    pLAN->G2MData->walkingDSP[18] = ui->LE_STEP_LENGTH_LEFT_ROLL_2->text().toDouble();
//    pLAN->G2MData->walkingDSP[19] = ui->LE_STEP_LENGTH_LEFT_PITCH_2->text().toDouble();

//    pLAN->G2MData->walkingDSP[20] = ui->LE_STEP_LENGTH_RIGHT_X2->text().toDouble();
//    pLAN->G2MData->walkingDSP[21] = ui->LE_STEP_LENGTH_RIGHT_Y2->text().toDouble();
//    pLAN->G2MData->walkingDSP[22] = ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble();
//    pLAN->G2MData->walkingDSP[23] = ui->LE_STEP_LENGTH_RIGHT_YAW_2->text().toDouble();
//    pLAN->G2MData->walkingDSP[24] = ui->LE_STEP_LENGTH_RIGHT_ROLL_2->text().toDouble();
//    pLAN->G2MData->walkingDSP[25] = ui->LE_STEP_LENGTH_RIGHT_PITCH_2->text().toDouble();

//    pLAN->G2MData->walkingDSP[26] = ui->LE_STEP_LEFT_COM_Z2->text().toDouble();
//    pLAN->G2MData->walkingDSP[27] = 0.;
//    //===============================3rd DSP===============================
//    pLAN->G2MData->walkingDSP[28] = ui->LE_STEP_LENGTH_LEFT_X3->text().toDouble();
//    pLAN->G2MData->walkingDSP[29] = ui->LE_STEP_LENGTH_LEFT_Y3->text().toDouble();
//    pLAN->G2MData->walkingDSP[30] = ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble();
//    pLAN->G2MData->walkingDSP[31] = ui->LE_STEP_LENGTH_LEFT_YAW_3->text().toDouble();
//    pLAN->G2MData->walkingDSP[32] =  ui->LE_STEP_LENGTH_LEFT_ROLL_3->text().toDouble();
//    pLAN->G2MData->walkingDSP[33] = ui->LE_STEP_LENGTH_LEFT_PITCH_3->text().toDouble();

//    pLAN->G2MData->walkingDSP[34] = ui->LE_STEP_LENGTH_RIGHT_X3->text().toDouble();
//    pLAN->G2MData->walkingDSP[35] = ui->LE_STEP_LENGTH_RIGHT_Y3->text().toDouble();
//    pLAN->G2MData->walkingDSP[36] = ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble();
//    pLAN->G2MData->walkingDSP[37] = ui->LE_STEP_LENGTH_RIGHT_YAW_3->text().toDouble();
//    pLAN->G2MData->walkingDSP[38] =  ui->LE_STEP_LENGTH_RIGHT_ROLL_3->text().toDouble();
//    pLAN->G2MData->walkingDSP[39] = ui->LE_STEP_LENGTH_RIGHT_PITCH_3->text().toDouble();

//    pLAN->G2MData->walkingDSP[40] = ui->LE_STEP_LEFT_COM_Z3->text().toDouble();
//    pLAN->G2MData->walkingDSP[41] = 0.;
//    //===============================4th DSP===============================
//    pLAN->G2MData->walkingDSP[42] = ui->LE_STEP_LENGTH_LEFT_X4->text().toDouble();
//    pLAN->G2MData->walkingDSP[43] = ui->LE_STEP_LENGTH_LEFT_Y4->text().toDouble();
//    pLAN->G2MData->walkingDSP[44] = ui->LE_STEP_LENGTH_LEFT_Z4->text().toDouble();
//    pLAN->G2MData->walkingDSP[45] = ui->LE_STEP_LENGTH_LEFT_YAW_4->text().toDouble();
//    pLAN->G2MData->walkingDSP[46] =  ui->LE_STEP_LENGTH_LEFT_ROLL_4->text().toDouble();
//    pLAN->G2MData->walkingDSP[47] = ui->LE_STEP_LENGTH_LEFT_PITCH_4->text().toDouble();

//    pLAN->G2MData->walkingDSP[48] = ui->LE_STEP_LENGTH_RIGHT_X4->text().toDouble();
//    pLAN->G2MData->walkingDSP[49] = ui->LE_STEP_LENGTH_RIGHT_Y4->text().toDouble();
//    pLAN->G2MData->walkingDSP[50] = ui->LE_STEP_LENGTH_RIGHT_Z4->text().toDouble();
//    pLAN->G2MData->walkingDSP[51] = ui->LE_STEP_LENGTH_RIGHT_YAW_4->text().toDouble();
//    pLAN->G2MData->walkingDSP[52] = ui->LE_STEP_LENGTH_RIGHT_ROLL_4->text().toDouble();
//    pLAN->G2MData->walkingDSP[53] =ui->LE_STEP_LENGTH_RIGHT_PITCH_4->text().toDouble();

//    pLAN->G2MData->walkingDSP[54] = ui->LE_STEP_LEFT_COM_Z4->text().toDouble();
//    pLAN->G2MData->walkingDSP[55] = 0.;
//    //===============================5th DSP===============================
//    pLAN->G2MData->walkingDSP[56] = ui->LE_STEP_LENGTH_LEFT_X5->text().toDouble();
//    pLAN->G2MData->walkingDSP[57] = ui->LE_STEP_LENGTH_LEFT_Y5->text().toDouble();
//    pLAN->G2MData->walkingDSP[58] = ui->LE_STEP_LENGTH_LEFT_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[59] = ui->LE_STEP_LENGTH_LEFT_YAW_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[60] =  ui->LE_STEP_LENGTH_LEFT_ROLL_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[61] =  ui->LE_STEP_LENGTH_LEFT_PITCH_5->text().toDouble();

//    pLAN->G2MData->walkingDSP[62] = ui->LE_STEP_LENGTH_RIGHT_X5->text().toDouble();
//    pLAN->G2MData->walkingDSP[63] = ui->LE_STEP_LENGTH_RIGHT_Y5->text().toDouble();
//    pLAN->G2MData->walkingDSP[64] = ui->LE_STEP_LENGTH_RIGHT_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[65] = ui->LE_STEP_LENGTH_RIGHT_YAW_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[66] = ui->LE_STEP_LENGTH_RIGHT_ROLL_5->text().toDouble();
//    pLAN->G2MData->walkingDSP[67] = ui->LE_STEP_LENGTH_RIGHT_PITCH_5->text().toDouble();

//    pLAN->G2MData->walkingDSP[68] = ui->LE_STEP_LEFT_COM_Z5->text().toDouble();
//    pLAN->G2MData->walkingDSP[69] = 0.;
//    //===============================6th DSP===============================
//    pLAN->G2MData->walkingDSP[70] = ui->LE_STEP_LENGTH_LEFT_X6->text().toDouble();
//    pLAN->G2MData->walkingDSP[71] = ui->LE_STEP_LENGTH_LEFT_Y6->text().toDouble();
//    pLAN->G2MData->walkingDSP[72] = ui->LE_STEP_LENGTH_LEFT_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[73] = ui->LE_STEP_LENGTH_LEFT_YAW_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[74] =  ui->LE_STEP_LENGTH_LEFT_ROLL_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[75] =  ui->LE_STEP_LENGTH_LEFT_PITCH_6->text().toDouble();

//    pLAN->G2MData->walkingDSP[76] = ui->LE_STEP_LENGTH_RIGHT_X6->text().toDouble();
//    pLAN->G2MData->walkingDSP[77] = ui->LE_STEP_LENGTH_RIGHT_Y6->text().toDouble();
//    pLAN->G2MData->walkingDSP[78] = ui->LE_STEP_LENGTH_RIGHT_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[79] = ui->LE_STEP_LENGTH_RIGHT_YAW_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[80] = ui->LE_STEP_LENGTH_RIGHT_ROLL_6->text().toDouble();
//    pLAN->G2MData->walkingDSP[81] = ui->LE_STEP_LENGTH_RIGHT_PITCH_6->text().toDouble();

//    pLAN->G2MData->walkingDSP[82] = ui->LE_STEP_LEFT_COM_Z6->text().toDouble();
//    pLAN->G2MData->walkingDSP[83] = 0.;
//    //===============================7th DSP===============================
//    pLAN->G2MData->walkingDSP[84] = ui->LE_STEP_LENGTH_LEFT_X7->text().toDouble();
//    pLAN->G2MData->walkingDSP[85] = ui->LE_STEP_LENGTH_LEFT_Y7->text().toDouble();
//    pLAN->G2MData->walkingDSP[86] = ui->LE_STEP_LENGTH_LEFT_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[87] = ui->LE_STEP_LENGTH_LEFT_YAW_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[88] =  ui->LE_STEP_LENGTH_LEFT_ROLL_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[89] =  ui->LE_STEP_LENGTH_LEFT_PITCH_7->text().toDouble();

//    pLAN->G2MData->walkingDSP[90] = ui->LE_STEP_LENGTH_RIGHT_X7->text().toDouble();
//    pLAN->G2MData->walkingDSP[91] = ui->LE_STEP_LENGTH_RIGHT_Y7->text().toDouble();
//    pLAN->G2MData->walkingDSP[92] = ui->LE_STEP_LENGTH_RIGHT_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[93] = ui->LE_STEP_LENGTH_RIGHT_YAW_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[94] = ui->LE_STEP_LENGTH_RIGHT_ROLL_7->text().toDouble();
//    pLAN->G2MData->walkingDSP[95] = ui->LE_STEP_LENGTH_RIGHT_PITCH_7->text().toDouble();

//    pLAN->G2MData->walkingDSP[96] = ui->LE_STEP_LEFT_COM_Z7->text().toDouble();
//    pLAN->G2MData->walkingDSP[97] = 0.;
//    //===============================8th DSP===============================
//    pLAN->G2MData->walkingDSP[98] = ui->LE_STEP_LENGTH_LEFT_X8->text().toDouble();
//    pLAN->G2MData->walkingDSP[99] = ui->LE_STEP_LENGTH_LEFT_Y8->text().toDouble();
//    pLAN->G2MData->walkingDSP[100] = ui->LE_STEP_LENGTH_LEFT_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[101] = ui->LE_STEP_LENGTH_LEFT_YAW_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[102] =  ui->LE_STEP_LENGTH_LEFT_ROLL_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[103] =  ui->LE_STEP_LENGTH_LEFT_PITCH_8->text().toDouble();

//    pLAN->G2MData->walkingDSP[104] = ui->LE_STEP_LENGTH_RIGHT_X8->text().toDouble();
//    pLAN->G2MData->walkingDSP[105] = ui->LE_STEP_LENGTH_RIGHT_Y8->text().toDouble();
//    pLAN->G2MData->walkingDSP[106] = ui->LE_STEP_LENGTH_RIGHT_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[107] = ui->LE_STEP_LENGTH_RIGHT_YAW_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[108] = ui->LE_STEP_LENGTH_RIGHT_ROLL_8->text().toDouble();
//    pLAN->G2MData->walkingDSP[109] = ui->LE_STEP_LENGTH_RIGHT_PITCH_8->text().toDouble();

//    pLAN->G2MData->walkingDSP[110] = ui->LE_STEP_LEFT_COM_Z8->text().toDouble();
//    pLAN->G2MData->walkingDSP[111] = 0.;


//    //===========================Vision DATA
//    //vision plane number

////    pLAN->G2MData->block_num = VisionData.Floor_min_num;


//    //vision center data

//    for(int i=0; i< pLAN->G2MData->block_num ; i++)
//    {
//        for(int j=0;j<3;j++)
//        {
//            pLAN->G2MData->block_center[i][j] = VisionData.Floor_center[i][j];
//        }
//    }



//    //edge data


//    for(int i=0; i< pLAN->G2MData->block_num ; i++)
//    {
//        for(int j=0;j<12;j++)
//        {
//            pLAN->G2MData->block_edge[i][j] = _G2M_block_edge[i][j]; //VisionData.Floor_prearrange[i][j];
//        }

//        for(int k = 0; k<3; k++)
//        {
//            pLAN->G2MData->block_normal[i][k] = VisionData.Floor_nrearrange[i][k];
//        }
//    }



//    cmd.COMMAND_DATA.USER_PARA_INT[9] = TERRAIN_WALKING;

//    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_MOTION_CHECK;
//    cmd.COMMAND_TARGET = alNumFreeWalking;
//    pLAN->SendCommand(cmd);
//    QString str;
//    str.sprintf("%s", cmd.COMMAND_DATA.USER_PARA_CHAR);
//    cout << str.toStdString().data() << endl;
}

void WalkingDialog::on_BTN_FILL_UP_FROM_VISION_3_clicked()
{
//    on_BTN_FILL_DSP3_LAST();
//    QString str;

////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());

//     double ORI_[3],X_[3],Y_[3],Z_[3];
//     double VISION_RMAT[3][3],roll,pitch,yaw;
//     double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.;


////     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + (-ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() - ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble())/2.;
////     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + (-ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() - ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble())/2.;

//     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//     ORI_[2] = VisionData.mZ + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
//     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
//     X_[2] = VisionData.xZ;

//     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
//     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
//     Y_[2] = VisionData.yZ;

//     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
//     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
//     Z_[2] = VisionData.nZ;

//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
//    ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", -(ORI_[0])));
//    ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", -(ORI_[1])));
//    ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", (ORI_[2])));
//    ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f",yaw));
//    ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f",-roll));
//    ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f",-pitch));
//    //ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));

//    if(ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble()));
//    else if(ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble()));
}

void WalkingDialog::on_BTN_FILL_UP_FROM_VISION_4_clicked()
{
//    on_BTN_FILL_DSP3_LAST();
//    QString str;

////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());

//     double ORI_[3],X_[3],Y_[3],Z_[3];
//     double VISION_RMAT[3][3],roll,pitch,yaw;
//     double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.0;


//     ORI_[0] = ((VisionData.mX + VisionData.offset[0])*cos(th*D2R) - (VisionData.mY + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//     ORI_[1] = ((VisionData.mX + VisionData.offset[0])*sin(th*D2R) + (VisionData.mY + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//     ORI_[2] = VisionData.mZ + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];
//     //printf("Vision.offset[2] = %f,pLAN->LANPODOData->GUI_PARA_FLOAT[16] = %f\n ",VisionData.offset[2],pLAN->LANPODOData->GUI_PARA_FLOAT[16]);
//     cout << "vision offset[2]"<<VisionData.offset[2]<<"PEL Z"<<PODO_DATA.UserM2G.curPEL[2] <<endl;
//     //ORI_[2] = VisionData.mZ + VisionData.offset[2] + ui->LE_CUR_COM_Z_OFFSET->text().toDouble();

//     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
//     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
//     X_[2] = VisionData.xZ;

//     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
//     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
//     Y_[2] = VisionData.yZ;

//     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
//     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
//     Z_[2] = VisionData.nZ;

//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
//    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
//    ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", -(ORI_[0])));
//    ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", -(ORI_[1])));
//    ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", (ORI_[2])));
//    ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f",yaw));
//    ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f",-roll));
//    ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f",-pitch));
////    ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
//    if(ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble() < ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble()));
//    else if(ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble() > ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble())
//        ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble()));
}



void WalkingDialog::on_BTN_RF_2nd_clicked()
{

//    on_BTN_FILL_1ST_DSP_clicked();
//    QString str;

////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());
//    //---Foot position control variable . 0<= Xc <= 1, 0<= Yc <= 1
//    //---Xc and Yc are ratio of foot position on block. bic Xc means bic step length in x direction, bic Yc means bic step length in y direction.
//    double Xc = 0.19f/0.38f,Yc=0.12f/0.38f;
//    double LF_ORI_[4][3],X_[3],Y_[3],Z_[3];
//    double VISION_RMAT[3][3],roll,pitch,yaw;
//    double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.;
//    double inner_pos[3],outer_pos[3],outer_vec[3],inner_vec[3],cross_vec[3],des_pos[3];

//    //-------------------------transformed 4 edge points in robot coordinate

//    //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//    LF_ORI_[0][0] = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    LF_ORI_[0][1] = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    LF_ORI_[0][2] = VisionData.LF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    LF_ORI_[1][0] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    LF_ORI_[1][1] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    LF_ORI_[1][2] = VisionData.LF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    LF_ORI_[2][0] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    LF_ORI_[2][1] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    LF_ORI_[2][2] = VisionData.LF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    LF_ORI_[3][0] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    LF_ORI_[3][1] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    LF_ORI_[3][2] = VisionData.LF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    cout << "Transform Left plane p1 \n"<<"x:"<<LF_ORI_[0][0]<<"y:"<<LF_ORI_[0][1]<<"z:"<<LF_ORI_[0][2]<<endl;
//    cout << "Transform Left plane p2 \n"<<"x:"<<LF_ORI_[1][0]<<"y:"<<LF_ORI_[1][1]<<"z:"<<LF_ORI_[1][2]<<endl;
//    cout << "Transform Left plane p3 \n"<<"x:"<<LF_ORI_[2][0]<<"y:"<<LF_ORI_[2][1]<<"z:"<<LF_ORI_[2][2]<<endl;
//    cout << "Transform Left plane p4 \n"<<"x:"<<LF_ORI_[3][0]<<"y:"<<LF_ORI_[3][1]<<"z:"<<LF_ORI_[3][2]<<endl;

//    //------------------linear equation from close to far(x direction) and inner to outer(y direction) in Left block

//    //------------------ direction vector
//    outer_vec[0] = LF_ORI_[2][0] - LF_ORI_[0][0];
//    outer_vec[1] = LF_ORI_[2][1] - LF_ORI_[0][1];
//    outer_vec[2] = LF_ORI_[2][2] - LF_ORI_[0][2];

//    inner_vec[0] = LF_ORI_[3][0] - LF_ORI_[1][0];
//    inner_vec[1] = LF_ORI_[3][1] - LF_ORI_[1][1];
//    inner_vec[2] = LF_ORI_[3][2] - LF_ORI_[1][2];

//    //------------------inner and outer linear equation , decision foot position on Left block
//    inner_pos[0] = LF_ORI_[1][0] + inner_vec[0]*Xc;
//    inner_pos[1] = LF_ORI_[1][1] + inner_vec[1]*Xc;
//    inner_pos[2] = LF_ORI_[1][2] + inner_vec[2]*Xc;

//    outer_pos[0] = LF_ORI_[0][0] + outer_vec[0]*Xc;
//    outer_pos[1] = LF_ORI_[0][1] + outer_vec[1]*Xc;
//    outer_pos[2] = LF_ORI_[0][2] + outer_vec[2]*Xc;

//    //------------------inner to outer vector (- y direction)
//    //-----direction vector

//    cross_vec[0] = outer_pos[0] - inner_pos[0];
//    cross_vec[1] = outer_pos[1] - inner_pos[1];
//    cross_vec[2] = outer_pos[2] - inner_pos[2];

//    des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//    des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//    des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//    cout << "Foot print \n" <<endl;
//    cout <<"x print:"<<des_pos[0]<<"y print:"<<des_pos[1]<<"z print:"<<des_pos[2]<<endl;


//    //------------------Orientation
//    X_[0] = VisionData.LF_vec[0][0]*cos(th*D2R) - VisionData.LF_vec[0][1]*sin(th*D2R);
//    X_[1] = VisionData.LF_vec[0][0]*sin(th*D2R) + VisionData.LF_vec[0][1]*cos(th*D2R);
//    X_[2] = VisionData.LF_vec[0][2];

//    Y_[0] = VisionData.LF_vec[1][0]*cos(th*D2R) - VisionData.LF_vec[1][1]*sin(th*D2R);
//    Y_[1] = VisionData.LF_vec[1][0]*sin(th*D2R) + VisionData.LF_vec[1][1]*cos(th*D2R);
//    Y_[2] = VisionData.LF_vec[1][2];

//    Z_[0] = VisionData.LF_vec[2][0]*cos(th*D2R) - VisionData.LF_vec[2][1]*sin(th*D2R);
//    Z_[1] = VisionData.LF_vec[2][0]*sin(th*D2R) + VisionData.LF_vec[2][1]*cos(th*D2R);
//    Z_[2] = VisionData.LF_vec[2][2];

//    cout << th <<endl;
//    cout << "normal vec  \n"<<"xX:"<<VisionData.LF_vec[0][0]<<"xY:"<<VisionData.LF_vec[0][1]<<"xZ:"<<VisionData.LF_vec[0][2]<<endl;
//    cout << "normal vec \n"<<"yX:"<<VisionData.LF_vec[1][0]<<"yY:"<<VisionData.LF_vec[1][1]<<"yZ:"<<VisionData.LF_vec[1][2]<<endl;
//    cout << "normal vec \n"<<"zX:"<<VisionData.LF_vec[2][0]<<"zY:"<<VisionData.LF_vec[2][1]<<"zZ:"<<VisionData.LF_vec[2][2]<<endl;

////     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
////     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
////     X_[2] = VisionData.xZ;

////     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
////     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
////     Y_[2] = VisionData.yZ;

////     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
////     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
////     Z_[2] = VisionData.nZ;

//    //---------------------------------------------
//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//     cout << "Block orientation in robot coordinate \n"<<"roll:"<<-roll<<"yaw:"<<yaw<<"pitch:"<<-pitch<<endl;

////    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
////    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
////    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -des_pos[0]));
////    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", -des_pos[1]));
////    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", des_pos[2]));
////    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",yaw));
////    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",-roll));
////    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",-pitch));
////    //ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
//////
////    if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
////    {
////        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
////    }
////    else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
////    {
////        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
////    }




}

void WalkingDialog::on_BTN_LF_2nd_clicked()
{

//    on_BTN_FILL_1ST_DSP_clicked();


////     VisionData.offset[0] + (ui->LE_STEP_LENGTH_RIGHT_X1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_X1->text().toDouble());
////     VisionData.offset[1] + (ui->LE_STEP_LENGTH_RIGHT_Y1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Y1->text().toDouble());
////     VisionData.offset[2] + (ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble());
//    //---Foot position control variable . 0<= Xc <= 1, 0<= Yc <= 1
//    //---Xc and Yc are ratio of foot position on block. bic Xc means bic step length in x direction, bic Yc means bic step length in y direction.
//    double Xc = 0.19f/0.38f,Yc=0.12f/0.38f;
//    double RF_ORI_[4][3],X_[3],Y_[3],Z_[3];
//    double VISION_RMAT[3][3],roll,pitch,yaw;
//    double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.;
//    double inner_pos[3],outer_pos[3],outer_vec[3],inner_vec[3],cross_vec[3],des_pos[3];

//    //-------------------------transformed 4 edge points in robot coordinate

//    //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//    RF_ORI_[0][0] = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    RF_ORI_[0][1] = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    RF_ORI_[0][2] = VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    RF_ORI_[1][0] = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    RF_ORI_[1][1] = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    RF_ORI_[1][2] = VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    RF_ORI_[2][0] = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    RF_ORI_[2][1] = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    RF_ORI_[2][2] = VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    RF_ORI_[3][0] = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//    RF_ORI_[3][1] = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//    RF_ORI_[3][2] = VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//    cout << "Transform Right plane p1 \n"<<"x:"<<RF_ORI_[0][0]<<"y:"<<RF_ORI_[0][1]<<"z:"<<RF_ORI_[0][2]<<endl;
//    cout << "Transform Right plane p2 \n"<<"x:"<<RF_ORI_[1][0]<<"y:"<<RF_ORI_[1][1]<<"z:"<<RF_ORI_[1][2]<<endl;
//    cout << "Transform Right plane p3 \n"<<"x:"<<RF_ORI_[2][0]<<"y:"<<RF_ORI_[2][1]<<"z:"<<RF_ORI_[2][2]<<endl;
//    cout << "Transform Right plane p4 \n"<<"x:"<<RF_ORI_[3][0]<<"y:"<<RF_ORI_[3][1]<<"z:"<<RF_ORI_[3][2]<<endl;

//    //------------------linear equation from close to far(x direction) and inner to outer(y direction) in right block

//    //------------------ direction vector
//    inner_vec[0] = RF_ORI_[2][0] - RF_ORI_[0][0];
//    inner_vec[1] = RF_ORI_[2][1] - RF_ORI_[0][1];
//    inner_vec[2] = RF_ORI_[2][2] - RF_ORI_[0][2];

//    outer_vec[0] = RF_ORI_[3][0] - RF_ORI_[1][0];
//    outer_vec[1] = RF_ORI_[3][1] - RF_ORI_[1][1];
//    outer_vec[2] = RF_ORI_[3][2] - RF_ORI_[1][2];

//    //------------------inner and outer linear equation , decision foot position on right block
//    inner_pos[0] = RF_ORI_[0][0] + inner_vec[0]*Xc;
//    inner_pos[1] = RF_ORI_[0][1] + inner_vec[1]*Xc;
//    inner_pos[2] = RF_ORI_[0][2] + inner_vec[2]*Xc;

//    outer_pos[0] = RF_ORI_[1][0] + outer_vec[0]*Xc;
//    outer_pos[1] = RF_ORI_[1][1] + outer_vec[1]*Xc;
//    outer_pos[2] = RF_ORI_[1][2] + outer_vec[2]*Xc;

//    //------------------inner to outer vector (- y direction)
//    //-----direction vector

//    cross_vec[0] = outer_pos[0] - inner_pos[0];
//    cross_vec[1] = outer_pos[1] - inner_pos[1];
//    cross_vec[2] = outer_pos[2] - inner_pos[2];

//    des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//    des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//    des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//    cout << "Foot print \n" <<endl;
//    cout <<"x print:"<<des_pos[0]<<"y print:"<<des_pos[1]<<"z print:"<<des_pos[2]<<endl;


//    //------------------Orientation
//    X_[0] = VisionData.RF_vec[0][0]*cos(th*D2R) - VisionData.RF_vec[0][1]*sin(th*D2R);
//    X_[1] = VisionData.RF_vec[0][0]*sin(th*D2R) + VisionData.RF_vec[0][1]*cos(th*D2R);
//    X_[2] = VisionData.RF_vec[0][2];

//    Y_[0] = VisionData.RF_vec[1][0]*cos(th*D2R) - VisionData.RF_vec[1][1]*sin(th*D2R);
//    Y_[1] = VisionData.RF_vec[1][0]*sin(th*D2R) + VisionData.RF_vec[1][1]*cos(th*D2R);
//    Y_[2] = VisionData.RF_vec[1][2];

//    Z_[0] = VisionData.RF_vec[2][0]*cos(th*D2R) - VisionData.RF_vec[2][1]*sin(th*D2R);
//    Z_[1] = VisionData.RF_vec[2][0]*sin(th*D2R) + VisionData.RF_vec[2][1]*cos(th*D2R);
//    Z_[2] = VisionData.RF_vec[2][2];

//    cout << th <<endl;
//    cout << "normal vec  \n"<<"xX:"<<VisionData.RF_vec[0][0]<<"xY:"<<VisionData.RF_vec[0][1]<<"xZ:"<<VisionData.RF_vec[0][2]<<endl;
//    cout << "normal vec \n"<<"yX:"<<VisionData.RF_vec[1][0]<<"yY:"<<VisionData.RF_vec[1][1]<<"yZ:"<<VisionData.RF_vec[1][2]<<endl;
//    cout << "normal vec \n"<<"zX:"<<VisionData.RF_vec[2][0]<<"zY:"<<VisionData.RF_vec[2][1]<<"zZ:"<<VisionData.RF_vec[2][2]<<endl;

////     X_[0] = VisionData.xX*cos(th*D2R) - VisionData.xY*sin(th*D2R);
////     X_[1] = VisionData.xX*sin(th*D2R) + VisionData.xY*cos(th*D2R);
////     X_[2] = VisionData.xZ;

////     Y_[0] = VisionData.yX*cos(th*D2R) - VisionData.yY*sin(th*D2R);
////     Y_[1] = VisionData.yX*sin(th*D2R) + VisionData.yY*cos(th*D2R);
////     Y_[2] = VisionData.yZ;

////     Z_[0] = VisionData.nX*cos(th*D2R) - VisionData.nY*sin(th*D2R);
////     Z_[1] = VisionData.nX*sin(th*D2R) + VisionData.nY*cos(th*D2R);
////     Z_[2] = VisionData.nZ;

//    //---------------------------------------------
//     VISION_RMAT[0][0] = X_[0];
//     VISION_RMAT[1][0] = X_[1];
//     VISION_RMAT[2][0] = X_[2];

//     VISION_RMAT[0][1] = Y_[0];
//     VISION_RMAT[1][1] = Y_[1];
//     VISION_RMAT[2][1] = Y_[2];

//     VISION_RMAT[0][2] = Z_[0];
//     VISION_RMAT[1][2] = Z_[1];
//     VISION_RMAT[2][2] = Z_[2];

//     roll    = asin(VISION_RMAT[2][1])*R2D;
//     yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//     pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//     cout << "Block orientation in robot coordinate \n"<<"roll:"<<-roll<<"yaw:"<<yaw<<"pitch:"<<-pitch<<endl;

////    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -(xi + L*cos(fabs(D2R*th1)) + L + L*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + FOOT*1.2/2)));
////    //ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -( L + xi + L*cos(fabs(D2R*th1)) + L +L/2.*cos(fabs(D2R*th3)) + H*sin(fabs(D2R*th3)) + L)));
////    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", -des_pos[0]));
////    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", -des_pos[1]));
////    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", des_pos[2]));
////    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",yaw));
////    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",-roll));
////    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",-pitch));
////    //ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
//////
////    if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
////    {
////        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
////    }
////    else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
////    {
////        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
////    }


}

void WalkingDialog::on_BTN_SET_FOOT_PRINT_clicked()
{
//    on_BTN_FILL_1ST_DSP_clicked();
//    QString str;
//    //---Foot position control variable . 0<= Xc <= 1, 0<= Yc <= 1
//    //---Xc and Yc are ratio of foot position on block. bic Xc means bic step length in x direction, bic Yc means bic step length in y direction.
//    double Xc = 0.19f/0.38f,Yc=0.13f/0.38f;
//    double step_threshold  = 0.6f;
//    double RF_ORI_[4][3]={{0,},},LF_ORI_[4][3]={{0,},},X_[3]={0,},Y_[3]={0,},Z_[3]={0,};
//    double VISION_RMAT[3][3]={{0,},},RP_roll=0.0f,LP_roll=0.0f,RP_pitch=0.0f,LP_pitch=0.0f,RP_yaw=0.0f,LP_yaw=0.0f;
//    double th = (ui->LE_STEP_LENGTH_RIGHT_YAW_1->text().toDouble() + ui->LE_STEP_LENGTH_LEFT_YAW_1->text().toDouble())/2.;
//    double inner_pos[3]={0,},outer_pos[3]={0,},outer_vec[3]={0,},inner_vec[3]={0,},cross_vec[3]={0,},RP_des_pos[3]={0,},LP_des_pos[3]={0,},RF_des_pos[3]={0,},LF_des_pos[3]={0,},RF_des_ori[3]={0,},LF_des_ori[3]={0,};
//    double RF_center_pos[9][3]={{0,},},LF_center_pos[9][3]={{0,},};
//    double TR[3][3] = {{1,0,0},{0,1,0},{0,0,1}},waist_theta = -180.0f;
//    double temp_com=0.0f;


//    //===========================have to exist one plane at least
//    if(VisionData.Floor_num >= 1)
//    {

//        //-------------------------transformed Right and Left plane center postion in robot coordinate
//        LF_center_pos[0][0] = ((VisionData.LF_center[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_center[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//        LF_center_pos[0][1] = ((VisionData.LF_center[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_center[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//        LF_center_pos[0][2] = VisionData.LF_center[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//        RF_center_pos[0][0] = ((VisionData.RF_center[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_center[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//        RF_center_pos[0][1] = ((VisionData.RF_center[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_center[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//        RF_center_pos[0][2] = VisionData.RF_center[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//        cout<<"Step_Threshold :"<<step_threshold<<endl;
//        cout<<"Left plane center pos"<<LF_center_pos[0][0]<<endl;
//        cout<<"Right plane center pos"<<RF_center_pos[0][0]<<endl;

//        if(LF_center_pos[0][0] < step_threshold && LF_center_pos[0][0] < step_threshold && VisionData.Floor_num>=2)
//        {
//            //====================transfer floor number (current 1 or 2)to Freewalking for checking the collision check .
//            //==================== later you have to change the number for N step walking.
//            pLAN->G2MData->block_num = 2;

//            //===============================================Left Plane====================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> Two plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            cout << ">>>>>>>>>>>>>>>>>>> Left Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//            LF_ORI_[0][0] = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[0][1] =((VisionData.LF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[0][2] = VisionData.LF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[1][0] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[1][1] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[1][2] = VisionData.LF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[2][0] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[2][1] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[2][2] = VisionData.LF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[3][0] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[3][1] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[3][2] = VisionData.LF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            cout << "Transform Left plane p1 \n"<<"x:"<<LF_ORI_[0][0]<<"y:"<<LF_ORI_[0][1]<<"z:"<<LF_ORI_[0][2]<<endl;
//            cout << "Transform Left plane p2 \n"<<"x:"<<LF_ORI_[1][0]<<"y:"<<LF_ORI_[1][1]<<"z:"<<LF_ORI_[1][2]<<endl;
//            cout << "Transform Left plane p3 \n"<<"x:"<<LF_ORI_[2][0]<<"y:"<<LF_ORI_[2][1]<<"z:"<<LF_ORI_[2][2]<<endl;
//            cout << "Transform Left plane p4 \n"<<"x:"<<LF_ORI_[3][0]<<"y:"<<LF_ORI_[3][1]<<"z:"<<LF_ORI_[3][2]<<endl;

//            //------------------linear equation from close to far(x direction) and inner to outer(y direction) in Left block

//            //------------------ direction vector
//            outer_vec[0] = LF_ORI_[2][0] - LF_ORI_[0][0];
//            outer_vec[1] = LF_ORI_[2][1] - LF_ORI_[0][1];
//            outer_vec[2] = LF_ORI_[2][2] - LF_ORI_[0][2];

//            inner_vec[0] = LF_ORI_[3][0] - LF_ORI_[1][0];
//            inner_vec[1] = LF_ORI_[3][1] - LF_ORI_[1][1];
//            inner_vec[2] = LF_ORI_[3][2] - LF_ORI_[1][2];

//            //------------------inner and outer linear equation , decision foot position on Left block
//            inner_pos[0] = LF_ORI_[1][0] + inner_vec[0]*Xc;
//            inner_pos[1] = LF_ORI_[1][1] + inner_vec[1]*Xc;
//            inner_pos[2] = LF_ORI_[1][2] + inner_vec[2]*Xc;

//            outer_pos[0] = LF_ORI_[0][0] + outer_vec[0]*Xc;
//            outer_pos[1] = LF_ORI_[0][1] + outer_vec[1]*Xc;
//            outer_pos[2] = LF_ORI_[0][2] + outer_vec[2]*Xc;

//            //------------------inner to outer vector (- y direction)
//            //-----direction vector

//            cross_vec[0] = outer_pos[0] - inner_pos[0];
//            cross_vec[1] = outer_pos[1] - inner_pos[1];
//            cross_vec[2] = outer_pos[2] - inner_pos[2];

//            //==============================Desired Left Plane Pos

//            LP_des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//            LP_des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//            LP_des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<LP_des_pos[0]<<"y print:"<<LP_des_pos[1]<<"z print:"<<LP_des_pos[2]<<endl;


//            //------------------Orientation
//            X_[0] = VisionData.LF_vec[0][0]*cos(th*D2R) - VisionData.LF_vec[0][1]*sin(th*D2R);
//            X_[1] = VisionData.LF_vec[0][0]*sin(th*D2R) + VisionData.LF_vec[0][1]*cos(th*D2R);
//            X_[2] = VisionData.LF_vec[0][2];

//            Y_[0] = VisionData.LF_vec[1][0]*cos(th*D2R) - VisionData.LF_vec[1][1]*sin(th*D2R);
//            Y_[1] = VisionData.LF_vec[1][0]*sin(th*D2R) + VisionData.LF_vec[1][1]*cos(th*D2R);
//            Y_[2] = VisionData.LF_vec[1][2];

//            Z_[0] = VisionData.LF_vec[2][0]*cos(th*D2R) - VisionData.LF_vec[2][1]*sin(th*D2R);
//            Z_[1] = VisionData.LF_vec[2][0]*sin(th*D2R) + VisionData.LF_vec[2][1]*cos(th*D2R);
//            Z_[2] = VisionData.LF_vec[2][2];

//            cout << th <<endl;
//            cout << "normal vec  \n"<<"xX:"<<VisionData.LF_vec[0][0]<<"xY:"<<VisionData.LF_vec[0][1]<<"xZ:"<<VisionData.LF_vec[0][2]<<endl;
//            cout << "normal vec \n"<<"yX:"<<VisionData.LF_vec[1][0]<<"yY:"<<VisionData.LF_vec[1][1]<<"yZ:"<<VisionData.LF_vec[1][2]<<endl;
//            cout << "normal vec \n"<<"zX:"<<VisionData.LF_vec[2][0]<<"zY:"<<VisionData.LF_vec[2][1]<<"zZ:"<<VisionData.LF_vec[2][2]<<endl;

//            //---------------------------------------------
//            VISION_RMAT[0][0] = X_[0];
//            VISION_RMAT[1][0] = X_[1];
//            VISION_RMAT[2][0] = X_[2];

//            VISION_RMAT[0][1] = Y_[0];
//            VISION_RMAT[1][1] = Y_[1];
//            VISION_RMAT[2][1] = Y_[2];

//            VISION_RMAT[0][2] = Z_[0];
//            VISION_RMAT[1][2] = Z_[1];
//            VISION_RMAT[2][2] = Z_[2];


//            //==============================Desired Left orientation

//            LP_roll    = asin(VISION_RMAT[2][1])*R2D;
//            LP_yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//            LP_pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//             cout << "Left Block orientation  \n"<<"roll:"<<LP_roll<<"yaw:"<<LP_yaw<<"pitch:"<<LP_pitch<<endl;




//            //===============================================Left Plane====================================================//


//            //===============================================Right Plane===================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> Right Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//            RF_ORI_[0][0]= ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[0][1] =((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[0][2] =VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[1][0] =((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[1][1] =((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[1][2] =VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[2][0] =((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[2][1] =((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[2][2] =VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[3][0] =((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[3][1] =((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[3][2] =VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            cout << "Transform Right plane p1 \n"<<"x:"<<RF_ORI_[0][0]<<"y:"<<RF_ORI_[0][1]<<"z:"<<RF_ORI_[0][2]<<endl;
//            cout << "Transform Right plane p2 \n"<<"x:"<<RF_ORI_[1][0]<<"y:"<<RF_ORI_[1][1]<<"z:"<<RF_ORI_[1][2]<<endl;
//            cout << "Transform Right plane p3 \n"<<"x:"<<RF_ORI_[2][0]<<"y:"<<RF_ORI_[2][1]<<"z:"<<RF_ORI_[2][2]<<endl;
//            cout << "Transform Right plane p4 \n"<<"x:"<<RF_ORI_[3][0]<<"y:"<<RF_ORI_[3][1]<<"z:"<<RF_ORI_[3][2]<<endl;

//            //------------------linear equation from close to far(x direction) and inner to outer(y direction) in right block

//            //------------------ direction vector
//            inner_vec[0] = RF_ORI_[2][0] - RF_ORI_[0][0];
//            inner_vec[1] = RF_ORI_[2][1] - RF_ORI_[0][1];
//            inner_vec[2] = RF_ORI_[2][2] - RF_ORI_[0][2];

//            outer_vec[0] = RF_ORI_[3][0] - RF_ORI_[1][0];
//            outer_vec[1] = RF_ORI_[3][1] - RF_ORI_[1][1];
//            outer_vec[2] = RF_ORI_[3][2] - RF_ORI_[1][2];

//            //------------------inner and outer linear equation , decision foot position on right block
//            inner_pos[0] = RF_ORI_[0][0] + inner_vec[0]*Xc;
//            inner_pos[1] = RF_ORI_[0][1] + inner_vec[1]*Xc;
//            inner_pos[2] = RF_ORI_[0][2] + inner_vec[2]*Xc;

//            outer_pos[0] = RF_ORI_[1][0] + outer_vec[0]*Xc;
//            outer_pos[1] = RF_ORI_[1][1] + outer_vec[1]*Xc;
//            outer_pos[2] = RF_ORI_[1][2] + outer_vec[2]*Xc;

//            //------------------inner to outer vector (- y direction)
//            //-----direction vector

//            cross_vec[0] = outer_pos[0] - inner_pos[0];
//            cross_vec[1] = outer_pos[1] - inner_pos[1];
//            cross_vec[2] = outer_pos[2] - inner_pos[2];


//            //=========================Desired Right Plane pos
//            RP_des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//            RP_des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//            RP_des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<RP_des_pos[0]<<"y print:"<<RP_des_pos[1]<<"z print:"<<RP_des_pos[2]<<endl;


//            //------------------Orientation
//            X_[0] = VisionData.RF_vec[0][0]*cos(th*D2R) - VisionData.RF_vec[0][1]*sin(th*D2R);
//            X_[1] = VisionData.RF_vec[0][0]*sin(th*D2R) + VisionData.RF_vec[0][1]*cos(th*D2R);
//            X_[2] = VisionData.RF_vec[0][2];

//            Y_[0] = VisionData.RF_vec[1][0]*cos(th*D2R) - VisionData.RF_vec[1][1]*sin(th*D2R);
//            Y_[1] = VisionData.RF_vec[1][0]*sin(th*D2R) + VisionData.RF_vec[1][1]*cos(th*D2R);
//            Y_[2] = VisionData.RF_vec[1][2];

//            Z_[0] = VisionData.RF_vec[2][0]*cos(th*D2R) - VisionData.RF_vec[2][1]*sin(th*D2R);
//            Z_[1] = VisionData.RF_vec[2][0]*sin(th*D2R) + VisionData.RF_vec[2][1]*cos(th*D2R);
//            Z_[2] = VisionData.RF_vec[2][2];

//            cout << th <<endl;
//            cout << "normal vec  \n"<<"xX:"<<VisionData.RF_vec[0][0]<<"xY:"<<VisionData.RF_vec[0][1]<<"xZ:"<<VisionData.RF_vec[0][2]<<endl;
//            cout << "normal vec \n"<<"yX:"<<VisionData.RF_vec[1][0]<<"yY:"<<VisionData.RF_vec[1][1]<<"yZ:"<<VisionData.RF_vec[1][2]<<endl;
//            cout << "normal vec \n"<<"zX:"<<VisionData.RF_vec[2][0]<<"zY:"<<VisionData.RF_vec[2][1]<<"zZ:"<<VisionData.RF_vec[2][2]<<endl;

//            //---------------------------------------------
//             VISION_RMAT[0][0] = X_[0];
//             VISION_RMAT[1][0] = X_[1];
//             VISION_RMAT[2][0] = X_[2];

//             VISION_RMAT[0][1] = Y_[0];
//             VISION_RMAT[1][1] = Y_[1];
//             VISION_RMAT[2][1] = Y_[2];

//             VISION_RMAT[0][2] = Z_[0];
//             VISION_RMAT[1][2] = Z_[1];
//             VISION_RMAT[2][2] = Z_[2];

//             RP_roll    = asin(VISION_RMAT[2][1])*R2D;
//             RP_yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//             RP_pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//             cout << "Right Block orientation  \n"<<"roll:"<<RP_roll<<"yaw:"<<RP_yaw<<"pitch:"<<RP_pitch<<endl;

//             //===============================================Right Plane===================================================//

//             //===================transfer to motion for collision check.
//             _G2M_block_edge[0][0] = -LF_ORI_[0][0];// = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[0][1] = -LF_ORI_[0][1];// =((VisionData.LF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[0][2] = LF_ORI_[0][2];// = VisionData.LF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[0][3] = -LF_ORI_[1][0];// = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[0][4] = -LF_ORI_[1][1];// = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[0][5] = LF_ORI_[1][2];// = VisionData.LF_mz[0][5] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[0][6] = -LF_ORI_[2][0];// = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[0][7] = -LF_ORI_[2][1];// = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[0][8] = LF_ORI_[2][2];//  =VisionData.LF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[0][9] = -LF_ORI_[3][0];// = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[0][10] = -LF_ORI_[3][1];// = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[0][11] = LF_ORI_[3][2];// = VisionData.LF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[1][0] = -RF_ORI_[0][0];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[1][1] = -RF_ORI_[0][1];// =((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[1][2] = RF_ORI_[0][2];// =VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[1][3] = -RF_ORI_[1][0];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[1][4] = -RF_ORI_[1][1];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[1][5] = RF_ORI_[1][2];// =VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[1][6] = -RF_ORI_[2][0];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[1][7] = -RF_ORI_[2][1];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[1][8] = RF_ORI_[2][2];// =VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//             _G2M_block_edge[1][9] = -RF_ORI_[3][0];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//             _G2M_block_edge[1][10] = -RF_ORI_[3][1];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//             _G2M_block_edge[1][11] = RF_ORI_[3][2];// =VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];


//        }else if(LF_center_pos[0][0] < step_threshold)
//        {


//            pLAN->G2MData->block_num = 1;

//            //===============================================Left Plane====================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> just one Left Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//            LF_ORI_[0][0] = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[0][1] = ((VisionData.LF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[0][2] = VisionData.LF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[1][0] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[1][1] = ((VisionData.LF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[1][2] = VisionData.LF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[2][0] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[2][1] = ((VisionData.LF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[2][2] = VisionData.LF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            LF_ORI_[3][0] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.LF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            LF_ORI_[3][1] = ((VisionData.LF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.LF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            LF_ORI_[3][2] = VisionData.LF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            cout << "Transform Left plane p1 \n"<<"x:"<<LF_ORI_[0][0]<<"y:"<<LF_ORI_[0][1]<<"z:"<<LF_ORI_[0][2]<<endl;
//            cout << "Transform Left plane p2 \n"<<"x:"<<LF_ORI_[1][0]<<"y:"<<LF_ORI_[1][1]<<"z:"<<LF_ORI_[1][2]<<endl;
//            cout << "Transform Left plane p3 \n"<<"x:"<<LF_ORI_[2][0]<<"y:"<<LF_ORI_[2][1]<<"z:"<<LF_ORI_[2][2]<<endl;
//            cout << "Transform Left plane p4 \n"<<"x:"<<LF_ORI_[3][0]<<"y:"<<LF_ORI_[3][1]<<"z:"<<LF_ORI_[3][2]<<endl;

//            //------------------linear equation from close to far(x direction) and inner to outer(y direction) in Left block

//            //------------------ direction vector
//            outer_vec[0] = LF_ORI_[2][0] - LF_ORI_[0][0];
//            outer_vec[1] = LF_ORI_[2][1] - LF_ORI_[0][1];
//            outer_vec[2] = LF_ORI_[2][2] - LF_ORI_[0][2];

//            inner_vec[0] = LF_ORI_[3][0] - LF_ORI_[1][0];
//            inner_vec[1] = LF_ORI_[3][1] - LF_ORI_[1][1];
//            inner_vec[2] = LF_ORI_[3][2] - LF_ORI_[1][2];

//            //------------------inner and outer linear equation , decision foot position on Left block
//            inner_pos[0] = LF_ORI_[1][0] + inner_vec[0]*Xc;
//            inner_pos[1] = LF_ORI_[1][1] + inner_vec[1]*Xc;
//            inner_pos[2] = LF_ORI_[1][2] + inner_vec[2]*Xc;

//            outer_pos[0] = LF_ORI_[0][0] + outer_vec[0]*Xc;
//            outer_pos[1] = LF_ORI_[0][1] + outer_vec[1]*Xc;
//            outer_pos[2] = LF_ORI_[0][2] + outer_vec[2]*Xc;

//            //------------------inner to outer vector (- y direction)
//            //-----direction vector

//            cross_vec[0] = outer_pos[0] - inner_pos[0];
//            cross_vec[1] = outer_pos[1] - inner_pos[1];
//            cross_vec[2] = outer_pos[2] - inner_pos[2];

//            //==============================Desired Left Plane Pos

//            LP_des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//            LP_des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//            LP_des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<LP_des_pos[0]<<"y print:"<<LP_des_pos[1]<<"z print:"<<LP_des_pos[2]<<endl;


//            //------------------Orientation
//            X_[0] = VisionData.LF_vec[0][0]*cos(th*D2R) - VisionData.LF_vec[0][1]*sin(th*D2R);
//            X_[1] = VisionData.LF_vec[0][0]*sin(th*D2R) + VisionData.LF_vec[0][1]*cos(th*D2R);
//            X_[2] = VisionData.LF_vec[0][2];

//            Y_[0] = VisionData.LF_vec[1][0]*cos(th*D2R) - VisionData.LF_vec[1][1]*sin(th*D2R);
//            Y_[1] = VisionData.LF_vec[1][0]*sin(th*D2R) + VisionData.LF_vec[1][1]*cos(th*D2R);
//            Y_[2] = VisionData.LF_vec[1][2];

//            Z_[0] = VisionData.LF_vec[2][0]*cos(th*D2R) - VisionData.LF_vec[2][1]*sin(th*D2R);
//            Z_[1] = VisionData.LF_vec[2][0]*sin(th*D2R) + VisionData.LF_vec[2][1]*cos(th*D2R);
//            Z_[2] = VisionData.LF_vec[2][2];

//            cout << th <<endl;
//            cout << "normal vec  \n"<<"xX:"<<VisionData.LF_vec[0][0]<<"xY:"<<VisionData.LF_vec[0][1]<<"xZ:"<<VisionData.LF_vec[0][2]<<endl;
//            cout << "normal vec \n"<<"yX:"<<VisionData.LF_vec[1][0]<<"yY:"<<VisionData.LF_vec[1][1]<<"yZ:"<<VisionData.LF_vec[1][2]<<endl;
//            cout << "normal vec \n"<<"zX:"<<VisionData.LF_vec[2][0]<<"zY:"<<VisionData.LF_vec[2][1]<<"zZ:"<<VisionData.LF_vec[2][2]<<endl;

//            //---------------------------------------------
//            VISION_RMAT[0][0] = X_[0];
//            VISION_RMAT[1][0] = X_[1];
//            VISION_RMAT[2][0] = X_[2];

//            VISION_RMAT[0][1] = Y_[0];
//            VISION_RMAT[1][1] = Y_[1];
//            VISION_RMAT[2][1] = Y_[2];

//            VISION_RMAT[0][2] = Z_[0];
//            VISION_RMAT[1][2] = Z_[1];
//            VISION_RMAT[2][2] = Z_[2];


//            //==============================Desired Left orientation

//            LP_roll    = asin(VISION_RMAT[2][1])*R2D;
//            LP_yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//            LP_pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//            cout << "Left Block orientation  \n"<<"roll:"<<LP_roll<<"yaw:"<<LP_yaw<<"pitch:"<<LP_pitch<<endl;
//            //===============================================Left Plane====================================================//

//            //===============================================Right Plane===================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> Right Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            RP_des_pos[0] =  LP_des_pos[0];
//            RP_des_pos[1] =  LP_des_pos[1] - 0.26f;
//            RP_des_pos[2] =  0.0f;

//            RP_roll = 0.0f;
//            RP_yaw = 0.0f;
//            RP_pitch = 0.0f;
//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<RP_des_pos[0]<<"y print:"<<RP_des_pos[1]<<"z print:"<<RP_des_pos[2]<<endl;
//            cout << "Right Block orientation \n"<<"roll:"<<RP_roll<<"yaw:"<<RP_yaw<<"pitch:"<<RP_pitch<<endl;
//            //===============================================Right Plane===================================================//


//            //============================transfer to motion


//            _G2M_block_edge[0][0] = -LF_ORI_[0][0];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][1] = -LF_ORI_[0][1];// =((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][2] = LF_ORI_[0][2];// =VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][3] = -LF_ORI_[1][0];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][4] = -LF_ORI_[1][1];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][5] = LF_ORI_[1][2];// =VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][6] = -LF_ORI_[2][0];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][7] = -LF_ORI_[2][1];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][8] = LF_ORI_[2][2];// =VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][9] = -LF_ORI_[3][0];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][10] = -LF_ORI_[3][1];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][11] = LF_ORI_[3][2];// =VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];


//            _G2M_block_edge[1][0] = -LF_ORI_[0][0];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][1] = -LF_ORI_[0][1];// _G2M_block_edge[1][1];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][2] = LF_ORI_[0][2];// = VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][3] = -LF_ORI_[1][0] ;// = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][4] = -LF_ORI_[1][1];// = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][5] = LF_ORI_[1][2];// = VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][6] = -LF_ORI_[2][0];// = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][7] = -LF_ORI_[2][1];// = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][8] = LF_ORI_[2][2];// = VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][9] = -LF_ORI_[3][0];// = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][10] = -LF_ORI_[3][1];// = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][11] = LF_ORI_[3][2];// = VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];


//        }else if(RF_center_pos[0][0] < step_threshold)
//        {
//            //===============================================Right Plane===================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> Right Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            //------------------left to right and down to up sequence RF_ORI_[edge][xyz]
//            RF_ORI_[0][0] = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[0][1] = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[0][2] = VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[1][0] = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[1][1] = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[1][2] = VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[2][0] = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[2][1] = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[2][2] = VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            RF_ORI_[3][0] = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            RF_ORI_[3][1] = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            RF_ORI_[3][2] = VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            cout << "Transform Right plane p1 \n"<<"x:"<<RF_ORI_[0][0]<<"y:"<<RF_ORI_[0][1]<<"z:"<<RF_ORI_[0][2]<<endl;
//            cout << "Transform Right plane p2 \n"<<"x:"<<RF_ORI_[1][0]<<"y:"<<RF_ORI_[1][1]<<"z:"<<RF_ORI_[1][2]<<endl;
//            cout << "Transform Right plane p3 \n"<<"x:"<<RF_ORI_[2][0]<<"y:"<<RF_ORI_[2][1]<<"z:"<<RF_ORI_[2][2]<<endl;
//            cout << "Transform Right plane p4 \n"<<"x:"<<RF_ORI_[3][0]<<"y:"<<RF_ORI_[3][1]<<"z:"<<RF_ORI_[3][2]<<endl;

//            //------------------linear equation from close to far(x direction) and inner to outer(y direction) in right block

//            //------------------ direction vector
//            inner_vec[0] = RF_ORI_[2][0] - RF_ORI_[0][0];
//            inner_vec[1] = RF_ORI_[2][1] - RF_ORI_[0][1];
//            inner_vec[2] = RF_ORI_[2][2] - RF_ORI_[0][2];

//            outer_vec[0] = RF_ORI_[3][0] - RF_ORI_[1][0];
//            outer_vec[1] = RF_ORI_[3][1] - RF_ORI_[1][1];
//            outer_vec[2] = RF_ORI_[3][2] - RF_ORI_[1][2];

//            //------------------inner and outer linear equation , decision foot position on right block
//            inner_pos[0] = RF_ORI_[0][0] + inner_vec[0]*Xc;
//            inner_pos[1] = RF_ORI_[0][1] + inner_vec[1]*Xc;
//            inner_pos[2] = RF_ORI_[0][2] + inner_vec[2]*Xc;

//            outer_pos[0] = RF_ORI_[1][0] + outer_vec[0]*Xc;
//            outer_pos[1] = RF_ORI_[1][1] + outer_vec[1]*Xc;
//            outer_pos[2] = RF_ORI_[1][2] + outer_vec[2]*Xc;

//            //------------------inner to outer vector (- y direction)
//            //-----direction vector

//            cross_vec[0] = outer_pos[0] - inner_pos[0];
//            cross_vec[1] = outer_pos[1] - inner_pos[1];
//            cross_vec[2] = outer_pos[2] - inner_pos[2];


//            //=========================Desired Right Plane pos
//            RP_des_pos[0] = inner_pos[0] + cross_vec[0]*Yc;
//            RP_des_pos[1] = inner_pos[1] + cross_vec[1]*Yc;
//            RP_des_pos[2] = inner_pos[2] + cross_vec[2]*Yc;

//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<RP_des_pos[0]<<"y print:"<<RP_des_pos[1]<<"z print:"<<RP_des_pos[2]<<endl;


//            //------------------Orientation
//            X_[0] = VisionData.RF_vec[0][0]*cos(th*D2R) - VisionData.RF_vec[0][1]*sin(th*D2R);
//            X_[1] = VisionData.RF_vec[0][0]*sin(th*D2R) + VisionData.RF_vec[0][1]*cos(th*D2R);
//            X_[2] = VisionData.RF_vec[0][2];

//            Y_[0] = VisionData.RF_vec[1][0]*cos(th*D2R) - VisionData.RF_vec[1][1]*sin(th*D2R);
//            Y_[1] = VisionData.RF_vec[1][0]*sin(th*D2R) + VisionData.RF_vec[1][1]*cos(th*D2R);
//            Y_[2] = VisionData.RF_vec[1][2];

//            Z_[0] = VisionData.RF_vec[2][0]*cos(th*D2R) - VisionData.RF_vec[2][1]*sin(th*D2R);
//            Z_[1] = VisionData.RF_vec[2][0]*sin(th*D2R) + VisionData.RF_vec[2][1]*cos(th*D2R);
//            Z_[2] = VisionData.RF_vec[2][2];

//            cout << th <<endl;
//            cout << "normal vec  \n"<<"xX:"<<VisionData.RF_vec[0][0]<<"xY:"<<VisionData.RF_vec[0][1]<<"xZ:"<<VisionData.RF_vec[0][2]<<endl;
//            cout << "normal vec \n"<<"yX:"<<VisionData.RF_vec[1][0]<<"yY:"<<VisionData.RF_vec[1][1]<<"yZ:"<<VisionData.RF_vec[1][2]<<endl;
//            cout << "normal vec \n"<<"zX:"<<VisionData.RF_vec[2][0]<<"zY:"<<VisionData.RF_vec[2][1]<<"zZ:"<<VisionData.RF_vec[2][2]<<endl;

//            //---------------------------------------------
//             VISION_RMAT[0][0] = X_[0];
//             VISION_RMAT[1][0] = X_[1];
//             VISION_RMAT[2][0] = X_[2];

//             VISION_RMAT[0][1] = Y_[0];
//             VISION_RMAT[1][1] = Y_[1];
//             VISION_RMAT[2][1] = Y_[2];

//             VISION_RMAT[0][2] = Z_[0];
//             VISION_RMAT[1][2] = Z_[1];
//             VISION_RMAT[2][2] = Z_[2];

//             RP_roll    = asin(VISION_RMAT[2][1])*R2D;
//             RP_yaw     = atan2(-VISION_RMAT[0][1],VISION_RMAT[1][1])*R2D;
//             RP_pitch   = atan2(-VISION_RMAT[2][0],VISION_RMAT[2][2])*R2D;

//            cout << "Right Block orientation  \n"<<"roll:"<<RP_roll<<"yaw:"<<RP_yaw<<"pitch:"<<RP_pitch<<endl;

//            //===============================================Right Plane===================================================//


//            //===============================================Left Plane===================================================//
//            cout << ">>>>>>>>>>>>>>>>>>> Left Plane <<<<<<<<<<<<<<<<  \n" <<endl;
//            LP_des_pos[0] =  RP_des_pos[0];
//            LP_des_pos[1] =  RP_des_pos[1] + 0.26;
//            LP_des_pos[2] =  0.0f;

//            LP_roll = 0.0f;
//            LP_yaw = 0.0f;
//            LP_pitch = 0.0f;

//            cout << "Foot print \n" <<endl;
//            cout <<"x print:"<<LP_des_pos[0]<<"y print:"<<LP_des_pos[1]<<"z print:"<<LP_des_pos[2]<<endl;
//            cout << "Left Block orientation \n"<<"roll:"<<LP_roll<<"yaw:"<<LP_yaw<<"pitch:"<<LP_pitch<<endl;
//            //===============================================Left Plane===================================================//

//            //============================transfer to motion for collision check
//            _G2M_block_edge[1][0] = -RF_ORI_[0][0];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][1] = -RF_ORI_[0][1];// _G2M_block_edge[1][1];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][2] = RF_ORI_[0][2]; // = VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][3] = -RF_ORI_[1][0] ;// = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][4] = -RF_ORI_[1][1];// = ((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][5] = RF_ORI_[1][2];// = VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][6] = -RF_ORI_[2][0];// = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][7] = -RF_ORI_[2][1];// = ((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][8] = RF_ORI_[2][2];// = VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[1][9] = -RF_ORI_[3][0];// = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[1][10] = -RF_ORI_[3][1];// = ((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[1][11] = RF_ORI_[3][2];// = VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];


//            _G2M_block_edge[0][0] = -RF_ORI_[0][0];// = ((VisionData.RF_mx[0][0] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][0] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][1] = -RF_ORI_[0][1];// =((VisionData.RF_mx[0][0] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][0] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][2] = RF_ORI_[0][2];// =VisionData.RF_mz[0][0] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][3] = -RF_ORI_[1][0];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][1] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][4] = -RF_ORI_[1][1];// =((VisionData.RF_mx[0][1] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][1] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][5] = RF_ORI_[1][2];// =VisionData.RF_mz[0][1] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][6] = -RF_ORI_[2][0];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][2] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][7] = -RF_ORI_[2][1];// =((VisionData.RF_mx[0][2] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][2] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][8] = RF_ORI_[2][2];// =VisionData.RF_mz[0][2] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            _G2M_block_edge[0][9] = -RF_ORI_[3][0];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*cos(th*D2R) - (VisionData.RF_my[0][3] + VisionData.offset[1])*sin(th*D2R)) + -PODO_DATA.UserM2G.curPEL[0];
//            _G2M_block_edge[0][10] = -RF_ORI_[3][1];// =((VisionData.RF_mx[0][3] + VisionData.offset[0])*sin(th*D2R) + (VisionData.RF_my[0][3] + VisionData.offset[1])*cos(th*D2R)) + -PODO_DATA.UserM2G.curPEL[1];
//            _G2M_block_edge[0][11] = RF_ORI_[3][2];// =VisionData.RF_mz[0][3] + VisionData.offset[2] + PODO_DATA.UserM2G.curPEL[2];

//            pLAN->G2MData->block_num = 1;
//        }


//        //=================================Decide right and left foot sequence
//        //--- differce between right and left z direction position. small difference is the first step condition.

//        //====================robot walk backward ... we have to consider the waist orientation.
//        //we have to reflect a waist_theta when a waist is changed. so I`ll make a variable .

//        TR[0][0] = cos(waist_theta*D2R);
//        TR[1][0] = sin(waist_theta*D2R);
//        TR[2][0] = 0.0f;

//        TR[0][1] = -sin(waist_theta*D2R);
//        TR[1][1] = cos(waist_theta*D2R);
//        TR[2][1] = 0.0f;

//        TR[0][2] = 0.0f;
//        TR[1][2] = 0.0f;
//        TR[2][2] = 1.0f;

//        RF_des_pos[0] = LP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        RF_des_pos[1] = LP_des_pos[0]*TR[1][0] + LP_des_pos[1]*TR[1][1] +LP_des_pos[2]*TR[1][2];
//        RF_des_pos[2] = LP_des_pos[0]*TR[2][0] + LP_des_pos[1]*TR[2][1] +LP_des_pos[2]*TR[2][2];

//        _RF_POS[0] = LP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        _RF_POS[1] = LP_des_pos[0]*TR[1][0] + LP_des_pos[1]*TR[1][1] +LP_des_pos[2]*TR[1][2];
//        _RF_POS[2] = LP_des_pos[0]*TR[2][0] + LP_des_pos[1]*TR[2][1] +LP_des_pos[2]*TR[2][2];

//        cout<<"_RF_POS!!!"<<endl;
//        cout<<_RF_POS[0]<<endl;
//        cout<<_RF_POS[1]<<endl;
//        cout<<_RF_POS[2]<<endl;

//        RF_des_ori[2] = _RF_ORI[2] = -LP_pitch; //LP_roll[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        RF_des_ori[1] = _RF_ORI[1] = -LP_roll;//LP_des_pos[0]*TR[1][0] + LP_des_pos[1]*TR[1][1] +LP_des_pos[2]*TR[1][2];
//        RF_des_ori[0] = _RF_ORI[0] = LP_yaw;//LP_des_pos[0]*TR[2][0] + LP_des_pos[1]*TR[2][1] +LP_des_pos[2]*TR[2][2];

//        LF_des_pos[0] = RP_des_pos[0]*TR[0][0] + RP_des_pos[1]*TR[0][1] + RP_des_pos[2]*TR[0][2];
//        LF_des_pos[1] = RP_des_pos[0]*TR[1][0] + RP_des_pos[1]*TR[1][1] + RP_des_pos[2]*TR[1][2];
//        LF_des_pos[2] = RP_des_pos[0]*TR[2][0] + RP_des_pos[1]*TR[2][1] + RP_des_pos[2]*TR[2][2];

//        _LF_POS[0] = RP_des_pos[0]*TR[0][0] + RP_des_pos[1]*TR[0][1] + RP_des_pos[2]*TR[0][2];
//        _LF_POS[1] = RP_des_pos[0]*TR[1][0] + RP_des_pos[1]*TR[1][1] + RP_des_pos[2]*TR[1][2];
//        _LF_POS[2] = RP_des_pos[0]*TR[2][0] + RP_des_pos[1]*TR[2][1] + RP_des_pos[2]*TR[2][2];

//        cout<<"_LF_POS!!!"<<endl;
//        cout<<_LF_POS[0]<<endl;
//        cout<<_LF_POS[1]<<endl;
//        cout<<_LF_POS[2]<<endl;

//        LF_des_ori[2] = _LF_ORI[2] = -RP_pitch;//RP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        LF_des_ori[1] = _LF_ORI[1] = -RP_roll;//RP_des_pos[0]*TR[1][0] + RP_des_pos[1]*TR[1][1] +RP_des_pos[2]*TR[1][2];
//        LF_des_ori[0] = _LF_ORI[0] = RP_yaw;//RP_des_pos[0]*TR[2][0] + RP_des_pos[1]*TR[2][1] +RP_des_pos[2]*TR[2][2];


//        if(fabs(_RF_POS[2] - ui->LE_STEP_LENGTH_LEFT_Z1->text().toDouble()) < fabs(LF_des_pos[2] - ui->LE_STEP_LENGTH_RIGHT_Z1->text().toDouble()))
//        {
//            // RIGHT FOOT FIRST!!!
//            cout<<"Right foot first!!!"<<endl;
//            cout<<"Right foot first!!!"<<endl;
//            cout<<"Right foot first!!!"<<endl;
//            //===================================second DSP========================================//

//            ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", _RF_POS[0]));
//            ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", _RF_POS[1]));
//            ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", _RF_POS[2]));
//            ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",_RF_ORI[0]));
//            ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",_RF_ORI[1]));
//            ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",_RF_ORI[2]));

////            if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
////            {
////                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
////                cout<<"Right foot down!!!";
////            }
////            else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
////            {
////                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
////                cout<<"Right foot up!!!";
////            }

//            if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
//            {
//                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f", ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
//                cout<<"second dsp 111Right foot down!!!";
//            }
//            else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
//            {
//                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
//                cout<<"second dsp 222Right foot up!!!";
//            }
//            //===================================second DSP========================================//


//            //===================================Third DSP========================================//
//            ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", _RF_POS[0]));
//            ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", _RF_POS[1]));
//            ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", _RF_POS[2]));
//            ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f",_RF_ORI[0]));
//            ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f",_RF_ORI[1]));
//            ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f",_RF_ORI[2]));

//            ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", _LF_POS[0]));
//            ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", _LF_POS[1]));
//            ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", _LF_POS[2]));
//            ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f",_LF_ORI[0]));
//            ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f",_LF_ORI[1]));
//            ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f",_LF_ORI[2]));

////            if(_LF_POS[2] < _RF_POS[2])   //left foot down
////            {
////                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",_LF_POS[2]));
////                cout<<"333left foot down!!!";
////            }
////            else if(_LF_POS[2] > _RF_POS[2])   //left foot up
////            {
////                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",_RF_POS[2]));
////                cout<<"444left foot down!!!";
////            }


//            if(ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble() < ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble())   //left foot down
//            {
//                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble() ));
//                cout<<"third dsp 333left foot down!!!";
//            }
//            else if(ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble() > ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble())   //left foot up
//            {
//                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble()));
//                cout<<"third dsp 444left foot up!!!";
//            }

//            //===================================Third DSP========================================//
//        }else
//        {
//            //===================================second DSP========================================//
//            cout<<"Left foot first!!!"<<endl;
//            cout<<"Left foot first!!!"<<endl;
//            cout<<"Left foot first!!!"<<endl;

//            ui->LE_STEP_LENGTH_LEFT_X2->setText(str.sprintf("%.4f", _LF_POS[0]));
//            ui->LE_STEP_LENGTH_LEFT_Y2->setText(str.sprintf("%.4f", _LF_POS[1]));
//            ui->LE_STEP_LENGTH_LEFT_Z2->setText(str.sprintf("%.4f", _LF_POS[2]));
//            ui->LE_STEP_LENGTH_LEFT_YAW_2->setText(str.sprintf("%.4f",_LF_ORI[0]));
//            ui->LE_STEP_LENGTH_LEFT_ROLL_2->setText(str.sprintf("%.4f",_LF_ORI[1]));
//            ui->LE_STEP_LENGTH_LEFT_PITCH_2->setText(str.sprintf("%.4f",_LF_ORI[2]));

////            if(_LF_POS[2] < ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())//Right foot down
////            {
////                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",_LF_POS[2])); //set com to lower foot
////            }
////            else if(_LF_POS[2] > ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())//Right foot up
////            {
////                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
////            }

//            if(ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())//left foot down
//            {
//                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
//            }
//            else if(ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())//left foot up
//            {
//                ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
//            }
//            //===================================second DSP========================================//


//            //===================================Third DSP========================================//
//            ui->LE_STEP_LENGTH_LEFT_X3->setText(str.sprintf("%.4f", _LF_POS[0]));
//            ui->LE_STEP_LENGTH_LEFT_Y3->setText(str.sprintf("%.4f", _LF_POS[1]));
//            ui->LE_STEP_LENGTH_LEFT_Z3->setText(str.sprintf("%.4f", _LF_POS[2]));
//            ui->LE_STEP_LENGTH_LEFT_YAW_3->setText(str.sprintf("%.4f",_LF_ORI[0]));
//            ui->LE_STEP_LENGTH_LEFT_ROLL_3->setText(str.sprintf("%.4f",_LF_ORI[1]));
//            ui->LE_STEP_LENGTH_LEFT_PITCH_3->setText(str.sprintf("%.4f",_LF_ORI[2]));


//            ui->LE_STEP_LENGTH_RIGHT_X3->setText(str.sprintf("%.4f", _RF_POS[0]));
//            ui->LE_STEP_LENGTH_RIGHT_Y3->setText(str.sprintf("%.4f", _RF_POS[1]));
//            ui->LE_STEP_LENGTH_RIGHT_Z3->setText(str.sprintf("%.4f", _RF_POS[2]));
//            ui->LE_STEP_LENGTH_RIGHT_YAW_3->setText(str.sprintf("%.4f",_RF_ORI[0]));
//            ui->LE_STEP_LENGTH_RIGHT_ROLL_3->setText(str.sprintf("%.4f",_RF_ORI[1]));
//            ui->LE_STEP_LENGTH_RIGHT_PITCH_3->setText(str.sprintf("%.4f",_RF_ORI[2]));

////            if(_RF_POS[2] < _RF_POS[2])   //left foot down
////                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",_RF_POS[2]));
////            else if(_RF_POS[2] > _RF_POS[2])   //left foot up
////                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",_LF_POS[2]));

//            if(ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble())   //left foot down
//                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble()));
//            else if(ui->LE_STEP_LENGTH_RIGHT_Z3->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble())   //left foot up
//                ui->LE_STEP_LEFT_COM_Z3->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z3->text().toDouble()));

//            //===================================Third DSP========================================//
//        }


//    }else
//    {
//        //=============================Just go two step in No plane state

//        pLAN->G2MData->block_num = 0;


//        RF_des_pos[0] = LP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        RF_des_pos[1] = LP_des_pos[0]*TR[1][0] + LP_des_pos[1]*TR[1][1] +LP_des_pos[2]*TR[1][2];
//        RF_des_pos[2] = LP_des_pos[0]*TR[2][0] + LP_des_pos[1]*TR[2][1] +LP_des_pos[2]*TR[2][2];

//        RF_des_ori[0] = 0.0; //LP_roll[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        RF_des_ori[1] = 0.0;//LP_des_pos[0]*TR[1][0] + LP_des_pos[1]*TR[1][1] +LP_des_pos[2]*TR[1][2];
//        RF_des_ori[2] = 0.0;//LP_des_pos[0]*TR[2][0] + LP_des_pos[1]*TR[2][1] +LP_des_pos[2]*TR[2][2];

//        LF_des_pos[0] = RP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        LF_des_pos[1] = RP_des_pos[0]*TR[1][0] + RP_des_pos[1]*TR[1][1] +RP_des_pos[2]*TR[1][2];
//        LF_des_pos[2] = RP_des_pos[0]*TR[2][0] + RP_des_pos[1]*TR[2][1] +RP_des_pos[2]*TR[2][2];

//        LF_des_ori[0] = 0.0;//RP_des_pos[0]*TR[0][0] + LP_des_pos[1]*TR[0][1] +LP_des_pos[2]*TR[0][2];
//        LF_des_ori[1] = 0.0;//RP_des_pos[0]*TR[1][0] + RP_des_pos[1]*TR[1][1] +RP_des_pos[2]*TR[1][2];
//        LF_des_ori[2] = 0.0;;//RP_des_pos[0]*TR[2][0] + RP_des_pos[1]*TR[2][1] +RP_des_pos[2]*TR[2][2];

//        cout<<"No Plane!!! No Plane!!! No Plane!!!"<<endl;
//        cout<<"No Plane!!! No Plane!!! No Plane!!!"<<endl;
//        cout<<"No Plane!!! No Plane!!! No Plane!!!"<<endl;
//    }
}





void WalkingDialog::on_BTN_FILL_RF_clicked()
{
//    ui->LE_STEP_LENGTH_RIGHT_X2->setText(str.sprintf("%.4f", _RF_POS[0]));
//    ui->LE_STEP_LENGTH_RIGHT_Y2->setText(str.sprintf("%.4f", _RF_POS[1]));
//    ui->LE_STEP_LENGTH_RIGHT_Z2->setText(str.sprintf("%.4f", _RF_POS[2]));
//    ui->LE_STEP_LENGTH_RIGHT_YAW_2->setText(str.sprintf("%.4f",_RF_ORI[2]));
//    ui->LE_STEP_LENGTH_RIGHT_ROLL_2->setText(str.sprintf("%.4f",_RF_ORI[1]));
//    ui->LE_STEP_LENGTH_RIGHT_PITCH_2->setText(str.sprintf("%.4f",_RF_ORI[0]));
//    //ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",0.));
////
//    if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() < ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot down
//    {
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble())); //set com to lower foot
//    }
//    else if(ui->LE_STEP_LENGTH_RIGHT_Z2->text().toDouble() > ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())//Right foot up
//    {
//        ui->LE_STEP_LEFT_COM_Z2->setText(str.sprintf("%.4f",ui->LE_STEP_LENGTH_LEFT_Z2->text().toDouble())); //set com to lower foot
//    }

//    usleep(100000);



}

void WalkingDialog::on_BTN_GOAL_GO_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->GoalPosX = ui->LE_VISION_GOALX->text().toDouble();
    pLAN->G2MData->GoalPosY = ui->LE_VISION_GOALY->text().toDouble();
    pLAN->G2MData->GoalAngle = ui->LE_VISION_GOAL_ANGLE->text().toDouble();

    if(ui->RB_GOALGO_NORMAL->isChecked())
        pLAN->G2MData->WalkingGoalGoModeCommand = GOALGO_NORMAL;
    else if(ui->RB_GOALGO_FOR_SIDE->isChecked())
        pLAN->G2MData->WalkingGoalGoModeCommand = GOALGO_FOR_SIDE;
    else if(ui->RB_GOALGO_SIDE_FOR->isChecked())
        pLAN->G2MData->WalkingGoalGoModeCommand = GOALGO_SIDE_FOR;
    else if(ui->RB_GOALGO_DIAG->isChecked())
        pLAN->G2MData->WalkingGoalGoModeCommand = GOALGO_DIAGONAL;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = GOAL_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_FORWARD_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = FORWARD_WALKING;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}
void WalkingDialog::on_BTN_INT_BACKWARD_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = BACKWARD_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_RIGHT_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = RIGHTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}


void WalkingDialog::on_BTN_INT_LEFT_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = LEFTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_CW_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = CWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_CCW_2_clicked()
{
    USER_COMMAND cmd;
    on_BTN_CLEAR_ALL_clicked();
    pLAN->G2MData->WalkingModeCommand = CCWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_DSP_HOLD_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_WALK_READY_POS_WITH_M180_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 10;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INITIALIZE_1step_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_INITIALIZE;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_INT_INITIALIZE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_INITIALIZE;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_DATA_SAVE_2_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = COMM_SEND_DATA;
//    cmd.COMMAND_TARGET = 0;
//    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_WALK_READY_POS_3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 7;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_Foot_up_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = FOOTUP;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_Knee_Down_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = KNEEDOWN;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_Foot_Down_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = FOOTDOWN;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_BOW_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = BOW;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_INFINITY;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_WALK_READY_POS_4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 8;
        cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

        pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_WALK_READY_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = 3050;

    pLAN->SendCommand(cmd);
}

void WalkingDialog::on_BTN_WALK_READY_TEST_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = 3050;

    pLAN->SendCommand(cmd);

}

void WalkingDialog::on_BTN_STOP_WALKING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_STOP;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}
