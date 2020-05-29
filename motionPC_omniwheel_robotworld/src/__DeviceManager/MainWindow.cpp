#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "RBDataBase.h"

// CAN handler
pRBCAN          canHandler;


// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_OF           RBDataBase::_DB_OF[MAX_OF];
DB_AL           RBDataBase::_DB_AL[MAX_AL];


int     _SrvOnFlag = false;
int     _HomeStartFlag = false;
int     _EncoderZeroFlag = false;
int     _HomeSequence = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    IS_CAN_OK = false;
    IS_WORKING = true;

    selectedDevice = -1;


    //int ret = false;
    DBInitialize();
    CANInitialize();
    PanelInitialize();
    ThreadInitialize();

    connect(qApp, SIGNAL(aboutToQuit()), this, SLOT(Closing()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Closing(){
    FILE_LOG(logERROR) << "Closing everything..";
    canHandler->Finish();
    IS_WORKING = false;
    usleep(1000*1000);
}

void MainWindow::RBCore_RTThreadCon(void *arg){
    MainWindow *pMain = (MainWindow*)arg;

    int deviceType = 0;
    int selectedCh = 0;
    int selectedId = 0;

    long cnt = 0;

    rt_task_set_periodic(NULL, TM_NOW, (RT_TIMER_PERIOD_MS)*1000000);

    while(pMain->IS_WORKING){
        rt_task_wait_period(NULL);

        if(pMain->selectedDevice < 0)
            continue;

        if(pMain->selectedDevice < pMain->indexFT){
            deviceType = 0;
        }else{
            deviceType = 1;
        }

        if(deviceType == 0){
            //=======================================
            // Real-time thread for motor controller
            //=======================================

            selectedId = pMain->selectedDevice;
            selectedCh = pMain->_PAN_MC[pMain->selectedDevice]->CurChannel;

            // Read Encoder & Status
            pMain->_DEV_MC[selectedId].RBBoard_ReadEncoderData();
            pMain->_DEV_MC[selectedId].RBBoard_GetStatus();



            // Servo On ---
            if(_SrvOnFlag){
                double enc = pMain->_DEV_MC[selectedId].Joints[selectedCh].CurrentPosition;
                pMain->_DEV_MC[selectedId].Joints[selectedCh].Reference = enc;
                pMain->_DEV_MC[selectedId].MoveJoints[selectedCh].RefAngleCurrent = enc;
                pMain->_DEV_MC[selectedId].RBJoint_EnableFeedbackControl(selectedCh+1, true);
                _SrvOnFlag = false;
            }
            // -------------

            // Encoder Zero ---
            if(_EncoderZeroFlag){
                pMain->_DEV_MC[selectedId].RBJoint_ResetEncoder(selectedCh+1);
                pMain->_DEV_MC[selectedId].Joints[selectedCh].Reference = 0.0;
                pMain->_DEV_MC[selectedId].MoveJoints[selectedCh].RefAngleCurrent = 0.0;
                _EncoderZeroFlag = false;
            }
            // -------------

            // Home Start ---
            if(_HomeStartFlag){
                if(_HomeSequence == 0){
                    pMain->_DEV_MC[selectedId].RBJoint_ResetEncoder(selectedCh+1);
                    pMain->_DEV_MC[selectedId].Joints[selectedCh].Reference = 0.0;
                    pMain->_DEV_MC[selectedId].MoveJoints[selectedCh].RefAngleCurrent = 0.0;
                    pMain->_DEV_MC[selectedId].RBJoint_EnableFETDriver(selectedCh+1, true);
                    _HomeSequence = 1;
                }else if(_HomeSequence == 1){
                    if(pMain->_DEV_MC[selectedId].Joints[selectedCh].CurrentStatus.b.HIP){
                        pMain->_DEV_MC[selectedId].RBJoint_FindHome(selectedCh+1);
                        _HomeStartFlag = false;
                    }
                }
            }
            // -------------



            // Send Reference (if ctrl enabled)
            if(pMain->_DEV_MC[selectedId].Joints[selectedCh].CurrentStatus.b.RUN){
                switch(pMain->_PAN_MC[pMain->selectedDevice]->OpMode){
                case OPMODE_DUTY:

                    break;
                case OPMODE_VEL:

                    break;
                case OPMODE_POS:

                    break;
                }
            }


            // Request Encoder & Status
            pMain->_DEV_MC[selectedId].RBBoard_RequestEncoder(0); // one-shot
            pMain->_DEV_MC[selectedId].RBBoard_RequestStatus();
        }else if(deviceType == 1){
            //=======================================
            //    Real-time thread for FT sensor
            //=======================================

        }

        cnt++;
    }
}


void MainWindow::on_BTN_SCAN_clicked()
{
    if(IS_CAN_OK == false)
        return;

    for(int i=0; i<_NO_OF_MC; i++){
        if(_DEV_MC[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS) == true){
            ui->LW_DEVICE->item(i)->setIcon(QIcon(AL_ICON_SUCCESS));
        }else{
            ui->LW_DEVICE->item(i)->setIcon(QIcon(AL_ICON_FAIL));
        }
    }

    for(int i=0; i<_NO_OF_FT; i++){
        if(_DEV_FT[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS) == true){
            ui->LW_DEVICE->item(i+_NO_OF_MC)->setIcon(QIcon(AL_ICON_SUCCESS));
        }else{
            ui->LW_DEVICE->item(i+_NO_OF_MC)->setIcon(QIcon(AL_ICON_FAIL));
        }
    }
}


int MainWindow::DBInitialize(){
    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
        return false;
    }

    _VERSION        = RBDataBase::_DB_GENERAL.VERSION;
    _NO_OF_AL       = RBDataBase::_DB_GENERAL.NO_OF_AL;
    _NO_OF_COMM_CH  = RBDataBase::_DB_GENERAL.NO_OF_COMM_CH;
    _NO_OF_MC       = RBDataBase::_DB_GENERAL.NO_OF_MC;
    _NO_OF_FT       = RBDataBase::_DB_GENERAL.NO_OF_FT;
    _NO_OF_IMU      = RBDataBase::_DB_GENERAL.NO_OF_IMU;
    _NO_OF_SP       = RBDataBase::_DB_GENERAL.NO_OF_SP;
    _NO_OF_OF       = RBDataBase::_DB_GENERAL.NO_OF_OF;

    FILE_LOG(logSUCCESS) << "Core load database = OK";

    std::cout << "----------------------------------" << std::endl;
    std::cout << "     VERSION       : " << _VERSION << std::endl;
    std::cout << "     NO_OF_AL      : " << _NO_OF_AL << std::endl;
    std::cout << "     NO_OF_COMM_CH : " << _NO_OF_COMM_CH << std::endl;
    std::cout << "     NO_OF_MC      : " << _NO_OF_MC << std::endl;
    std::cout << "     NO_OF_FT      : " << _NO_OF_FT << std::endl;
    std::cout << "     NO_OF_IMU     : " << _NO_OF_IMU << std::endl;
    std::cout << "     NO_OF_SP      : " << _NO_OF_SP << std::endl;
    std::cout << "     NO_OF_OF      : " << _NO_OF_OF << std::endl;
    std::cout << "----------------------------------" << std::endl;

    ui->LB_VERSION->setText(QString().sprintf("%d", _VERSION));
    ui->LB_AL->setText(QString().sprintf("%d", _NO_OF_AL));
    ui->LB_CAN->setText(QString().sprintf("%d", _NO_OF_COMM_CH));
    ui->LB_MC->setText(QString().sprintf("%d", _NO_OF_MC));
    ui->LB_FT->setText(QString().sprintf("%d", _NO_OF_FT));
    ui->LB_IMU->setText(QString().sprintf("%d", _NO_OF_IMU));

    for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBBoard_GetDBData(RBDataBase::_DB_MC[i]);
    for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBBoard_GetDBData(RBDataBase::_DB_FT[i]);
    return true;
}


int MainWindow::CANInitialize(){
    ui->LE_CAN1->setStyleSheet("background-color: yellow");
    ui->LE_CAN2->setStyleSheet("background-color: yellow");
    ui->LE_CAN3->setStyleSheet("background-color: yellow");
    ui->LE_CAN4->setStyleSheet("background-color: yellow");

    QLineEdit *tempLE[4];
    tempLE[0] = ui->LE_CAN1;
    tempLE[1] = ui->LE_CAN2;
    tempLE[2] = ui->LE_CAN3;
    tempLE[3] = ui->LE_CAN4;

    canHandler = new RBCAN(_NO_OF_COMM_CH);


    if(canHandler->IsWorking() == false){
        for(int i=0; i<_NO_OF_COMM_CH; i++){
            if(canHandler->canHandler[i] == NULL)   tempLE[i]->setStyleSheet("background-color: red");
            else                                    tempLE[i]->setStyleSheet("background-color: green");
        }

        IS_CAN_OK = false;
        return false;
    }else{
        for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBMC_AddCANMailBox();
        for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBFT_AddCANMailBox();

        for(int i=0; i<_NO_OF_COMM_CH; i++){
            if(canHandler->canHandler[i] == NULL)   tempLE[i]->setStyleSheet("background-color: red");
            else                                    tempLE[i]->setStyleSheet("background-color: green");
        }
        IS_CAN_OK = true;
        return true;
    }
}


int MainWindow::PanelInitialize(){
    _Frame_Panel = new QFrame(this);
    _Frame_Graph = new QFrame(this);

    indexMC = 0;
    for(int i=0; i<_NO_OF_MC; i++){
        _PAN_MC[i] = new Panel_MC(_Frame_Panel, &_DEV_MC[i]);
        _PAN_MC[i]->setWindowFlags(Qt::Widget);
        _PAN_MC[i]->move(0,0);
        _PAN_MC[i]->hide();
        ui->LW_DEVICE->addItem(QString().sprintf("%2d, ", _PAN_MC[i]->dev_info->BOARD_ID) + _DEV_MC[i].BOARD_NAME);
        ui->LW_DEVICE->item(i)->setIcon(QIcon(AL_ICON_FAIL));
    }

    indexFT = _NO_OF_MC;
    for(int i=0; i<_NO_OF_FT; i++){
        _PAN_FT[i] = new Panel_FT(_Frame_Panel, &_DEV_FT[i]);
        _PAN_FT[i]->setWindowFlags(Qt::Widget);
        _PAN_FT[i]->move(0,0);
        _PAN_FT[i]->hide();
        ui->LW_DEVICE->addItem(QString().sprintf("%2d, ", _PAN_FT[i]->dev_info->BOARD_ID) + _DEV_FT[i].BOARD_NAME);
        ui->LW_DEVICE->item(i+_NO_OF_MC)->setIcon(QIcon(AL_ICON_FAIL));
    }
    _Frame_Panel->setFixedSize(_PAN_MC[0]->size());
    _Frame_Panel->move(490, 10);



    _GraphPlot = new RBGraph(_Frame_Graph);
    _GraphPlot->setWindowFlags(Qt::Widget);
    _GraphPlot->move(0,0);
    _Frame_Graph->setFixedSize(_GraphPlot->size());
    _Frame_Graph->move(920, 10);


    return true;
}

int MainWindow::ThreadInitialize(){
    if(rt_task_create(&rtTaskCon, "RBDeviceManager", 0, 99, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(0, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Core real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, this) == 0){
            FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Core real-time thread start = FAIL";
            return false;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create core real-time thread";
        return false;
    }
}

void MainWindow::on_LW_DEVICE_doubleClicked(const QModelIndex &index)
{
    if(IS_CAN_OK == false)
        return;

    int row = index.row();
    for(int i=0; i<_NO_OF_MC; i++){
        _PAN_MC[i]->hide();
    }
    for(int i=0; i<_NO_OF_FT; i++){
        _PAN_FT[i]->hide();
    }

    if(row < indexFT){
        _PAN_MC[row]->show();
        _PAN_MC[row]->NewAccess();
    }else{
        _PAN_FT[row-indexFT]->show();
    }

    selectedDevice = row;
}
