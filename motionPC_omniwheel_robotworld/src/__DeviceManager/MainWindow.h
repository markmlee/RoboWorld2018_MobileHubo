#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFrame>


#include "RBGraph.h"

#include "RBMotorController.h"
#include "RBFTSensor.h"

#include "Panel_MC.h"
#include "Panel_FT.h"


namespace Ui {
class MainWindow;
}


#define AL_ICON_FAIL        "../share/GUI/icon/disable.png"
#define AL_ICON_SUCCESS     "../share/GUI/icon/enable.png"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();




private slots:
    void Closing();
    void on_BTN_SCAN_clicked();
    void on_LW_DEVICE_doubleClicked(const QModelIndex &index);

private:
    Ui::MainWindow *ui;

    RT_TASK rtTaskCon;
    int     IS_WORKING;
    int     IS_CAN_OK;

    int     _VERSION;
    int     _NO_OF_AL;
    int     _NO_OF_COMM_CH;
    int     _NO_OF_MC;
    int     _NO_OF_FT;
    int     _NO_OF_IMU;
    int     _NO_OF_SP;
    int     _NO_OF_OF;

    QFrame*             _Frame_Panel;
    QFrame*             _Frame_Graph;

    RBMotorController   _DEV_MC[MAX_MC];
    Panel_MC*           _PAN_MC[MAX_MC];

    RBFTSensor          _DEV_FT[MAX_FT];
    Panel_FT*           _PAN_FT[MAX_FT];

    RBGraph*            _GraphPlot;

    int     indexMC;
    int     indexFT;


    // ===========================

    int     selectedDevice;


    // ===========================


    // real-time thread
    static void    RBCore_RTThreadCon(void *);

    int     DBInitialize();
    int     CANInitialize();
    int     PanelInitialize();
    int     ThreadInitialize();
};

#endif // MAINWINDOW_H
