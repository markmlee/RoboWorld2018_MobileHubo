#include "LauncherDialog.h"
#include "ui_LauncherDialog.h"

#include <iostream>

LauncherDialog::LauncherDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LauncherDialog)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/icon/icon.png"));
    setWindowFlags(Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);

    Daemon_processID = GUI_processID = 0;

    ui->RB_GAZEBO->setChecked(true);
    ui->RB_ROBOT->setChecked(true);
    ui->LE_DAEMON_PID->setText("0");
    ui->LE_GUI_PID->setText("0");


    settingFile = QApplication::applicationDirPath().left(1) + "/demosettings.ini";
    settingFile = "configs/LauncherConfig.ini";
    QSettings settings(settingFile, QSettings::NativeFormat);
    ui->LE_DAEMON_PATH->setText(settings.value("daemon", "").toString());
    ui->LE_GUI_PATH->setText(settings.value("gui", "").toString());
    if(settings.value("gazebo", "").toBool()){
        ui->RB_GAZEBO->setChecked(true);
    }else{
        ui->RB_ROBOT->setChecked(true);
    }
    ui->CB_EXF_R->setChecked(settings.value("exfr", "").toBool());
    ui->CB_EXF_L->setChecked(settings.value("exfl", "").toBool());
    if(settings.value("fog", "").toBool()){
        ui->RB_FOG_OK->setChecked(true);
    }else{
        ui->RB_FOG_NO->setChecked(true);
    }
    if(settings.value("ros", "").toBool()){
        ui->RB_ROS_OK->setChecked(true);
    }else{
        ui->RB_ROS_NO->setChecked(true);
    }
    ui->SB_HORIZONTAL->setValue(settings.value("geo_hori","").toInt());
    ui->SB_VERTICAL->setValue(settings.value("geo_vert","").toInt());

    createActions();
    createTrayIcon();
    trayIcon->show();

//    if(settings.value("show", "").toBool()){
//        this->show();
//    }else{
//        QTimer::singleShot(50, this, SLOT(close()));
//    }
    this->show();
}

LauncherDialog::~LauncherDialog()
{
    delete ui;
}


void LauncherDialog::closeEvent(QCloseEvent *event){
//    if(trayIcon->isVisible()){
//        hide();
//        event->ignore();
//        QSettings settings(settingFile, QSettings::NativeFormat);
//        settings.setValue("show", false);
//    }
}

void LauncherDialog::createActions(){
    restoreAction = new QAction(tr("&restore"), this);
    quitAction = new QAction(tr("&exit"), this);
    connect(restoreAction, SIGNAL(triggered(bool)), this, SLOT(on_showNormal()));
    connect(quitAction, SIGNAL(triggered(bool)), qApp,SLOT(quit()));

    startDaemonAction = new QAction(tr("&start Daemon"), this);
    stopDaemonAction = new QAction(tr("&stop Daemon"), this);
    startGUIAction = new QAction(tr("&start GUI"), this);
    stopGUIAction = new QAction(tr("&stop GUI"), this);
    connect(startDaemonAction, SIGNAL(triggered(bool)), this, SLOT(on_BTN_START_DAEMON_clicked()));
    connect(stopDaemonAction, SIGNAL(triggered(bool)), this, SLOT(on_BTN_STOP_DAEMON_clicked()));
    connect(startGUIAction, SIGNAL(triggered(bool)), this, SLOT(on_BTN_START_GUI_clicked()));
    connect(stopGUIAction, SIGNAL(triggered(bool)), this, SLOT(on_BTN_STOP_GUI_clicked()));
}

void LauncherDialog::createTrayIcon(){
    trayIconMenu = new QMenu(this);
    trayIconMenu->addAction(restoreAction);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(startDaemonAction);
    trayIconMenu->addAction(stopDaemonAction);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(startGUIAction);
    trayIconMenu->addAction(stopGUIAction);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(quitAction);
    trayIcon = new QSystemTrayIcon(this);
    QIcon icon(":/icon/icon.png");
    trayIcon->setIcon(icon);
    trayIcon->setContextMenu(trayIconMenu);
}

void LauncherDialog::on_BTN_CHANGE_DAEMON_clicked(){
    QString fileName = QFileDialog::getOpenFileName(this, tr("Change Daemon"), "", tr("AllFiles (*)"));
    if(fileName.isEmpty())
        return;
    else{
        ui->LE_DAEMON_PATH->setText(fileName);
    }
}

void LauncherDialog::on_BTN_CHANGE_GUI_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Change GUI"), "", tr("AllFiles (*)"));
    if(fileName.isEmpty())
        return;
    else{
        ui->LE_GUI_PATH->setText(fileName);
    }
}

void LauncherDialog::on_showNormal(){
    this->showNormal();
    QSettings settings(settingFile, QSettings::NativeFormat);
    settings.setValue("show", true);
}

void LauncherDialog::on_BTN_START_DAEMON_clicked(){
    if(true){
        if(Daemon_processID != 0){
            QMessageBox::warning(this, "double executing Daemon", "There is an another Daemon working now..");
            return;
        }
        QStringList arguments;
        QFileInfo fileInfo(ui->LE_DAEMON_PATH->text());

        arguments << "-geometry";
        arguments << ui->SB_HORIZONTAL->text() + "x" + ui->SB_VERTICAL->text();
        arguments << "-e";
        arguments << "./"+fileInfo.fileName();

        int exf_setting = ui->CB_EXF_R->isChecked() | (ui->CB_EXF_L->isChecked()<<1);
        arguments << "-e";
        arguments << QString().sprintf("%d", exf_setting);

        if(ui->RB_GAZEBO->isChecked()){
            arguments << "-g";
            arguments << "true";
            arguments << "-f";
            arguments << "false";
        }else{
            arguments << "-g";
            arguments << "false";
            arguments << "-f";
            if(ui->RB_FOG_OK->isChecked()){
                arguments << "true";
            }else{
                arguments << "false";
            }
        }        
        if(ui->RB_ROS_OK->isChecked()){
            arguments << "-r";
            arguments << "true";
        }else{
            arguments << "-r";
            arguments << "false";
        }

        if(!QProcess::startDetached("xterm", arguments, fileInfo.path(), &Daemon_processID)){
            QMessageBox::warning(this, "Process Creation Error", "Process Creation Error");
            return;
        }

        ui->LE_DAEMON_PID->setText(QString().sprintf("%d", Daemon_processID));
        QSettings settings(settingFile, QSettings::NativeFormat);
        settings.setValue("daemon", ui->LE_DAEMON_PATH->text());
        settings.setValue("gazebo", ui->RB_GAZEBO->isChecked());
        settings.setValue("fog", ui->RB_FOG_OK->isChecked());
        settings.setValue("ros", ui->RB_ROS_OK->isChecked());
        settings.setValue("geo_hori", ui->SB_HORIZONTAL->text());
        settings.setValue("geo_vert", ui->SB_VERTICAL->text());
        settings.setValue("exfr", ui->CB_EXF_R->isChecked());
        settings.setValue("exfl", ui->CB_EXF_L->isChecked());

        ui->BTN_START_DAEMON->setEnabled(false);
        startDaemonAction->setEnabled(false);
    }else{
        //QProcess::execute(ui->LE_DAEMON_PATH->text());
    }
}

void LauncherDialog::on_RB_ROBOT_toggled(bool checked){
    if(checked){
        ui->RB_FOG_NO->setEnabled(true);
        ui->RB_FOG_OK->setEnabled(true);
    }else{
        ui->RB_FOG_NO->setEnabled(false);
        ui->RB_FOG_OK->setEnabled(false);
    }
}

void LauncherDialog::on_BTN_STOP_DAEMON_clicked(){
    if(Daemon_processID == 0)
        return;
    kill(Daemon_processID, SIGKILL);
    Daemon_processID = 0;
    ui->LE_DAEMON_PID->setText(QString().sprintf("%d", Daemon_processID));

    ui->BTN_START_DAEMON->setEnabled(true);
    startDaemonAction->setEnabled(true);
}


void LauncherDialog::on_BTN_START_GUI_clicked()
{
    if(GUI_processID != 0){
        QMessageBox::warning(this, "double executing GUI", "There is an another GUI working now..");
        return;
    }

    QStringList args;
    if(!QProcess::startDetached(ui->LE_GUI_PATH->text(), args, NULL, &GUI_processID)){
        QMessageBox::warning(this, "Process Creation Error", "Process Creation Error");
        return;
    }

    ui->LE_GUI_PID->setText(QString().sprintf("%d", GUI_processID));
    QSettings settings(settingFile, QSettings::NativeFormat);
    settings.setValue("gui", ui->LE_GUI_PATH->text());

    ui->BTN_START_GUI->setEnabled(false);
    startGUIAction->setEnabled(false);
}

void LauncherDialog::on_BTN_STOP_GUI_clicked()
{
    if(GUI_processID == 0)
        return;
    kill(GUI_processID, SIGKILL);
    GUI_processID = 0;
    ui->LE_GUI_PID->setText(QString().sprintf("%d", GUI_processID));

    ui->BTN_START_GUI->setEnabled(true);
    startGUIAction->setEnabled(true);
}
