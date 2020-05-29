#ifndef LAUNCHERDIALOG_H
#define LAUNCHERDIALOG_H

#include <signal.h>

#include <QDialog>
#include <QFileDialog>
#include <QSettings>
#include <QProcess>
#include <QMessageBox>
#include <QSystemTrayIcon>
#include <QCloseEvent>
#include <QMenu>
#include <QTimer>

class QAction;


namespace Ui {
class LauncherDialog;
}

class LauncherDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LauncherDialog(QWidget *parent = 0);
    ~LauncherDialog();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void on_BTN_CHANGE_DAEMON_clicked();
    void on_BTN_CHANGE_GUI_clicked();

    void on_RB_ROBOT_toggled(bool checked);

    void on_BTN_START_DAEMON_clicked();
    void on_BTN_STOP_DAEMON_clicked();

    void on_BTN_START_GUI_clicked();
    void on_BTN_STOP_GUI_clicked();

    void on_showNormal();

private:
    Ui::LauncherDialog *ui;

    qint64      Daemon_processID, GUI_processID;
    QString     settingFile;

    QAction         *restoreAction, *quitAction;
    QAction         *startDaemonAction, *stopDaemonAction;
    QAction         *startGUIAction, *stopGUIAction;

    QSystemTrayIcon *trayIcon;
    QMenu           *trayIconMenu;

    void    createActions();
    void    createTrayIcon();

};

#endif // LAUNCHERDIALOG_H
