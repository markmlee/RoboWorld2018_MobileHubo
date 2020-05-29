/********************************************************************************
** Form generated from reading UI file 'LauncherDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LAUNCHERDIALOG_H
#define UI_LAUNCHERDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LauncherDialog
{
public:
    QPushButton *BTN_CHANGE_DAEMON;
    QLineEdit *LE_DAEMON_PATH;
    QPushButton *BTN_START_DAEMON;
    QGroupBox *GB_CONTROL;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QRadioButton *RB_ROS_OK;
    QRadioButton *RB_ROS_NO;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QRadioButton *RB_FOG_OK;
    QRadioButton *RB_FOG_NO;
    QFrame *line;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QRadioButton *RB_ROBOT;
    QRadioButton *RB_GAZEBO;
    QFrame *line_4;
    QSpinBox *SB_HORIZONTAL;
    QSpinBox *SB_VERTICAL;
    QLabel *label;
    QLabel *label_2;
    QFrame *line_2;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QCheckBox *CB_EXF_R;
    QCheckBox *CB_EXF_L;
    QPushButton *BTN_STOP_DAEMON;
    QLineEdit *LE_DAEMON_PID;
    QPushButton *BTN_CHANGE_GUI;
    QLineEdit *LE_GUI_PID;
    QPushButton *BTN_START_GUI;
    QLineEdit *LE_GUI_PATH;
    QPushButton *BTN_STOP_GUI;

    void setupUi(QDialog *LauncherDialog)
    {
        if (LauncherDialog->objectName().isEmpty())
            LauncherDialog->setObjectName(QStringLiteral("LauncherDialog"));
        LauncherDialog->resize(770, 320);
        BTN_CHANGE_DAEMON = new QPushButton(LauncherDialog);
        BTN_CHANGE_DAEMON->setObjectName(QStringLiteral("BTN_CHANGE_DAEMON"));
        BTN_CHANGE_DAEMON->setGeometry(QRect(320, 20, 101, 51));
        LE_DAEMON_PATH = new QLineEdit(LauncherDialog);
        LE_DAEMON_PATH->setObjectName(QStringLiteral("LE_DAEMON_PATH"));
        LE_DAEMON_PATH->setGeometry(QRect(20, 20, 291, 51));
        BTN_START_DAEMON = new QPushButton(LauncherDialog);
        BTN_START_DAEMON->setObjectName(QStringLiteral("BTN_START_DAEMON"));
        BTN_START_DAEMON->setGeometry(QRect(20, 90, 141, 41));
        QFont font;
        font.setPointSize(13);
        BTN_START_DAEMON->setFont(font);
        GB_CONTROL = new QGroupBox(LauncherDialog);
        GB_CONTROL->setObjectName(QStringLiteral("GB_CONTROL"));
        GB_CONTROL->setGeometry(QRect(440, 20, 311, 281));
        QFont font1;
        font1.setPointSize(11);
        GB_CONTROL->setFont(font1);
        GB_CONTROL->setAutoFillBackground(false);
        GB_CONTROL->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 2px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        verticalLayoutWidget = new QWidget(GB_CONTROL);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(158, 130, 141, 71));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        RB_ROS_OK = new QRadioButton(verticalLayoutWidget);
        RB_ROS_OK->setObjectName(QStringLiteral("RB_ROS_OK"));

        verticalLayout->addWidget(RB_ROS_OK);

        RB_ROS_NO = new QRadioButton(verticalLayoutWidget);
        RB_ROS_NO->setObjectName(QStringLiteral("RB_ROS_NO"));

        verticalLayout->addWidget(RB_ROS_NO);

        verticalLayoutWidget_3 = new QWidget(GB_CONTROL);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(10, 130, 141, 71));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        RB_FOG_OK = new QRadioButton(verticalLayoutWidget_3);
        RB_FOG_OK->setObjectName(QStringLiteral("RB_FOG_OK"));

        verticalLayout_3->addWidget(RB_FOG_OK);

        RB_FOG_NO = new QRadioButton(verticalLayoutWidget_3);
        RB_FOG_NO->setObjectName(QStringLiteral("RB_FOG_NO"));

        verticalLayout_3->addWidget(RB_FOG_NO);

        line = new QFrame(GB_CONTROL);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(20, 60, 271, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        horizontalLayoutWidget = new QWidget(GB_CONTROL);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 20, 291, 41));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        RB_ROBOT = new QRadioButton(horizontalLayoutWidget);
        RB_ROBOT->setObjectName(QStringLiteral("RB_ROBOT"));

        horizontalLayout->addWidget(RB_ROBOT);

        RB_GAZEBO = new QRadioButton(horizontalLayoutWidget);
        RB_GAZEBO->setObjectName(QStringLiteral("RB_GAZEBO"));

        horizontalLayout->addWidget(RB_GAZEBO);

        line_4 = new QFrame(GB_CONTROL);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(20, 200, 271, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        SB_HORIZONTAL = new QSpinBox(GB_CONTROL);
        SB_HORIZONTAL->setObjectName(QStringLiteral("SB_HORIZONTAL"));
        SB_HORIZONTAL->setGeometry(QRect(20, 240, 111, 27));
        SB_HORIZONTAL->setMinimum(20);
        SB_HORIZONTAL->setMaximum(200);
        SB_HORIZONTAL->setSingleStep(20);
        SB_HORIZONTAL->setValue(60);
        SB_VERTICAL = new QSpinBox(GB_CONTROL);
        SB_VERTICAL->setObjectName(QStringLiteral("SB_VERTICAL"));
        SB_VERTICAL->setGeometry(QRect(160, 240, 111, 27));
        SB_VERTICAL->setMinimum(20);
        SB_VERTICAL->setMaximum(200);
        SB_VERTICAL->setSingleStep(20);
        SB_VERTICAL->setValue(40);
        label = new QLabel(GB_CONTROL);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 220, 131, 17));
        label_2 = new QLabel(GB_CONTROL);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(160, 220, 131, 17));
        line_2 = new QFrame(GB_CONTROL);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(20, 110, 271, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        gridLayoutWidget = new QWidget(GB_CONTROL);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 80, 291, 31));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        CB_EXF_R = new QCheckBox(gridLayoutWidget);
        CB_EXF_R->setObjectName(QStringLiteral("CB_EXF_R"));

        gridLayout->addWidget(CB_EXF_R, 0, 0, 1, 1);

        CB_EXF_L = new QCheckBox(gridLayoutWidget);
        CB_EXF_L->setObjectName(QStringLiteral("CB_EXF_L"));

        gridLayout->addWidget(CB_EXF_L, 0, 1, 1, 1);

        BTN_STOP_DAEMON = new QPushButton(LauncherDialog);
        BTN_STOP_DAEMON->setObjectName(QStringLiteral("BTN_STOP_DAEMON"));
        BTN_STOP_DAEMON->setGeometry(QRect(170, 90, 141, 41));
        BTN_STOP_DAEMON->setFont(font);
        LE_DAEMON_PID = new QLineEdit(LauncherDialog);
        LE_DAEMON_PID->setObjectName(QStringLiteral("LE_DAEMON_PID"));
        LE_DAEMON_PID->setEnabled(false);
        LE_DAEMON_PID->setGeometry(QRect(320, 90, 101, 41));
        LE_DAEMON_PID->setReadOnly(true);
        BTN_CHANGE_GUI = new QPushButton(LauncherDialog);
        BTN_CHANGE_GUI->setObjectName(QStringLiteral("BTN_CHANGE_GUI"));
        BTN_CHANGE_GUI->setGeometry(QRect(320, 190, 101, 51));
        LE_GUI_PID = new QLineEdit(LauncherDialog);
        LE_GUI_PID->setObjectName(QStringLiteral("LE_GUI_PID"));
        LE_GUI_PID->setEnabled(false);
        LE_GUI_PID->setGeometry(QRect(320, 260, 101, 41));
        LE_GUI_PID->setReadOnly(true);
        BTN_START_GUI = new QPushButton(LauncherDialog);
        BTN_START_GUI->setObjectName(QStringLiteral("BTN_START_GUI"));
        BTN_START_GUI->setGeometry(QRect(20, 260, 141, 41));
        BTN_START_GUI->setFont(font);
        LE_GUI_PATH = new QLineEdit(LauncherDialog);
        LE_GUI_PATH->setObjectName(QStringLiteral("LE_GUI_PATH"));
        LE_GUI_PATH->setGeometry(QRect(20, 190, 291, 51));
        BTN_STOP_GUI = new QPushButton(LauncherDialog);
        BTN_STOP_GUI->setObjectName(QStringLiteral("BTN_STOP_GUI"));
        BTN_STOP_GUI->setGeometry(QRect(170, 260, 141, 41));
        BTN_STOP_GUI->setFont(font);

        retranslateUi(LauncherDialog);

        QMetaObject::connectSlotsByName(LauncherDialog);
    } // setupUi

    void retranslateUi(QDialog *LauncherDialog)
    {
        LauncherDialog->setWindowTitle(QApplication::translate("LauncherDialog", "Daemon Launcher", 0));
        BTN_CHANGE_DAEMON->setText(QApplication::translate("LauncherDialog", "Change\n"
"Daemon", 0));
        BTN_START_DAEMON->setText(QApplication::translate("LauncherDialog", "Start Daemon", 0));
        GB_CONTROL->setTitle(QApplication::translate("LauncherDialog", "Daemon Setting", 0));
        RB_ROS_OK->setText(QApplication::translate("LauncherDialog", "Use ROS", 0));
        RB_ROS_NO->setText(QApplication::translate("LauncherDialog", "No ROS", 0));
        RB_FOG_OK->setText(QApplication::translate("LauncherDialog", "Use FOG", 0));
        RB_FOG_NO->setText(QApplication::translate("LauncherDialog", "No FOG", 0));
        RB_ROBOT->setText(QApplication::translate("LauncherDialog", "Robot", 0));
        RB_GAZEBO->setText(QApplication::translate("LauncherDialog", "Gazebo", 0));
        label->setText(QApplication::translate("LauncherDialog", "Horizontal", 0));
        label_2->setText(QApplication::translate("LauncherDialog", "Vertical", 0));
        CB_EXF_R->setText(QApplication::translate("LauncherDialog", "Extra Finger (R)", 0));
        CB_EXF_L->setText(QApplication::translate("LauncherDialog", "Extra Finger (L)", 0));
        BTN_STOP_DAEMON->setText(QApplication::translate("LauncherDialog", "Stop Daemon", 0));
        BTN_CHANGE_GUI->setText(QApplication::translate("LauncherDialog", "Change\n"
"GUI", 0));
        BTN_START_GUI->setText(QApplication::translate("LauncherDialog", "Start GUI", 0));
        BTN_STOP_GUI->setText(QApplication::translate("LauncherDialog", "Stop GUI", 0));
    } // retranslateUi

};

namespace Ui {
    class LauncherDialog: public Ui_LauncherDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LAUNCHERDIALOG_H
