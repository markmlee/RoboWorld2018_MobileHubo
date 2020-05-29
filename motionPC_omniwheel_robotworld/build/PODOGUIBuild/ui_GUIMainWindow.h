/********************************************************************************
** Form generated from reading UI file 'GUIMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUIMAINWINDOW_H
#define UI_GUIMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GUIMainWindow
{
public:
    QAction *actionLAN;
    QAction *actionPODOAL;
    QAction *actionJOINT;
    QAction *actionSENSOR;
    QAction *actionDUMMY;
    QAction *actionMODEL;
    QWidget *centralWidget;
    QTabWidget *MAIN_TAB;
    QGroupBox *groupBox_2;
    QLabel *LB_VOLTAGE;
    QFrame *line;
    QLabel *LB_CURRENT;
    QPushButton *BTN_REF_ENABLE;
    QPushButton *BTN_REF_DISABLE;
    QPushButton *BTN_CAN_DISABLE;
    QPushButton *BTN_CAN_ENABLE;
    QPushButton *BTN_SENSOR_ENABLE;
    QPushButton *BTN_SENSOR_DISABLE;
    QPushButton *BTN_ENCODER_DISABLE;
    QPushButton *BTN_ENCODER_ENABLE;
    QLineEdit *LE_ENC_STAT;
    QLineEdit *LE_SEN_STAT;
    QLineEdit *LE_CAN_STAT;
    QLineEdit *LE_REF_STAT;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GUIMainWindow)
    {
        if (GUIMainWindow->objectName().isEmpty())
            GUIMainWindow->setObjectName(QStringLiteral("GUIMainWindow"));
        GUIMainWindow->resize(880, 880);
        actionLAN = new QAction(GUIMainWindow);
        actionLAN->setObjectName(QStringLiteral("actionLAN"));
        actionLAN->setCheckable(true);
        QIcon icon;
        icon.addFile(QStringLiteral("../../share/GUI/icon/LAN_OFF_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon.addFile(QStringLiteral("../../share/GUI/icon/LAN_OFF_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionLAN->setIcon(icon);
        actionPODOAL = new QAction(GUIMainWindow);
        actionPODOAL->setObjectName(QStringLiteral("actionPODOAL"));
        actionPODOAL->setCheckable(true);
        QIcon icon1;
        icon1.addFile(QStringLiteral("../../share/GUI/icon/MODULE_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QStringLiteral("../../share/GUI/icon/MODULE_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionPODOAL->setIcon(icon1);
        actionJOINT = new QAction(GUIMainWindow);
        actionJOINT->setObjectName(QStringLiteral("actionJOINT"));
        actionJOINT->setCheckable(true);
        QIcon icon2;
        icon2.addFile(QStringLiteral("../../share/GUI/icon/JOINT_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QStringLiteral("../../share/GUI/icon/JOINT_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionJOINT->setIcon(icon2);
        actionSENSOR = new QAction(GUIMainWindow);
        actionSENSOR->setObjectName(QStringLiteral("actionSENSOR"));
        actionSENSOR->setCheckable(true);
        QIcon icon3;
        icon3.addFile(QStringLiteral("../../share/GUI/icon/SENSOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon3.addFile(QStringLiteral("../../share/GUI/icon/SENSOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionSENSOR->setIcon(icon3);
        actionDUMMY = new QAction(GUIMainWindow);
        actionDUMMY->setObjectName(QStringLiteral("actionDUMMY"));
        actionDUMMY->setEnabled(false);
        actionMODEL = new QAction(GUIMainWindow);
        actionMODEL->setObjectName(QStringLiteral("actionMODEL"));
        actionMODEL->setCheckable(true);
        QIcon icon4;
        icon4.addFile(QStringLiteral("../../share/GUI/icon/SIMULATOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon4.addFile(QStringLiteral("../../share/GUI/icon/SIMULATOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionMODEL->setIcon(icon4);
        centralWidget = new QWidget(GUIMainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MAIN_TAB = new QTabWidget(centralWidget);
        MAIN_TAB->setObjectName(QStringLiteral("MAIN_TAB"));
        MAIN_TAB->setGeometry(QRect(10, 90, 760, 760));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 10, 161, 51));
        groupBox_2->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
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
        LB_VOLTAGE = new QLabel(groupBox_2);
        LB_VOLTAGE->setObjectName(QStringLiteral("LB_VOLTAGE"));
        LB_VOLTAGE->setGeometry(QRect(10, 20, 61, 21));
        QFont font;
        font.setPointSize(13);
        LB_VOLTAGE->setFont(font);
        LB_VOLTAGE->setAlignment(Qt::AlignCenter);
        line = new QFrame(groupBox_2);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(70, 20, 20, 21));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        LB_CURRENT = new QLabel(groupBox_2);
        LB_CURRENT->setObjectName(QStringLiteral("LB_CURRENT"));
        LB_CURRENT->setGeometry(QRect(90, 20, 61, 21));
        LB_CURRENT->setFont(font);
        LB_CURRENT->setAlignment(Qt::AlignCenter);
        BTN_REF_ENABLE = new QPushButton(centralWidget);
        BTN_REF_ENABLE->setObjectName(QStringLiteral("BTN_REF_ENABLE"));
        BTN_REF_ENABLE->setGeometry(QRect(650, 10, 61, 41));
        BTN_REF_DISABLE = new QPushButton(centralWidget);
        BTN_REF_DISABLE->setObjectName(QStringLiteral("BTN_REF_DISABLE"));
        BTN_REF_DISABLE->setGeometry(QRect(710, 10, 61, 41));
        BTN_CAN_DISABLE = new QPushButton(centralWidget);
        BTN_CAN_DISABLE->setObjectName(QStringLiteral("BTN_CAN_DISABLE"));
        BTN_CAN_DISABLE->setGeometry(QRect(560, 10, 61, 41));
        BTN_CAN_ENABLE = new QPushButton(centralWidget);
        BTN_CAN_ENABLE->setObjectName(QStringLiteral("BTN_CAN_ENABLE"));
        BTN_CAN_ENABLE->setGeometry(QRect(500, 10, 61, 41));
        BTN_SENSOR_ENABLE = new QPushButton(centralWidget);
        BTN_SENSOR_ENABLE->setObjectName(QStringLiteral("BTN_SENSOR_ENABLE"));
        BTN_SENSOR_ENABLE->setGeometry(QRect(350, 10, 61, 41));
        BTN_SENSOR_DISABLE = new QPushButton(centralWidget);
        BTN_SENSOR_DISABLE->setObjectName(QStringLiteral("BTN_SENSOR_DISABLE"));
        BTN_SENSOR_DISABLE->setGeometry(QRect(410, 10, 61, 41));
        BTN_ENCODER_DISABLE = new QPushButton(centralWidget);
        BTN_ENCODER_DISABLE->setObjectName(QStringLiteral("BTN_ENCODER_DISABLE"));
        BTN_ENCODER_DISABLE->setGeometry(QRect(260, 10, 61, 41));
        BTN_ENCODER_ENABLE = new QPushButton(centralWidget);
        BTN_ENCODER_ENABLE->setObjectName(QStringLiteral("BTN_ENCODER_ENABLE"));
        BTN_ENCODER_ENABLE->setGeometry(QRect(200, 10, 61, 41));
        LE_ENC_STAT = new QLineEdit(centralWidget);
        LE_ENC_STAT->setObjectName(QStringLiteral("LE_ENC_STAT"));
        LE_ENC_STAT->setEnabled(false);
        LE_ENC_STAT->setGeometry(QRect(200, 50, 121, 21));
        LE_ENC_STAT->setReadOnly(true);
        LE_SEN_STAT = new QLineEdit(centralWidget);
        LE_SEN_STAT->setObjectName(QStringLiteral("LE_SEN_STAT"));
        LE_SEN_STAT->setEnabled(false);
        LE_SEN_STAT->setGeometry(QRect(350, 50, 121, 21));
        LE_SEN_STAT->setReadOnly(true);
        LE_CAN_STAT = new QLineEdit(centralWidget);
        LE_CAN_STAT->setObjectName(QStringLiteral("LE_CAN_STAT"));
        LE_CAN_STAT->setEnabled(false);
        LE_CAN_STAT->setGeometry(QRect(500, 50, 121, 21));
        LE_CAN_STAT->setReadOnly(true);
        LE_REF_STAT = new QLineEdit(centralWidget);
        LE_REF_STAT->setObjectName(QStringLiteral("LE_REF_STAT"));
        LE_REF_STAT->setEnabled(false);
        LE_REF_STAT->setGeometry(QRect(650, 50, 121, 21));
        LE_REF_STAT->setReadOnly(true);
        GUIMainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(GUIMainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 880, 19));
        GUIMainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(GUIMainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mainToolBar->sizePolicy().hasHeightForWidth());
        mainToolBar->setSizePolicy(sizePolicy);
        mainToolBar->setStyleSheet(QStringLiteral("background-color: rgb(202, 203, 210);"));
        mainToolBar->setMovable(false);
        mainToolBar->setIconSize(QSize(80, 80));
        GUIMainWindow->addToolBar(Qt::LeftToolBarArea, mainToolBar);
        statusBar = new QStatusBar(GUIMainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        GUIMainWindow->setStatusBar(statusBar);

        mainToolBar->addAction(actionLAN);
        mainToolBar->addAction(actionPODOAL);
        mainToolBar->addAction(actionDUMMY);
        mainToolBar->addAction(actionJOINT);
        mainToolBar->addAction(actionSENSOR);
        mainToolBar->addAction(actionMODEL);
        mainToolBar->addSeparator();

        retranslateUi(GUIMainWindow);

        MAIN_TAB->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(GUIMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *GUIMainWindow)
    {
        GUIMainWindow->setWindowTitle(QApplication::translate("GUIMainWindow", "PODO GUI -- Rainbow Robotics", 0));
        actionLAN->setText(QApplication::translate("GUIMainWindow", "LAN Connection", 0));
#ifndef QT_NO_TOOLTIP
        actionLAN->setToolTip(QApplication::translate("GUIMainWindow", "LAN Connection", 0));
#endif // QT_NO_TOOLTIP
        actionPODOAL->setText(QApplication::translate("GUIMainWindow", "PODOAL Control", 0));
#ifndef QT_NO_TOOLTIP
        actionPODOAL->setToolTip(QApplication::translate("GUIMainWindow", "PODOAL Control", 0));
#endif // QT_NO_TOOLTIP
        actionJOINT->setText(QApplication::translate("GUIMainWindow", "JOINT Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionJOINT->setToolTip(QApplication::translate("GUIMainWindow", "JOINT Pannel", 0));
#endif // QT_NO_TOOLTIP
        actionSENSOR->setText(QApplication::translate("GUIMainWindow", "SENSOR Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionSENSOR->setToolTip(QApplication::translate("GUIMainWindow", "SENSOR Pannel", 0));
#endif // QT_NO_TOOLTIP
        actionDUMMY->setText(QString());
        actionMODEL->setText(QApplication::translate("GUIMainWindow", "MODEL Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionMODEL->setToolTip(QApplication::translate("GUIMainWindow", "MODEL Pannel", 0));
#endif // QT_NO_TOOLTIP
        groupBox_2->setTitle(QApplication::translate("GUIMainWindow", "Power", 0));
        LB_VOLTAGE->setText(QApplication::translate("GUIMainWindow", "NC", 0));
        LB_CURRENT->setText(QApplication::translate("GUIMainWindow", "NC", 0));
        BTN_REF_ENABLE->setText(QApplication::translate("GUIMainWindow", "Ref\n"
"Enable", 0));
        BTN_REF_DISABLE->setText(QApplication::translate("GUIMainWindow", "Ref\n"
"Disable", 0));
        BTN_CAN_DISABLE->setText(QApplication::translate("GUIMainWindow", "CAN\n"
"Disable", 0));
        BTN_CAN_ENABLE->setText(QApplication::translate("GUIMainWindow", "CAN\n"
"Enable", 0));
        BTN_SENSOR_ENABLE->setText(QApplication::translate("GUIMainWindow", "Sensor\n"
"Enable", 0));
        BTN_SENSOR_DISABLE->setText(QApplication::translate("GUIMainWindow", "Sensor\n"
"Disable", 0));
        BTN_ENCODER_DISABLE->setText(QApplication::translate("GUIMainWindow", "Enc\n"
"Disable", 0));
        BTN_ENCODER_ENABLE->setText(QApplication::translate("GUIMainWindow", "Enc.\n"
"Enable", 0));
    } // retranslateUi

};

namespace Ui {
    class GUIMainWindow: public Ui_GUIMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUIMAINWINDOW_H
