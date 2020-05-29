/********************************************************************************
** Form generated from reading UI file 'RobotWorldDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOTWORLDDIALOG_H
#define UI_ROBOTWORLDDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RobotWorldDialog
{
public:
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *PB_WALK_READY;
    QGridLayout *gridLayout;
    QLineEdit *LE_OMNI_ROT;
    QLabel *label_4;
    QLabel *label_2;
    QLineEdit *LE_OMNI_TIME;
    QLabel *label;
    QLineEdit *LE_OMNI_X;
    QLineEdit *LE_OMNI_Y;
    QLabel *label_3;
    QVBoxLayout *verticalLayout;
    QPushButton *PB_GO_TO_DES_NEW;
    QPushButton *PB_STOP;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *PB_SAVE;
    QPushButton *PB_SAVE_START;
    QFrame *frame_2;
    QWidget *layoutWidget_3;
    QHBoxLayout *horizontalLayout_7;
    QGridLayout *gridLayout_4;
    QLabel *label_12;
    QLineEdit *LE_HANDPOS_X;
    QLabel *label_13;
    QLineEdit *LE_HANDPOS_Y;
    QLabel *label_14;
    QLineEdit *LE_HANDPOS_Z;
    QGridLayout *gridLayout_5;
    QLabel *label_15;
    QLineEdit *LE_HANDORI_x;
    QLineEdit *LE_HANDORI_w;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *LE_HANDORI_z;
    QLineEdit *LE_HANDORI_y;
    QLabel *label_18;
    QLineEdit *LE_HAND_ELB;
    QLabel *label_24;
    QVBoxLayout *verticalLayout_7;
    QPushButton *PB_RHAND_GO;
    QPushButton *PB_LHAND_GO;
    QLabel *label_19;
    QWidget *layoutWidget_5;
    QGridLayout *gridLayout_6;
    QLineEdit *LE_VPOS_X;
    QLabel *label_22;
    QLabel *label_21;
    QLineEdit *LE_VPOS_Y;
    QLineEdit *LE_VPOS_Z;
    QLabel *label_23;
    QLineEdit *LE_HAND_ELB_2;
    QLabel *label_20;
    QPushButton *PB_GRASP;
    QPushButton *PB_ESTOP;
    QPushButton *PB_PUT;
    QWidget *layoutWidget_4;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_8;
    QRadioButton *RB_HANDboth;
    QRadioButton *RB_HANDr;
    QRadioButton *RB_HANDl;
    QPushButton *PB_HAND_GRASP;
    QPushButton *PB_HAND_STOP;
    QPushButton *PB_HAND_OPEN;
    QPushButton *PB_WHEEL_MOVE_FRONT;
    QPushButton *PB_WHEEL_MOVE_BACK;
    QRadioButton *RB_WST_180;

    void setupUi(QDialog *RobotWorldDialog)
    {
        if (RobotWorldDialog->objectName().isEmpty())
            RobotWorldDialog->setObjectName(QStringLiteral("RobotWorldDialog"));
        RobotWorldDialog->resize(782, 714);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(RobotWorldDialog->sizePolicy().hasHeightForWidth());
        RobotWorldDialog->setSizePolicy(sizePolicy);
        layoutWidget = new QWidget(RobotWorldDialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 20, 451, 161));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        PB_WALK_READY = new QPushButton(layoutWidget);
        PB_WALK_READY->setObjectName(QStringLiteral("PB_WALK_READY"));
        sizePolicy.setHeightForWidth(PB_WALK_READY->sizePolicy().hasHeightForWidth());
        PB_WALK_READY->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(PB_WALK_READY);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        LE_OMNI_ROT = new QLineEdit(layoutWidget);
        LE_OMNI_ROT->setObjectName(QStringLiteral("LE_OMNI_ROT"));

        gridLayout->addWidget(LE_OMNI_ROT, 2, 2, 1, 1);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 3, 1, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 1, 1, 1, 1);

        LE_OMNI_TIME = new QLineEdit(layoutWidget);
        LE_OMNI_TIME->setObjectName(QStringLiteral("LE_OMNI_TIME"));

        gridLayout->addWidget(LE_OMNI_TIME, 3, 2, 1, 1);

        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 1, 1, 1);

        LE_OMNI_X = new QLineEdit(layoutWidget);
        LE_OMNI_X->setObjectName(QStringLiteral("LE_OMNI_X"));

        gridLayout->addWidget(LE_OMNI_X, 0, 2, 1, 1);

        LE_OMNI_Y = new QLineEdit(layoutWidget);
        LE_OMNI_Y->setObjectName(QStringLiteral("LE_OMNI_Y"));

        gridLayout->addWidget(LE_OMNI_Y, 1, 2, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 1, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        PB_GO_TO_DES_NEW = new QPushButton(layoutWidget);
        PB_GO_TO_DES_NEW->setObjectName(QStringLiteral("PB_GO_TO_DES_NEW"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(PB_GO_TO_DES_NEW->sizePolicy().hasHeightForWidth());
        PB_GO_TO_DES_NEW->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(PB_GO_TO_DES_NEW);

        PB_STOP = new QPushButton(layoutWidget);
        PB_STOP->setObjectName(QStringLiteral("PB_STOP"));
        sizePolicy1.setHeightForWidth(PB_STOP->sizePolicy().hasHeightForWidth());
        PB_STOP->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(PB_STOP);


        horizontalLayout->addLayout(verticalLayout);

        layoutWidget1 = new QWidget(RobotWorldDialog);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(620, 30, 131, 91));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        PB_SAVE = new QPushButton(layoutWidget1);
        PB_SAVE->setObjectName(QStringLiteral("PB_SAVE"));

        verticalLayout_2->addWidget(PB_SAVE);

        PB_SAVE_START = new QPushButton(layoutWidget1);
        PB_SAVE_START->setObjectName(QStringLiteral("PB_SAVE_START"));

        verticalLayout_2->addWidget(PB_SAVE_START);

        frame_2 = new QFrame(RobotWorldDialog);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(410, 500, 361, 211));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        layoutWidget_3 = new QWidget(frame_2);
        layoutWidget_3->setObjectName(QStringLiteral("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(10, 60, 331, 138));
        horizontalLayout_7 = new QHBoxLayout(layoutWidget_3);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_12 = new QLabel(layoutWidget_3);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_4->addWidget(label_12, 0, 0, 1, 1);

        LE_HANDPOS_X = new QLineEdit(layoutWidget_3);
        LE_HANDPOS_X->setObjectName(QStringLiteral("LE_HANDPOS_X"));

        gridLayout_4->addWidget(LE_HANDPOS_X, 0, 1, 1, 1);

        label_13 = new QLabel(layoutWidget_3);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_4->addWidget(label_13, 1, 0, 1, 1);

        LE_HANDPOS_Y = new QLineEdit(layoutWidget_3);
        LE_HANDPOS_Y->setObjectName(QStringLiteral("LE_HANDPOS_Y"));

        gridLayout_4->addWidget(LE_HANDPOS_Y, 1, 1, 1, 1);

        label_14 = new QLabel(layoutWidget_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_4->addWidget(label_14, 2, 0, 1, 1);

        LE_HANDPOS_Z = new QLineEdit(layoutWidget_3);
        LE_HANDPOS_Z->setObjectName(QStringLiteral("LE_HANDPOS_Z"));

        gridLayout_4->addWidget(LE_HANDPOS_Z, 2, 1, 1, 1);


        horizontalLayout_7->addLayout(gridLayout_4);

        gridLayout_5 = new QGridLayout();
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_15 = new QLabel(layoutWidget_3);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_5->addWidget(label_15, 2, 0, 1, 1);

        LE_HANDORI_x = new QLineEdit(layoutWidget_3);
        LE_HANDORI_x->setObjectName(QStringLiteral("LE_HANDORI_x"));

        gridLayout_5->addWidget(LE_HANDORI_x, 1, 1, 1, 1);

        LE_HANDORI_w = new QLineEdit(layoutWidget_3);
        LE_HANDORI_w->setObjectName(QStringLiteral("LE_HANDORI_w"));

        gridLayout_5->addWidget(LE_HANDORI_w, 0, 1, 1, 1);

        label_16 = new QLabel(layoutWidget_3);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_5->addWidget(label_16, 1, 0, 1, 1);

        label_17 = new QLabel(layoutWidget_3);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_5->addWidget(label_17, 0, 0, 1, 1);

        LE_HANDORI_z = new QLineEdit(layoutWidget_3);
        LE_HANDORI_z->setObjectName(QStringLiteral("LE_HANDORI_z"));

        gridLayout_5->addWidget(LE_HANDORI_z, 3, 1, 1, 1);

        LE_HANDORI_y = new QLineEdit(layoutWidget_3);
        LE_HANDORI_y->setObjectName(QStringLiteral("LE_HANDORI_y"));

        gridLayout_5->addWidget(LE_HANDORI_y, 2, 1, 1, 1);

        label_18 = new QLabel(layoutWidget_3);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_5->addWidget(label_18, 3, 0, 1, 1);

        LE_HAND_ELB = new QLineEdit(layoutWidget_3);
        LE_HAND_ELB->setObjectName(QStringLiteral("LE_HAND_ELB"));

        gridLayout_5->addWidget(LE_HAND_ELB, 4, 1, 1, 1);

        label_24 = new QLabel(layoutWidget_3);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_5->addWidget(label_24, 4, 0, 1, 1);


        horizontalLayout_7->addLayout(gridLayout_5);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        PB_RHAND_GO = new QPushButton(layoutWidget_3);
        PB_RHAND_GO->setObjectName(QStringLiteral("PB_RHAND_GO"));
        PB_RHAND_GO->setMinimumSize(QSize(0, 40));

        verticalLayout_7->addWidget(PB_RHAND_GO);

        PB_LHAND_GO = new QPushButton(layoutWidget_3);
        PB_LHAND_GO->setObjectName(QStringLiteral("PB_LHAND_GO"));
        PB_LHAND_GO->setMinimumSize(QSize(0, 40));

        verticalLayout_7->addWidget(PB_LHAND_GO);


        horizontalLayout_7->addLayout(verticalLayout_7);

        label_19 = new QLabel(frame_2);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(10, 0, 201, 41));
        layoutWidget_5 = new QWidget(RobotWorldDialog);
        layoutWidget_5->setObjectName(QStringLiteral("layoutWidget_5"));
        layoutWidget_5->setGeometry(QRect(30, 240, 111, 128));
        gridLayout_6 = new QGridLayout(layoutWidget_5);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        LE_VPOS_X = new QLineEdit(layoutWidget_5);
        LE_VPOS_X->setObjectName(QStringLiteral("LE_VPOS_X"));

        gridLayout_6->addWidget(LE_VPOS_X, 0, 1, 1, 1);

        label_22 = new QLabel(layoutWidget_5);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_6->addWidget(label_22, 2, 0, 1, 1);

        label_21 = new QLabel(layoutWidget_5);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_6->addWidget(label_21, 1, 0, 1, 1);

        LE_VPOS_Y = new QLineEdit(layoutWidget_5);
        LE_VPOS_Y->setObjectName(QStringLiteral("LE_VPOS_Y"));

        gridLayout_6->addWidget(LE_VPOS_Y, 1, 1, 1, 1);

        LE_VPOS_Z = new QLineEdit(layoutWidget_5);
        LE_VPOS_Z->setObjectName(QStringLiteral("LE_VPOS_Z"));

        gridLayout_6->addWidget(LE_VPOS_Z, 2, 1, 1, 1);

        label_23 = new QLabel(layoutWidget_5);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_6->addWidget(label_23, 3, 0, 1, 1);

        LE_HAND_ELB_2 = new QLineEdit(layoutWidget_5);
        LE_HAND_ELB_2->setObjectName(QStringLiteral("LE_HAND_ELB_2"));

        gridLayout_6->addWidget(LE_HAND_ELB_2, 3, 1, 1, 1);

        label_20 = new QLabel(layoutWidget_5);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_6->addWidget(label_20, 0, 0, 1, 1);

        PB_GRASP = new QPushButton(RobotWorldDialog);
        PB_GRASP->setObjectName(QStringLiteral("PB_GRASP"));
        PB_GRASP->setGeometry(QRect(150, 200, 111, 61));
        PB_ESTOP = new QPushButton(RobotWorldDialog);
        PB_ESTOP->setObjectName(QStringLiteral("PB_ESTOP"));
        PB_ESTOP->setGeometry(QRect(150, 270, 111, 61));
        PB_PUT = new QPushButton(RobotWorldDialog);
        PB_PUT->setObjectName(QStringLiteral("PB_PUT"));
        PB_PUT->setGeometry(QRect(150, 340, 111, 61));
        layoutWidget_4 = new QWidget(RobotWorldDialog);
        layoutWidget_4->setObjectName(QStringLiteral("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(430, 410, 321, 76));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget_4);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        RB_HANDboth = new QRadioButton(layoutWidget_4);
        RB_HANDboth->setObjectName(QStringLiteral("RB_HANDboth"));

        verticalLayout_8->addWidget(RB_HANDboth);

        RB_HANDr = new QRadioButton(layoutWidget_4);
        RB_HANDr->setObjectName(QStringLiteral("RB_HANDr"));

        verticalLayout_8->addWidget(RB_HANDr);

        RB_HANDl = new QRadioButton(layoutWidget_4);
        RB_HANDl->setObjectName(QStringLiteral("RB_HANDl"));

        verticalLayout_8->addWidget(RB_HANDl);


        horizontalLayout_8->addLayout(verticalLayout_8);

        PB_HAND_GRASP = new QPushButton(layoutWidget_4);
        PB_HAND_GRASP->setObjectName(QStringLiteral("PB_HAND_GRASP"));
        PB_HAND_GRASP->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_GRASP);

        PB_HAND_STOP = new QPushButton(layoutWidget_4);
        PB_HAND_STOP->setObjectName(QStringLiteral("PB_HAND_STOP"));
        PB_HAND_STOP->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_STOP);

        PB_HAND_OPEN = new QPushButton(layoutWidget_4);
        PB_HAND_OPEN->setObjectName(QStringLiteral("PB_HAND_OPEN"));
        PB_HAND_OPEN->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_OPEN);

        PB_WHEEL_MOVE_FRONT = new QPushButton(RobotWorldDialog);
        PB_WHEEL_MOVE_FRONT->setObjectName(QStringLiteral("PB_WHEEL_MOVE_FRONT"));
        PB_WHEEL_MOVE_FRONT->setGeometry(QRect(360, 210, 171, 71));
        PB_WHEEL_MOVE_BACK = new QPushButton(RobotWorldDialog);
        PB_WHEEL_MOVE_BACK->setObjectName(QStringLiteral("PB_WHEEL_MOVE_BACK"));
        PB_WHEEL_MOVE_BACK->setGeometry(QRect(360, 290, 171, 71));
        RB_WST_180 = new QRadioButton(RobotWorldDialog);
        RB_WST_180->setObjectName(QStringLiteral("RB_WST_180"));
        RB_WST_180->setGeometry(QRect(20, 180, 131, 20));

        retranslateUi(RobotWorldDialog);

        QMetaObject::connectSlotsByName(RobotWorldDialog);
    } // setupUi

    void retranslateUi(QDialog *RobotWorldDialog)
    {
        RobotWorldDialog->setWindowTitle(QApplication::translate("RobotWorldDialog", "Dialog", 0));
        PB_WALK_READY->setText(QApplication::translate("RobotWorldDialog", "WalkReady", 0));
        LE_OMNI_ROT->setText(QApplication::translate("RobotWorldDialog", "0.0", 0));
        label_4->setText(QApplication::translate("RobotWorldDialog", "Time (s) :", 0));
        label_2->setText(QApplication::translate("RobotWorldDialog", "Y(m) :", 0));
        LE_OMNI_TIME->setText(QApplication::translate("RobotWorldDialog", "0.0", 0));
        label->setText(QApplication::translate("RobotWorldDialog", "X(m) :", 0));
        LE_OMNI_X->setText(QApplication::translate("RobotWorldDialog", "0.0", 0));
        LE_OMNI_Y->setText(QApplication::translate("RobotWorldDialog", "0.0", 0));
        label_3->setText(QApplication::translate("RobotWorldDialog", "Rot (Deg) :", 0));
        PB_GO_TO_DES_NEW->setText(QApplication::translate("RobotWorldDialog", "Go to Des NEW", 0));
        PB_STOP->setText(QApplication::translate("RobotWorldDialog", "Stop!", 0));
        PB_SAVE->setText(QApplication::translate("RobotWorldDialog", "Save", 0));
        PB_SAVE_START->setText(QApplication::translate("RobotWorldDialog", "DataSaveStart", 0));
        label_12->setText(QApplication::translate("RobotWorldDialog", "X :", 0));
        label_13->setText(QApplication::translate("RobotWorldDialog", "Y :", 0));
        label_14->setText(QApplication::translate("RobotWorldDialog", "Z :", 0));
        label_15->setText(QApplication::translate("RobotWorldDialog", "y :", 0));
        label_16->setText(QApplication::translate("RobotWorldDialog", "x :", 0));
        label_17->setText(QApplication::translate("RobotWorldDialog", "w :", 0));
        label_18->setText(QApplication::translate("RobotWorldDialog", "z :", 0));
        label_24->setText(QApplication::translate("RobotWorldDialog", "Elb:", 0));
        PB_RHAND_GO->setText(QApplication::translate("RobotWorldDialog", "RHAND GO", 0));
        PB_LHAND_GO->setText(QApplication::translate("RobotWorldDialog", "LHAND GO", 0));
        label_19->setText(QApplication::translate("RobotWorldDialog", "<html><head/><body><p><span style=\" font-size:18pt; font-weight:600;\">Grasping test</span></p></body></html>", 0));
        LE_VPOS_X->setText(QApplication::translate("RobotWorldDialog", "0.3", 0));
        label_22->setText(QApplication::translate("RobotWorldDialog", "Z :", 0));
        label_21->setText(QApplication::translate("RobotWorldDialog", "Y :", 0));
        label_23->setText(QApplication::translate("RobotWorldDialog", "Elb:", 0));
        LE_HAND_ELB_2->setText(QApplication::translate("RobotWorldDialog", "-50", 0));
        label_20->setText(QApplication::translate("RobotWorldDialog", "X :", 0));
        PB_GRASP->setText(QApplication::translate("RobotWorldDialog", "Grasp!", 0));
        PB_ESTOP->setText(QApplication::translate("RobotWorldDialog", "E-Stop", 0));
        PB_PUT->setText(QApplication::translate("RobotWorldDialog", "Put!", 0));
        RB_HANDboth->setText(QApplication::translate("RobotWorldDialog", "Both", 0));
        RB_HANDr->setText(QApplication::translate("RobotWorldDialog", "RHAND only", 0));
        RB_HANDl->setText(QApplication::translate("RobotWorldDialog", "LHAND only", 0));
        PB_HAND_GRASP->setText(QApplication::translate("RobotWorldDialog", "Grasp", 0));
        PB_HAND_STOP->setText(QApplication::translate("RobotWorldDialog", "Stop", 0));
        PB_HAND_OPEN->setText(QApplication::translate("RobotWorldDialog", "Open", 0));
        PB_WHEEL_MOVE_FRONT->setText(QApplication::translate("RobotWorldDialog", "WheelMove FRONT", 0));
        PB_WHEEL_MOVE_BACK->setText(QApplication::translate("RobotWorldDialog", "WheelMove BACK", 0));
        RB_WST_180->setText(QApplication::translate("RobotWorldDialog", "WST180Rotation", 0));
    } // retranslateUi

};

namespace Ui {
    class RobotWorldDialog: public Ui_RobotWorldDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTWORLDDIALOG_H
