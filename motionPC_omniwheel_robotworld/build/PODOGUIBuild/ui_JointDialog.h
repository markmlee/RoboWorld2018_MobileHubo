/********************************************************************************
** Form generated from reading UI file 'JointDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOINTDIALOG_H
#define UI_JOINTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_JointDialog
{
public:
    QLabel *label;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLineEdit *LE_JOINT_WST;
    QLabel *label_18;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *LE_JOINT_LSP;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *LE_JOINT_LSR;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *LE_JOINT_LSY;
    QLineEdit *LE_JOINT_LEB;
    QLineEdit *LE_JOINT_LWY;
    QLineEdit *LE_JOINT_LWP;
    QLineEdit *LE_JOINT_LHAND;
    QLabel *label_80;
    QLabel *label_83;
    QLineEdit *LE_JOINT_LWY2;
    QWidget *gridLayoutWidget_6;
    QGridLayout *gridLayout_5;
    QLabel *label_27;
    QLabel *label_25;
    QLineEdit *LE_JOINT_LWH;
    QLineEdit *LE_JOINT_RWH;
    QLabel *label_28;
    QLineEdit *LE_JOINT_BWH;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QRadioButton *RB_JOINT_REFERENCE;
    QRadioButton *RB_JOINT_ENCODER;
    QPushButton *BTN_ENC_ENABLE;
    QPushButton *BTN_ENC_DISABLE;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLabel *label_14;
    QLabel *label_15;
    QLineEdit *LE_JOINT_RSP;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *LE_JOINT_RSR;
    QLabel *label_19;
    QLabel *label_20;
    QLineEdit *LE_JOINT_RSY;
    QLineEdit *LE_JOINT_REB;
    QLineEdit *LE_JOINT_RWY;
    QLineEdit *LE_JOINT_RWP;
    QLineEdit *LE_JOINT_RHAND;
    QLabel *label_81;
    QLabel *label_84;
    QLineEdit *LE_JOINT_RWY2;
    QFrame *line;

    void setupUi(QDialog *JointDialog)
    {
        if (JointDialog->objectName().isEmpty())
            JointDialog->setObjectName(QStringLiteral("JointDialog"));
        JointDialog->resize(339, 820);
        label = new QLabel(JointDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 0, 111, 31));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        groupBox = new QGroupBox(JointDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 50, 321, 761));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        groupBox->setFont(font1);
        groupBox->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        gridLayoutWidget_3 = new QWidget(groupBox);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(30, 390, 128, 31));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_WST = new QLineEdit(gridLayoutWidget_3);
        LE_JOINT_WST->setObjectName(QStringLiteral("LE_JOINT_WST"));
        LE_JOINT_WST->setEnabled(false);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_JOINT_WST->sizePolicy().hasHeightForWidth());
        LE_JOINT_WST->setSizePolicy(sizePolicy);
        LE_JOINT_WST->setMinimumSize(QSize(70, 20));
        LE_JOINT_WST->setMaximumSize(QSize(70, 20));
        QFont font2;
        font2.setFamily(QStringLiteral("Serif"));
        font2.setPointSize(9);
        LE_JOINT_WST->setFont(font2);
        LE_JOINT_WST->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_WST->setReadOnly(true);

        gridLayout_3->addWidget(LE_JOINT_WST, 0, 1, 1, 1);

        label_18 = new QLabel(gridLayoutWidget_3);
        label_18->setObjectName(QStringLiteral("label_18"));
        sizePolicy.setHeightForWidth(label_18->sizePolicy().hasHeightForWidth());
        label_18->setSizePolicy(sizePolicy);
        label_18->setMinimumSize(QSize(50, 20));
        label_18->setMaximumSize(QSize(50, 20));
        label_18->setFont(font2);
        label_18->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_18, 0, 0, 1, 1);

        gridLayoutWidget_2 = new QWidget(groupBox);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(30, 150, 128, 211));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(gridLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);
        label_8->setMinimumSize(QSize(50, 20));
        label_8->setMaximumSize(QSize(50, 20));
        label_8->setFont(font2);
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        label_9 = new QLabel(gridLayoutWidget_2);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);
        label_9->setMinimumSize(QSize(50, 20));
        label_9->setMaximumSize(QSize(50, 20));
        label_9->setFont(font2);
        label_9->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_9, 3, 0, 1, 1);

        LE_JOINT_LSP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSP->setObjectName(QStringLiteral("LE_JOINT_LSP"));
        LE_JOINT_LSP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSP->setSizePolicy(sizePolicy);
        LE_JOINT_LSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSP->setFont(font2);
        LE_JOINT_LSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSP, 0, 1, 1, 1);

        label_10 = new QLabel(gridLayoutWidget_2);
        label_10->setObjectName(QStringLiteral("label_10"));
        sizePolicy.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy);
        label_10->setMinimumSize(QSize(50, 20));
        label_10->setMaximumSize(QSize(50, 20));
        label_10->setFont(font2);
        label_10->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_10, 5, 0, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        sizePolicy.setHeightForWidth(label_11->sizePolicy().hasHeightForWidth());
        label_11->setSizePolicy(sizePolicy);
        label_11->setMinimumSize(QSize(50, 20));
        label_11->setMaximumSize(QSize(50, 20));
        label_11->setFont(font2);
        label_11->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_11, 4, 0, 1, 1);

        LE_JOINT_LSR = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSR->setObjectName(QStringLiteral("LE_JOINT_LSR"));
        LE_JOINT_LSR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSR->setSizePolicy(sizePolicy);
        LE_JOINT_LSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSR->setFont(font2);
        LE_JOINT_LSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSR->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSR, 1, 1, 1, 1);

        label_12 = new QLabel(gridLayoutWidget_2);
        label_12->setObjectName(QStringLiteral("label_12"));
        sizePolicy.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy);
        label_12->setMinimumSize(QSize(50, 20));
        label_12->setMaximumSize(QSize(50, 20));
        label_12->setFont(font2);
        label_12->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_12, 2, 0, 1, 1);

        label_13 = new QLabel(gridLayoutWidget_2);
        label_13->setObjectName(QStringLiteral("label_13"));
        sizePolicy.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy);
        label_13->setMinimumSize(QSize(50, 20));
        label_13->setMaximumSize(QSize(50, 20));
        label_13->setFont(font2);
        label_13->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_13, 1, 0, 1, 1);

        LE_JOINT_LSY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSY->setObjectName(QStringLiteral("LE_JOINT_LSY"));
        LE_JOINT_LSY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSY->setSizePolicy(sizePolicy);
        LE_JOINT_LSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSY->setFont(font2);
        LE_JOINT_LSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSY, 2, 1, 1, 1);

        LE_JOINT_LEB = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LEB->setObjectName(QStringLiteral("LE_JOINT_LEB"));
        LE_JOINT_LEB->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LEB->sizePolicy().hasHeightForWidth());
        LE_JOINT_LEB->setSizePolicy(sizePolicy);
        LE_JOINT_LEB->setMinimumSize(QSize(70, 20));
        LE_JOINT_LEB->setMaximumSize(QSize(70, 20));
        LE_JOINT_LEB->setFont(font2);
        LE_JOINT_LEB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LEB->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LEB, 3, 1, 1, 1);

        LE_JOINT_LWY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWY->setObjectName(QStringLiteral("LE_JOINT_LWY"));
        LE_JOINT_LWY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWY->setSizePolicy(sizePolicy);
        LE_JOINT_LWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWY->setFont(font2);
        LE_JOINT_LWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWY, 4, 1, 1, 1);

        LE_JOINT_LWP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWP->setObjectName(QStringLiteral("LE_JOINT_LWP"));
        LE_JOINT_LWP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWP->setSizePolicy(sizePolicy);
        LE_JOINT_LWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWP->setFont(font2);
        LE_JOINT_LWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWP, 5, 1, 1, 1);

        LE_JOINT_LHAND = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LHAND->setObjectName(QStringLiteral("LE_JOINT_LHAND"));
        LE_JOINT_LHAND->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LHAND->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHAND->setSizePolicy(sizePolicy);
        LE_JOINT_LHAND->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHAND->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHAND->setFont(font2);
        LE_JOINT_LHAND->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHAND->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LHAND, 8, 1, 2, 1);

        label_80 = new QLabel(gridLayoutWidget_2);
        label_80->setObjectName(QStringLiteral("label_80"));
        sizePolicy.setHeightForWidth(label_80->sizePolicy().hasHeightForWidth());
        label_80->setSizePolicy(sizePolicy);
        label_80->setMinimumSize(QSize(50, 20));
        label_80->setMaximumSize(QSize(50, 20));
        label_80->setFont(font2);
        label_80->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_80, 8, 0, 2, 1);

        label_83 = new QLabel(gridLayoutWidget_2);
        label_83->setObjectName(QStringLiteral("label_83"));
        sizePolicy.setHeightForWidth(label_83->sizePolicy().hasHeightForWidth());
        label_83->setSizePolicy(sizePolicy);
        label_83->setMinimumSize(QSize(50, 20));
        label_83->setMaximumSize(QSize(50, 20));
        label_83->setFont(font2);
        label_83->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_83, 6, 0, 2, 1);

        LE_JOINT_LWY2 = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWY2->setObjectName(QStringLiteral("LE_JOINT_LWY2"));
        LE_JOINT_LWY2->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWY2->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWY2->setSizePolicy(sizePolicy);
        LE_JOINT_LWY2->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWY2->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWY2->setFont(font2);
        LE_JOINT_LWY2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWY2->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWY2, 6, 1, 2, 1);

        gridLayoutWidget_6 = new QWidget(groupBox);
        gridLayoutWidget_6->setObjectName(QStringLiteral("gridLayoutWidget_6"));
        gridLayoutWidget_6->setGeometry(QRect(30, 450, 128, 74));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_6);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        label_27 = new QLabel(gridLayoutWidget_6);
        label_27->setObjectName(QStringLiteral("label_27"));
        sizePolicy.setHeightForWidth(label_27->sizePolicy().hasHeightForWidth());
        label_27->setSizePolicy(sizePolicy);
        label_27->setMinimumSize(QSize(50, 20));
        label_27->setMaximumSize(QSize(50, 20));
        label_27->setFont(font2);
        label_27->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_27, 1, 0, 1, 1);

        label_25 = new QLabel(gridLayoutWidget_6);
        label_25->setObjectName(QStringLiteral("label_25"));
        sizePolicy.setHeightForWidth(label_25->sizePolicy().hasHeightForWidth());
        label_25->setSizePolicy(sizePolicy);
        label_25->setMinimumSize(QSize(50, 20));
        label_25->setMaximumSize(QSize(50, 20));
        label_25->setFont(font2);
        label_25->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_25, 0, 0, 1, 1);

        LE_JOINT_LWH = new QLineEdit(gridLayoutWidget_6);
        LE_JOINT_LWH->setObjectName(QStringLiteral("LE_JOINT_LWH"));
        LE_JOINT_LWH->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWH->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWH->setSizePolicy(sizePolicy);
        LE_JOINT_LWH->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWH->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWH->setFont(font2);
        LE_JOINT_LWH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWH->setReadOnly(true);

        gridLayout_5->addWidget(LE_JOINT_LWH, 1, 1, 1, 1);

        LE_JOINT_RWH = new QLineEdit(gridLayoutWidget_6);
        LE_JOINT_RWH->setObjectName(QStringLiteral("LE_JOINT_RWH"));
        LE_JOINT_RWH->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWH->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWH->setSizePolicy(sizePolicy);
        LE_JOINT_RWH->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWH->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWH->setFont(font2);
        LE_JOINT_RWH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWH->setReadOnly(true);

        gridLayout_5->addWidget(LE_JOINT_RWH, 0, 1, 1, 1);

        label_28 = new QLabel(gridLayoutWidget_6);
        label_28->setObjectName(QStringLiteral("label_28"));
        sizePolicy.setHeightForWidth(label_28->sizePolicy().hasHeightForWidth());
        label_28->setSizePolicy(sizePolicy);
        label_28->setMinimumSize(QSize(50, 20));
        label_28->setMaximumSize(QSize(50, 20));
        label_28->setFont(font2);
        label_28->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_28, 2, 0, 1, 1);

        LE_JOINT_BWH = new QLineEdit(gridLayoutWidget_6);
        LE_JOINT_BWH->setObjectName(QStringLiteral("LE_JOINT_BWH"));
        LE_JOINT_BWH->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_BWH->sizePolicy().hasHeightForWidth());
        LE_JOINT_BWH->setSizePolicy(sizePolicy);
        LE_JOINT_BWH->setMinimumSize(QSize(70, 20));
        LE_JOINT_BWH->setMaximumSize(QSize(70, 20));
        LE_JOINT_BWH->setFont(font2);
        LE_JOINT_BWH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_BWH->setReadOnly(true);

        gridLayout_5->addWidget(LE_JOINT_BWH, 2, 1, 1, 1);

        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(30, 80, 131, 51));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        RB_JOINT_REFERENCE = new QRadioButton(verticalLayoutWidget);
        RB_JOINT_REFERENCE->setObjectName(QStringLiteral("RB_JOINT_REFERENCE"));
        QFont font3;
        font3.setPointSize(9);
        RB_JOINT_REFERENCE->setFont(font3);
        RB_JOINT_REFERENCE->setChecked(true);

        verticalLayout->addWidget(RB_JOINT_REFERENCE);

        RB_JOINT_ENCODER = new QRadioButton(verticalLayoutWidget);
        RB_JOINT_ENCODER->setObjectName(QStringLiteral("RB_JOINT_ENCODER"));
        RB_JOINT_ENCODER->setFont(font3);

        verticalLayout->addWidget(RB_JOINT_ENCODER);

        BTN_ENC_ENABLE = new QPushButton(groupBox);
        BTN_ENC_ENABLE->setObjectName(QStringLiteral("BTN_ENC_ENABLE"));
        BTN_ENC_ENABLE->setGeometry(QRect(10, 30, 81, 31));
        BTN_ENC_ENABLE->setFont(font3);
        BTN_ENC_DISABLE = new QPushButton(groupBox);
        BTN_ENC_DISABLE->setObjectName(QStringLiteral("BTN_ENC_DISABLE"));
        BTN_ENC_DISABLE->setGeometry(QRect(100, 30, 81, 31));
        BTN_ENC_DISABLE->setFont(font3);
        gridLayoutWidget_4 = new QWidget(groupBox);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(160, 150, 128, 211));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_14 = new QLabel(gridLayoutWidget_4);
        label_14->setObjectName(QStringLiteral("label_14"));
        sizePolicy.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy);
        label_14->setMinimumSize(QSize(50, 20));
        label_14->setMaximumSize(QSize(50, 20));
        label_14->setFont(font2);
        label_14->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_14, 0, 0, 1, 1);

        label_15 = new QLabel(gridLayoutWidget_4);
        label_15->setObjectName(QStringLiteral("label_15"));
        sizePolicy.setHeightForWidth(label_15->sizePolicy().hasHeightForWidth());
        label_15->setSizePolicy(sizePolicy);
        label_15->setMinimumSize(QSize(50, 20));
        label_15->setMaximumSize(QSize(50, 20));
        label_15->setFont(font2);
        label_15->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_15, 3, 0, 1, 1);

        LE_JOINT_RSP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RSP->setObjectName(QStringLiteral("LE_JOINT_RSP"));
        LE_JOINT_RSP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSP->setSizePolicy(sizePolicy);
        LE_JOINT_RSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSP->setFont(font2);
        LE_JOINT_RSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RSP, 0, 1, 1, 1);

        label_16 = new QLabel(gridLayoutWidget_4);
        label_16->setObjectName(QStringLiteral("label_16"));
        sizePolicy.setHeightForWidth(label_16->sizePolicy().hasHeightForWidth());
        label_16->setSizePolicy(sizePolicy);
        label_16->setMinimumSize(QSize(50, 20));
        label_16->setMaximumSize(QSize(50, 20));
        label_16->setFont(font2);
        label_16->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_16, 5, 0, 1, 1);

        label_17 = new QLabel(gridLayoutWidget_4);
        label_17->setObjectName(QStringLiteral("label_17"));
        sizePolicy.setHeightForWidth(label_17->sizePolicy().hasHeightForWidth());
        label_17->setSizePolicy(sizePolicy);
        label_17->setMinimumSize(QSize(50, 20));
        label_17->setMaximumSize(QSize(50, 20));
        label_17->setFont(font2);
        label_17->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_17, 4, 0, 1, 1);

        LE_JOINT_RSR = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RSR->setObjectName(QStringLiteral("LE_JOINT_RSR"));
        LE_JOINT_RSR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSR->setSizePolicy(sizePolicy);
        LE_JOINT_RSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSR->setFont(font2);
        LE_JOINT_RSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSR->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RSR, 1, 1, 1, 1);

        label_19 = new QLabel(gridLayoutWidget_4);
        label_19->setObjectName(QStringLiteral("label_19"));
        sizePolicy.setHeightForWidth(label_19->sizePolicy().hasHeightForWidth());
        label_19->setSizePolicy(sizePolicy);
        label_19->setMinimumSize(QSize(50, 20));
        label_19->setMaximumSize(QSize(50, 20));
        label_19->setFont(font2);
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_19, 2, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));
        sizePolicy.setHeightForWidth(label_20->sizePolicy().hasHeightForWidth());
        label_20->setSizePolicy(sizePolicy);
        label_20->setMinimumSize(QSize(50, 20));
        label_20->setMaximumSize(QSize(50, 20));
        label_20->setFont(font2);
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_20, 1, 0, 1, 1);

        LE_JOINT_RSY = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RSY->setObjectName(QStringLiteral("LE_JOINT_RSY"));
        LE_JOINT_RSY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSY->setSizePolicy(sizePolicy);
        LE_JOINT_RSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSY->setFont(font2);
        LE_JOINT_RSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSY->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RSY, 2, 1, 1, 1);

        LE_JOINT_REB = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_REB->setObjectName(QStringLiteral("LE_JOINT_REB"));
        LE_JOINT_REB->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_REB->sizePolicy().hasHeightForWidth());
        LE_JOINT_REB->setSizePolicy(sizePolicy);
        LE_JOINT_REB->setMinimumSize(QSize(70, 20));
        LE_JOINT_REB->setMaximumSize(QSize(70, 20));
        LE_JOINT_REB->setFont(font2);
        LE_JOINT_REB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_REB->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_REB, 3, 1, 1, 1);

        LE_JOINT_RWY = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RWY->setObjectName(QStringLiteral("LE_JOINT_RWY"));
        LE_JOINT_RWY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWY->setSizePolicy(sizePolicy);
        LE_JOINT_RWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWY->setFont(font2);
        LE_JOINT_RWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWY->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RWY, 4, 1, 1, 1);

        LE_JOINT_RWP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RWP->setObjectName(QStringLiteral("LE_JOINT_RWP"));
        LE_JOINT_RWP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWP->setSizePolicy(sizePolicy);
        LE_JOINT_RWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWP->setFont(font2);
        LE_JOINT_RWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RWP, 5, 1, 1, 1);

        LE_JOINT_RHAND = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHAND->setObjectName(QStringLiteral("LE_JOINT_RHAND"));
        LE_JOINT_RHAND->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RHAND->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHAND->setSizePolicy(sizePolicy);
        LE_JOINT_RHAND->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHAND->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHAND->setFont(font2);
        LE_JOINT_RHAND->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHAND->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHAND, 8, 1, 2, 1);

        label_81 = new QLabel(gridLayoutWidget_4);
        label_81->setObjectName(QStringLiteral("label_81"));
        sizePolicy.setHeightForWidth(label_81->sizePolicy().hasHeightForWidth());
        label_81->setSizePolicy(sizePolicy);
        label_81->setMinimumSize(QSize(50, 20));
        label_81->setMaximumSize(QSize(50, 20));
        label_81->setFont(font2);
        label_81->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_81, 8, 0, 2, 1);

        label_84 = new QLabel(gridLayoutWidget_4);
        label_84->setObjectName(QStringLiteral("label_84"));
        sizePolicy.setHeightForWidth(label_84->sizePolicy().hasHeightForWidth());
        label_84->setSizePolicy(sizePolicy);
        label_84->setMinimumSize(QSize(50, 20));
        label_84->setMaximumSize(QSize(50, 20));
        label_84->setFont(font2);
        label_84->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_84, 6, 0, 2, 1);

        LE_JOINT_RWY2 = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RWY2->setObjectName(QStringLiteral("LE_JOINT_RWY2"));
        LE_JOINT_RWY2->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWY2->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWY2->setSizePolicy(sizePolicy);
        LE_JOINT_RWY2->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWY2->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWY2->setFont(font2);
        LE_JOINT_RWY2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWY2->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RWY2, 6, 1, 2, 1);

        line = new QFrame(JointDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(10, 30, 61, 16));
        QFont font4;
        font4.setPointSize(8);
        line->setFont(font4);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        retranslateUi(JointDialog);

        QMetaObject::connectSlotsByName(JointDialog);
    } // setupUi

    void retranslateUi(QDialog *JointDialog)
    {
        JointDialog->setWindowTitle(QApplication::translate("JointDialog", "Dialog", 0));
        label->setText(QApplication::translate("JointDialog", "Joint", 0));
        groupBox->setTitle(QApplication::translate("JointDialog", "Joint Ref && Encr", 0));
        label_18->setText(QApplication::translate("JointDialog", "WST", 0));
        label_8->setText(QApplication::translate("JointDialog", "LSP", 0));
        label_9->setText(QApplication::translate("JointDialog", "LEB", 0));
        label_10->setText(QApplication::translate("JointDialog", "LWP", 0));
        label_11->setText(QApplication::translate("JointDialog", "LWY", 0));
        label_12->setText(QApplication::translate("JointDialog", "LSY", 0));
        label_13->setText(QApplication::translate("JointDialog", "LSR", 0));
        label_80->setText(QApplication::translate("JointDialog", "LHand", 0));
        label_83->setText(QApplication::translate("JointDialog", "LWY2", 0));
        label_27->setText(QApplication::translate("JointDialog", "LWH", 0));
        label_25->setText(QApplication::translate("JointDialog", "RWH", 0));
        label_28->setText(QApplication::translate("JointDialog", "BWH", 0));
        RB_JOINT_REFERENCE->setText(QApplication::translate("JointDialog", "Reference", 0));
        RB_JOINT_ENCODER->setText(QApplication::translate("JointDialog", "Encoder", 0));
        BTN_ENC_ENABLE->setText(QApplication::translate("JointDialog", "Enc.Enable", 0));
        BTN_ENC_DISABLE->setText(QApplication::translate("JointDialog", "Enc. Disable", 0));
        label_14->setText(QApplication::translate("JointDialog", "RSP", 0));
        label_15->setText(QApplication::translate("JointDialog", "REB", 0));
        label_16->setText(QApplication::translate("JointDialog", "RWP", 0));
        label_17->setText(QApplication::translate("JointDialog", "RWY", 0));
        label_19->setText(QApplication::translate("JointDialog", "RSY", 0));
        label_20->setText(QApplication::translate("JointDialog", "RSR", 0));
        label_81->setText(QApplication::translate("JointDialog", "RHand", 0));
        label_84->setText(QApplication::translate("JointDialog", "RWY2", 0));
    } // retranslateUi

};

namespace Ui {
    class JointDialog: public Ui_JointDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOINTDIALOG_H
