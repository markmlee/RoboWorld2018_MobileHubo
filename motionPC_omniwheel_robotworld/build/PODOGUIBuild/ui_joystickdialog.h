/********************************************************************************
** Form generated from reading UI file 'joystickdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOYSTICKDIALOG_H
#define UI_JOYSTICKDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_JoyStickDialog
{
public:
    QPushButton *JOY_BTN_START;
    QPushButton *JOY_BTN_STOP;
    QTableWidget *JOY_TABLE_INFO_LEFT;
    QTableWidget *JOY_TABLE_INFO_RIGHT;
    QPushButton *JOY_TAB_WHEELSTART;
    QPushButton *JOY_TAB_WHEELSTOP;
    QPushButton *MANUAL_BTN_START;
    QPushButton *MANUAL_BTN_STOP;
    QFrame *line;

    void setupUi(QDialog *JoyStickDialog)
    {
        if (JoyStickDialog->objectName().isEmpty())
            JoyStickDialog->setObjectName(QStringLiteral("JoyStickDialog"));
        JoyStickDialog->resize(773, 520);
        JOY_BTN_START = new QPushButton(JoyStickDialog);
        JOY_BTN_START->setObjectName(QStringLiteral("JOY_BTN_START"));
        JOY_BTN_START->setGeometry(QRect(10, 10, 111, 31));
        JOY_BTN_STOP = new QPushButton(JoyStickDialog);
        JOY_BTN_STOP->setObjectName(QStringLiteral("JOY_BTN_STOP"));
        JOY_BTN_STOP->setEnabled(false);
        JOY_BTN_STOP->setGeometry(QRect(130, 10, 111, 31));
        JOY_TABLE_INFO_LEFT = new QTableWidget(JoyStickDialog);
        if (JOY_TABLE_INFO_LEFT->columnCount() < 1)
            JOY_TABLE_INFO_LEFT->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setHorizontalHeaderItem(0, __qtablewidgetitem);
        if (JOY_TABLE_INFO_LEFT->rowCount() < 9)
            JOY_TABLE_INFO_LEFT->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(0, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(1, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(2, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(3, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(4, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(5, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(6, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(7, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(8, __qtablewidgetitem9);
        JOY_TABLE_INFO_LEFT->setObjectName(QStringLiteral("JOY_TABLE_INFO_LEFT"));
        JOY_TABLE_INFO_LEFT->setGeometry(QRect(10, 50, 121, 271));
        QFont font;
        font.setPointSize(7);
        JOY_TABLE_INFO_LEFT->setFont(font);
        JOY_TABLE_INFO_LEFT->horizontalHeader()->setMinimumSectionSize(8);
        JOY_TABLE_INFO_LEFT->verticalHeader()->setDefaultSectionSize(27);
        JOY_TABLE_INFO_LEFT->verticalHeader()->setMinimumSectionSize(10);
        JOY_TABLE_INFO_RIGHT = new QTableWidget(JoyStickDialog);
        if (JOY_TABLE_INFO_RIGHT->columnCount() < 1)
            JOY_TABLE_INFO_RIGHT->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setHorizontalHeaderItem(0, __qtablewidgetitem10);
        if (JOY_TABLE_INFO_RIGHT->rowCount() < 9)
            JOY_TABLE_INFO_RIGHT->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(0, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(1, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(2, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(3, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(4, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(5, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(6, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(7, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(8, __qtablewidgetitem19);
        JOY_TABLE_INFO_RIGHT->setObjectName(QStringLiteral("JOY_TABLE_INFO_RIGHT"));
        JOY_TABLE_INFO_RIGHT->setGeometry(QRect(130, 50, 121, 271));
        JOY_TABLE_INFO_RIGHT->setFont(font);
        JOY_TABLE_INFO_RIGHT->horizontalHeader()->setMinimumSectionSize(8);
        JOY_TABLE_INFO_RIGHT->verticalHeader()->setDefaultSectionSize(27);
        JOY_TABLE_INFO_RIGHT->verticalHeader()->setMinimumSectionSize(10);
        JOY_TAB_WHEELSTART = new QPushButton(JoyStickDialog);
        JOY_TAB_WHEELSTART->setObjectName(QStringLiteral("JOY_TAB_WHEELSTART"));
        JOY_TAB_WHEELSTART->setEnabled(false);
        JOY_TAB_WHEELSTART->setGeometry(QRect(270, 10, 181, 31));
        JOY_TAB_WHEELSTOP = new QPushButton(JoyStickDialog);
        JOY_TAB_WHEELSTOP->setObjectName(QStringLiteral("JOY_TAB_WHEELSTOP"));
        JOY_TAB_WHEELSTOP->setEnabled(false);
        JOY_TAB_WHEELSTOP->setGeometry(QRect(460, 10, 181, 31));
        MANUAL_BTN_START = new QPushButton(JoyStickDialog);
        MANUAL_BTN_START->setObjectName(QStringLiteral("MANUAL_BTN_START"));
        MANUAL_BTN_START->setEnabled(true);
        MANUAL_BTN_START->setGeometry(QRect(270, 70, 181, 31));
        MANUAL_BTN_STOP = new QPushButton(JoyStickDialog);
        MANUAL_BTN_STOP->setObjectName(QStringLiteral("MANUAL_BTN_STOP"));
        MANUAL_BTN_STOP->setEnabled(true);
        MANUAL_BTN_STOP->setGeometry(QRect(460, 70, 181, 31));
        line = new QFrame(JoyStickDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(250, 10, 20, 501));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        retranslateUi(JoyStickDialog);

        QMetaObject::connectSlotsByName(JoyStickDialog);
    } // setupUi

    void retranslateUi(QDialog *JoyStickDialog)
    {
        JoyStickDialog->setWindowTitle(QApplication::translate("JoyStickDialog", "Dialog", 0));
        JOY_BTN_START->setText(QApplication::translate("JoyStickDialog", "Joy Start", 0));
        JOY_BTN_STOP->setText(QApplication::translate("JoyStickDialog", "Joy Stop", 0));
        QTableWidgetItem *___qtablewidgetitem = JOY_TABLE_INFO_LEFT->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("JoyStickDialog", "Value", 0));
        QTableWidgetItem *___qtablewidgetitem1 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(0);
        ___qtablewidgetitem1->setText(QApplication::translate("JoyStickDialog", "L-LT", 0));
        QTableWidgetItem *___qtablewidgetitem2 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(1);
        ___qtablewidgetitem2->setText(QApplication::translate("JoyStickDialog", "L-LB", 0));
        QTableWidgetItem *___qtablewidgetitem3 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(2);
        ___qtablewidgetitem3->setText(QApplication::translate("JoyStickDialog", "L-JOG-RL", 0));
        QTableWidgetItem *___qtablewidgetitem4 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(3);
        ___qtablewidgetitem4->setText(QApplication::translate("JoyStickDialog", "L-JOG-UD", 0));
        QTableWidgetItem *___qtablewidgetitem5 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(4);
        ___qtablewidgetitem5->setText(QApplication::translate("JoyStickDialog", "L-ARW_RL", 0));
        QTableWidgetItem *___qtablewidgetitem6 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(5);
        ___qtablewidgetitem6->setText(QApplication::translate("JoyStickDialog", "L-ARW_UD", 0));
        QTableWidgetItem *___qtablewidgetitem7 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(6);
        ___qtablewidgetitem7->setText(QApplication::translate("JoyStickDialog", "R-JOG-BTN", 0));
        QTableWidgetItem *___qtablewidgetitem8 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(7);
        ___qtablewidgetitem8->setText(QApplication::translate("JoyStickDialog", "L-JOG-BTN", 0));
        QTableWidgetItem *___qtablewidgetitem9 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(8);
        ___qtablewidgetitem9->setText(QApplication::translate("JoyStickDialog", "BTN-BACK", 0));
        QTableWidgetItem *___qtablewidgetitem10 = JOY_TABLE_INFO_RIGHT->horizontalHeaderItem(0);
        ___qtablewidgetitem10->setText(QApplication::translate("JoyStickDialog", "Value", 0));
        QTableWidgetItem *___qtablewidgetitem11 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(0);
        ___qtablewidgetitem11->setText(QApplication::translate("JoyStickDialog", "R-RT", 0));
        QTableWidgetItem *___qtablewidgetitem12 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(1);
        ___qtablewidgetitem12->setText(QApplication::translate("JoyStickDialog", "R-RB", 0));
        QTableWidgetItem *___qtablewidgetitem13 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(2);
        ___qtablewidgetitem13->setText(QApplication::translate("JoyStickDialog", "R-JOG-RL", 0));
        QTableWidgetItem *___qtablewidgetitem14 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(3);
        ___qtablewidgetitem14->setText(QApplication::translate("JoyStickDialog", "R-JOG-UD", 0));
        QTableWidgetItem *___qtablewidgetitem15 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(4);
        ___qtablewidgetitem15->setText(QApplication::translate("JoyStickDialog", "BTN-Y", 0));
        QTableWidgetItem *___qtablewidgetitem16 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(5);
        ___qtablewidgetitem16->setText(QApplication::translate("JoyStickDialog", "BTN-X", 0));
        QTableWidgetItem *___qtablewidgetitem17 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(6);
        ___qtablewidgetitem17->setText(QApplication::translate("JoyStickDialog", "BTN-B", 0));
        QTableWidgetItem *___qtablewidgetitem18 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(7);
        ___qtablewidgetitem18->setText(QApplication::translate("JoyStickDialog", "BTN-A", 0));
        QTableWidgetItem *___qtablewidgetitem19 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(8);
        ___qtablewidgetitem19->setText(QApplication::translate("JoyStickDialog", "BTN-START", 0));
        JOY_TAB_WHEELSTART->setText(QApplication::translate("JoyStickDialog", "Wheel Manual Move Start", 0));
        JOY_TAB_WHEELSTOP->setText(QApplication::translate("JoyStickDialog", "Wheel Manual Move Stop", 0));
        MANUAL_BTN_START->setText(QApplication::translate("JoyStickDialog", "One Hand", 0));
        MANUAL_BTN_STOP->setText(QApplication::translate("JoyStickDialog", "Manual Stop", 0));
    } // retranslateUi

};

namespace Ui {
    class JoyStickDialog: public Ui_JoyStickDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOYSTICKDIALOG_H
