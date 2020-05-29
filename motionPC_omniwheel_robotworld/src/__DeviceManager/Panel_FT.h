#ifndef PANEL_FT_H
#define PANEL_FT_H

#include <QDialog>
#include <QTimer>

#include "RBFTSensor.h"

namespace Ui {
class Panel_FT;
}

class Panel_FT : public QDialog
{
    Q_OBJECT

public:
    explicit Panel_FT(QWidget *parent, RBFTSensor *ft);
    ~Panel_FT();

    RBFTSensor      *dev_info;

private slots:
    void Display();

private:
    Ui::Panel_FT *ui;
    QTimer  *displayTimer;


};

#endif // PANEL_FT_H
