#include "Panel_FT.h"
#include "ui_Panel_FT.h"

Panel_FT::Panel_FT(QWidget *parent, RBFTSensor *ft) :
    QDialog(parent),
    ui(new Ui::Panel_FT)
{
    ui->setupUi(this);

    dev_info = ft;

    displayTimer = new QTimer();
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(Display()));
    displayTimer->start(100);
}

Panel_FT::~Panel_FT()
{
    delete ui;
}


void Panel_FT::Display(){

}
