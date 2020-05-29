#ifndef RBGRAPH_H
#define RBGRAPH_H

#include <QDialog>
#include <QTimer>

#include "qcustomplot.h"



namespace Ui {
class RBGraph;
}

class RBGraph : public QDialog
{
    Q_OBJECT

public:
    explicit RBGraph(QWidget *parent = 0);
    ~RBGraph();

private slots:
    void    dataSLOT();

private:
    Ui::RBGraph *ui;
    QTimer  *dataTimer;

    void    setupGraph(QCustomPlot *customPlot);
};

#endif // RBGRAPH_H
