#include "RBGraph.h"
#include "ui_RBGraph.h"

RBGraph::RBGraph(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RBGraph)
{
    ui->setupUi(this);

    setupGraph(ui->customPlot);

    dataTimer = new QTimer(this);
    connect(dataTimer, SIGNAL(timeout()), this, SLOT(dataSLOT()));
    dataTimer->start(10);

}

RBGraph::~RBGraph()
{
    delete ui;
}


void RBGraph::setupGraph(QCustomPlot *customPlot){
    customPlot->addGraph(); // blue line
    customPlot->graph(0)->setPen(QPen(Qt::blue));

    customPlot->addGraph(); // blue dot
    customPlot->graph(1)->setPen(QPen(Qt::blue));
    customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    customPlot->xAxis->setDateTimeFormat("mm:ss");
    customPlot->xAxis->setAutoTickStep(false);
    customPlot->xAxis->setTickStep(2);
    customPlot->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
}


void RBGraph::dataSLOT(){
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
    static double lastPointKey = 0;

    if (key-lastPointKey > 0.005) // at most add point every 10 ms
    {
        //saveSensorData();
        //double value0 = cPlotData[timerPlotDataIndex1];
        double value;

        // add data to lines:
        ui->customPlot->graph(0)->addData(key, value);
        // set data of dots:
        ui->customPlot->graph(1)->clearData();
        ui->customPlot->graph(1)->addData(key, value);
        // remove data of lines that's outside visible range:
        ui->customPlot->graph(0)->removeDataBefore(key-8);
        // rescale value (vertical) axis to fit the current data:
        ui->customPlot->graph(0)->rescaleValueAxis();
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    ui->customPlot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
    ui->customPlot->replot();

    // calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
        ui->label_PlotStatus->setText(
              QString("%1 FPS, Total Data points: %2")
              .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
              .arg(ui->customPlot->graph(0)->data()->count())
//                    QString("")
              );
        lastFpsKey = key;
        frameCount = 0;
    }
}
