#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "functions.cpp"
#include "files.cpp"

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>

#include <QFile>
#include <QTextStream>
#include <QDateTime>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <QStyle>
#include <QDesktopWidget>


#define REL1 6
#define REL2 5
#define REL3 25
#define REL4 9
#define REL5 18
#define REL6 22
#define REL7 23
#define REL8 11

#define RELE1 13
#define RELE2 19
#define RELE3 16
#define RELE4 26
#define RELE5 20
#define RELE6 21

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    wiringPiSetupGpio();
    pinMode(REL1,OUTPUT);
    pinMode(REL2,OUTPUT);
    pinMode(REL3,OUTPUT);
    pinMode(REL4,OUTPUT);
    pinMode(REL5,OUTPUT);
    pinMode(REL6,OUTPUT);
    pinMode(REL7,OUTPUT);
    pinMode(REL8,OUTPUT);

    pinMode(RELE1,OUTPUT);
    pinMode(RELE2,OUTPUT);
    pinMode(RELE3,OUTPUT);
    pinMode(RELE4,OUTPUT);
    pinMode(RELE5,OUTPUT);
    pinMode(RELE6,OUTPUT);


    pinMode(ALARM_OUT,OUTPUT);
    pinMode(ValveExp,OUTPUT);

    ads1115Setup(AD_BASE,0x48);
    digitalWrite(AD_BASE,1);
    timerStatusFlag=false;

    sensorTimer->setTimerType(Qt::PreciseTimer);
    plotTimer->setTimerType(Qt::PreciseTimer);
    controlTimer->setTimerType(Qt::PreciseTimer);

    QObject::connect(sensorTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::sensorTimerFunction));
    QObject::connect(plotTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::plotTimerFunction));
    QObject::connect(controlTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::controlTimerFunction));
    QObject::connect(testTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::testTimerFunction));

    pressData.insert(0,251,0.0);
    volData.insert(0,251,0.0);
    flowData.insert(0,251,0.0);

    increaseVolTemp=0;
    volTemp=0;

    ui->label_press_pip->setNum(setPIP);
    ui->label_fr->setNum(int(FRv));

    pressurePIP=false;
    pressure0=false;
    pressureMAX = false;

    ieRatioRef = 3;
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));

    getDateText();
    plotSetup(ui->customPlot);

    inspirationDetected2 = true;


    for(int i=0;i<=20;i++){
    readedO2 += o2Read();
    }
    readedO2 = readedO2/20;
    ui->label_o2->setText(QString::number(readedO2,'g',2));

    ui->tabWidget->setCurrentIndex(0);
    ui->tabWidget_sel->setCurrentIndex(0);

    AlarmOut();

    for(int x=0;x<=50;x++){
        offsetPressureTempInit+=double(analogRead(AD_BASE));
        offsetFlowTempInit+=double(analogRead(AD_BASE+3));
        delayMicroseconds(5000);
    }
    offsetPressureBits=-offsetPressureTempInit/51.0;
    offsetFlowBits=-offsetFlowTempInit/51.0;

    AlarmOut();

    valvesMainControl(0);
    valvesExControl(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

















