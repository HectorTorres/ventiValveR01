#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <math.h>

//Volumen Inspirado VTI
//Volumen expirado VTE
//Max pression
//Vol/min

//Alarma de apnea





#define DEBUG_STATUS 1
#define DEBUG_STATUS_HIGH_SPEED 0

#define TIMER_DELAY 40
#define TIMER_PLOT 100
#define TIMER_SENSOR 40

#define RISE_TIME_COMPENSATOR 50

#define TRESHOLD_TA 7
#define PEEP_VAL 5
#define TRESHOLD_TD 15

#define PRESS_LIMIT_MAX 50

#define DEBOUNCE_TIME 500000

#define ValveExp 7
#define ALARM_OUT 17

#define RG 2000
#define V0 1600
#define Vmax 26392
#define SLOPE_PRESSURE_SENSOR 0.26

#define AD_BASE 120

#define DATA 12
#define CLK 12
#define LATCH 12

#define REL1 6 //2
#define REL2 5 //4
#define REL3 25 //6
#define REL4 9 //8
#define REL5 18 //2
#define REL6 22 //4
#define REL7 23 //6
#define REL8 11 //8

#define RELE1 13
#define RELE2 19
#define RELE3 16
#define RELE4 26
#define RELE5 20
#define RELE6 21

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* PLOT FUNCTION
 * Plots data, check the timer starts from this function to see
 * which is the timeout set */
void MainWindow::plotTimerFunction(){
    plotData(ui->customPlot);
    ui->customPlot->replot();

    readedO2 = o2Read();
    ui->label_o2->setText(QString::number(readedO2,'g',2));
    /*
    if(readedO2 >= 90){
        activateAlarm(5);
    }
    else if (readedO2 <= 10) {
        activateAlarm(6);
    }*/
}

void MainWindow::validacion(){
    while(true){
        valvesMainControl(0x01); //Inspiracion
        qDebug() << "Activacion";
        delayMicroseconds(128000000);
        valvesMainControl(0x00); //Inspiracion

        valvesExControl(0x3F); //Exalación
        qDebug() << "Desactivado";
        delayMicroseconds(4000000);
        valvesExControl(0x00); //Exalación
    }
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR TIMER FUNCTION
 * reads and checs the preassure sensor data */
void MainWindow::sensorTimerFunction(){

     readedPress=pressureRead();

     pressProm[0] = pressProm[1]; pressProm[1] = pressProm[2]; pressProm[2] = pressProm[3]; pressProm[3] = pressProm[4]; pressProm[4] = pressProm[5];  pressProm[5] = readedPress;
     double readedPressProm = (pressProm[0]+pressProm[1]+pressProm[2]+pressProm[3]+pressProm[4]+pressProm[5])/6.0;
     readedPress = readedPressProm;

     if(readedPress >= maxPressLimit){
         valvesMainControl(0x00);
         valvesExControl(0x01);
         activateAlarm(1);
         qDebug() << "Alarma de activación alta.";
     }

     if(readedPress <= 1 && ((millis()-timeFromStart) >= 3000)){
        valvesMainControl(0x00);
        valvesExControl(0x01);
        activateAlarm(2);
        qDebug() << "Alarma de activación baja.";
     }


/*
    if((readedPress/readedPressTempD) >= 4 || (readedPress/readedPressTempD) <= 0.1 ){
        readedPress=readedPressTempD;
    }*/
    readedPressTempD = readedPress;


//Pendiente--------------------------------------------------------
    if(indexPress<=4){
        pressSlope.replace(indexPress,readedPress);
    }
    else {
        std::rotate(pressSlope.begin(),pressSlope.begin()+1,pressSlope.end());
        pressSlope.replace(4,readedPress);
    }
   double slope=0;
   slope=std::accumulate(pressSlope.begin(),pressSlope.end(),slope);
   slope=atan(((pressSlope[0]-pressSlope[1])+(pressSlope[1]-pressSlope[2])+(pressSlope[2]-pressSlope[3])+(pressSlope[3]-pressSlope[4]))/5)*180/3.1415;



   if(ui->radioButton_assit->isChecked() || ui->radioButton_esp->isChecked()){
        if(slope >= double(ui->horizontalSlider_sensibilidad->value())*6.1111+23.889){ // f001 Evaluación de el nuevo valor de la pendiente.
            qDebug() << "Slope angle inspiratio:" <<slope;
            timeMasterControl+=double(60.0/double(FRv))*1000;
            inspirationDetected = true;
        }
   }


    if(slopeIncrease == true && (millis()-timerSlopeIncrease) >= timeFRv){
        slopeIncrease = false;
    }

if(ui->checkBox_alarmNoConnection->isChecked()){
    if(slope<=1 && slope>=-1 && slopeFlat == false){
        qDebug() << "Deteccion de desconexión";
        timeFlatSlope = millis();
        slopeFlat=true;
     }

   if(slope<=1 && slope>=-1 && slopeFlat == true && (millis()-timeFlatSlope)>4000){
       qDebug() << "Deteccion de desconexión";
       timeFlatSlope = 0;
       activateAlarm(9);
       slopeFlat=false;
    }
}



   //readedFlow =(flowRead());
   readedFlow = (valvesValueControl*8.0)/1.0;

   flowProm[0] = flowProm[1]; flowProm[1] = flowProm[2]; flowProm[2] = flowProm[3]; flowProm[3] = flowProm[4]; flowProm[4] = flowProm[5];  flowProm[5] = readedFlow;
   double readedFlowProm = (flowProm[0]+flowProm[1]+flowProm[2]+flowProm[3]+flowProm[4]+flowProm[5])/6.0;
   readedFlow = readedFlowProm;

    if(flowDataReadStatus==false){
        readedFlow = 0;
        volTemp=0;
        //qDebug() << "zzzzzzzzzzzzzzzzzzzzzzz: ";
    }


    readedVol = volRead(readedFlow);


    //Almacenar los datos en los vectores_______________________________________________
    pressData.replace(indexPress,readedPress);
    flowData.replace(indexPress,readedFlow);
    volData.replace(indexPress,readedVol);
    indexPress++;
    if(indexPress>=251) {
        indexPress=0;
    }
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::controlTimerFunction(){

        periodTime = double(60.0/double(FRv))*1000;
        inspirationTime = periodTime/(ieRatioRef+1);

        //Inspiracion mandatoria y asistida


        if(!ui->radioButton_esp->isChecked()){

        if(((millis()-timeMasterControl) >= uint32_t(inspirationTime)) && (inspirationTimeTop == false)){
            valvesMainControl(0x00);
            delayMicroseconds(10000);
            valvesExControl(0x01);
            inspirationTimeTop = true;
            flowDataReadStatus = false;

        // Sistema de control básico./////////////////////////////////////////////////////
         if(ui->checkBox_control->isChecked()){
             if(ui->tabWidget_sel->currentIndex()==0){
                if(readedPress<=setPIP){
                    valvesValueControl++;
                }
                else {
                    valvesValueControl--;
                }
            }
             if(ui->tabWidget_sel->currentIndex()==1){
                 if(readedVol<=setVOL){
                     valvesValueControl++;
                 }
                 else {
                     valvesValueControl--;
                 }

             }
          }
         // /////////////////////////////////////////////////////



        //ui->la
        ui->label_vti->setText(QString::number(readedVol));
        ui->label_press_maxPress->setText(QString::number(readedPress,'f',1)); // f001 Impresion de presion maxima
            qDebug() << "Presión máxima: " << readedPress << " Presion Objetivo: " << setPIP << " Porcentaje de diferencia: " << (1-double(readedPress)/double(setPIP))*100 << "%   Valvula: " << valvesValueControl;

        }

        if(((millis()-timeMasterControl)>=periodTime) && (inspirationTimeTop == true)){
            cicleCounter++;
            ui->label_ciclos->setText(QString::number(cicleCounter));

            periodTimeRead = (millis()-timeMasterControl); //Alarmas
            if(60000/periodTime >= 180){
                valvesMainControl(0);
                valvesExControl(1);
                activateAlarm(3);
            }
            if(60000/periodTime <= 1){
                valvesMainControl(0);
                valvesExControl(1);
                activateAlarm(4);
            }

            ui->label_fr_current->setText(QString::number((1000.0/double(millis()-timeMasterControl))*60.0,'f',1));
            ui->label_press->setText("1:" + QString::number(ieRatioRef));
            timeMasterControl = millis();
            valvesExControl(0x00);
            delayMicroseconds(10000);
            valvesMainControl(uint8_t(valvesValueControl));
            inspirationTimeTop = false;
            flowDataReadStatus = true;
            }
        }
        
        //Inspiracion espontanea

        if(ui->radioButton_esp->isChecked()){
            if(inspirationDetected){
                espontaneoPeriod=millis();
                cicleCounter++;
                ui->label_ciclos->setText(QString::number(cicleCounter));
                
                valvesExControl(0x00);
                delayMicroseconds(10000);
                valvesMainControl(uint8_t(valvesValueControl));
                inspirationDetected = false;
                inspirationDetected2 = true;
            }           
            if((millis()-espontaneoPeriod) >= uint32_t(inspirationTime) && inspirationDetected2==true){
                valvesMainControl(0x00);
                delayMicroseconds(10000);
                valvesExControl(0x01);
                inspirationDetected2 = false;

                ui->label_press_maxPress->setText(QString::number(readedPress)); // f001 Impresion de presion maxima
                ui->label_fr_current->setText("NA");
                ui->label_press->setText("1:" + QString::number(ieRatioRef));

            }
            if((millis()-espontaneoPeriod)>=20000){
                AlarmOut();
                inspirationDetected=true;
            }
        }



    /*
    if((readedPress>=setPIP || readedVol >=40) && readedPressTop==false ){ // Punto alto
        qDebug() << "-----------------------------------------------";
        readedPressTop=true;
        flowDataReadStatus=false;
        timeUpIe = millis()-timeFRvTemporal;
        readedPressBottom=false;

        //valvesMainControl(0); //
       // valvesExControl(0);

        readedVol=0;
        volTemp = 0;

        delayMicroseconds(timeFRv/10);

    }

    if(((millis()-timeFRvTemporal)>=timeFRv) ){ //Fin de periodo
        qDebug() << "-----------------------------------------------------------------------------------------------------------------";
        timeFRvReal=millis()-timeFRvTemporal;
        timeFRvRealf = (1000.0/double(timeFRvReal))*60.0;
        ui->label_fr_current->setText(QString::number(timeFRvRealf,'f',2));
        IeRatio = double(timeUpIe)/double(timeFRvReal);
        timeDownIe = timeFRvReal - timeUpIe;
        ui->label_press->setText("1:" + QString::number((1/IeRatio)-1,'f',1));
        qDebug() << "Periodo: " << timeFRvReal << " Tiempo A: " << timeUpIe << "    Relacion: " << IeRatio;
        flowDataReadStatus=true;


        timeFRvTemporal=millis();
        readedPressBottom=true;
        readedPressTop=false;

        readedVol=0;
        volTemp = 0;

       // valvesMainControl(0); //
        //valvesExControl(0);
///////////////////////////////////////////////




        qDebug() << "Tiempos  " << IeRatio << " > " << IeRatioSetPoint << "   Mayor: " << IeRatio;

        int differenceIe = -int((IeRatioSetPoint - IeRatio)*40);
        qDebug() << "Diferencia real: " << (IeRatioSetPoint - IeRatio);

        if(differenceIe >= 4 ){
            differenceIe = 4;
        }
        else if (differenceIe <= -4){
            differenceIe = -4;
        }

            IeProm[0] = IeProm[1];
            IeProm[1] = IeProm[2];
            IeProm[2] = IeProm[3];
            IeProm[3] = IeProm[4];
            IeProm[4] = IeProm[5];
            IeProm[5] = differenceIe;

        double IePromInteg = (IeProm[0]+IeProm[1]+IeProm[2]+IeProm[3]+IeProm[4]+IeProm[5])/6.0;

        qDebug() << "diffProm:" << IePromInteg;
        qDebug() << "inspirationValvesV: " << inspirationValvesV;

        if((inspirationValvesV+int(IePromInteg)) < 5 ){
            differenceIe = 0;
        }

        inspirationValvesV+=int(IePromInteg);
        if(inspirationValvesV <= 13){
            inspirationValvesV=13;
        }

        qDebug() << "inspirationValvesV2: " << inspirationValvesV;


///////////////////////////////////////////////

    }

    if(readedPressBottom){
    valvesMainControl(inspirationValvesV); //Inspiracion
    valvesExControl(0); //Exalación
    readedPressBottom=false;
    qDebug() << "exa: "<< 0 << " Ina: " << inspirationValvesV;
    }

    if(readedPressTop){

    valvesMainControl(0); //Inspiracion
    valvesExControl(0x3F); //Exalación
    readedPressTop = false;
    //delayMicroseconds((timeFRv/10)*1000);
    qDebug() << "exa: "<< exalationValvesV << " Ina: " << 0;
    }
    //qDebug() << readedPress;

   // qDebug() << "zzzzzzzzzzzzzzzzzzzzzzz: " << readedPressBottom;
*/
}












/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_3_clicked()
{
    ui->tabWidget->setCurrentIndex(0);
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_resetC_clicked()
{
    cicleCounter = 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::pressureRead(){
    uint16_t bitsA0 = uint16_t(analogRead(AD_BASE));
    //qDebug() << (double(bitsA0));
    //double kPaSensor = ((double(bitsA0)-12265.0)*(30.0/6028.0)*1.25)+offsetPressure;
    //double kPaSensor = ((double(bitsA0)-12265.0)*(30.0/6028.0)*1.25)+offsetPressure;
    double kPaSensor = (double(bitsA0)+offsetPressureBits)*0.0055 + 1.6324;

    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de presión: " << kPaSensor << " mmH2O." ; }
    return kPaSensor;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::flowRead(){   
    uint16_t bitsD23 = uint16_t(analogRead(AD_BASE+3));
    //double flowSensor = double(bitsD23)*0.041-879.9+71;
    double flowSensor = (double(bitsD23)+offsetPressureBits);
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de flujo: " << flowSensor << " ml/min." ; }
    //qDebug() << "Sensor de flujo: " << flowSensor << " ml/min.";
    //if(flowSensor>=40) {flowSensor = 0;}
    return flowSensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::volRead(double flowIn){
    //double volTemp2 = ((volTemp+flowIn)*(0.003))*2.65;
    double volTemp2 = (flowIn*(TIMER_SENSOR/1000.0))*0.8;
    volTemp = volTemp+volTemp2;
    //qDebug() << "Volumen: " << volTemp << " ml.";
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Volumen: " << volTemp << " ml." ; }
    return volTemp;
}

double MainWindow::o2Read(){
    uint16_t bitsA1 = uint16_t(analogRead(AD_BASE+1));
    double o2Sensor = double(bitsA1);
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Saturación de O2: " << o2Sensor << " %." ; }
   // qDebug() << "Saturación de O2: " << o2Sensor << " %." ;
    return o2Sensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::plotData(QCustomPlot *customPlot)
{
  customPlot->graph(0)->setData(x, pressData);
  customPlot->graph(1)->setData(x, flowData);
  customPlot->graph(2)->setData(x, volData);
  minPEEP = *std::min_element(pressData.begin(), pressData.end())+0.2;
  if(minPEEP <= 1){
      minPEEP = 5.2;
  }

  if(ui->tabWidget_sel->currentIndex()==0 ){
        customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
        customPlot->graph(2)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
        customPlot->yAxis->setRange(-1,25);
        customPlot->yAxis2->setRange(-1,25);
        QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
        customPlot->yAxis->setTicker(fixedTickerY);
        fixedTickerY->setTickStep(2);
        fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
        customPlot->yAxis->setLabel("Presion [cmH2O]");
  }

  if(ui->tabWidget_sel->currentIndex()==1 ){
      customPlot->graph(2)->setBrush(QBrush(QColor(255, 0, 0, 20))); // first graph will be filled with translucent blue
      customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
      customPlot->yAxis->setRange(-3,1000);
      customPlot->yAxis2->setRange(-3,1000);
      QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
      customPlot->yAxis->setTicker(fixedTickerY);
      fixedTickerY->setTickStep(100);
      fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
      customPlot->yAxis->setLabel("Flujo [ml/s] / Volumen [ml]");
  }

  ui->label_PEEP->setText(QString::number(minPEEP,'g',3));
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::plotSetup(QCustomPlot *customPlot){
    x.insert(0,251,0.0);
    for (int i=0; i<251; ++i)
    {
      x.replace(i,i*TIMER_SENSOR);
    }
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::darkGreen)); // line color red for second graph
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::red)); // line color red for second graph

    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setRange(0,251);
    customPlot->yAxis->setRange(-3,40);
    customPlot->yAxis2->setRange(-3,40);
    customPlot->xAxis->setLabel("Tiempo [ms]");
    customPlot->yAxis->setLabel("Pr [cmH2O] / Fl [l/min] / Vol [ml]");

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    customPlot->xAxis->setTicker(timeTicker);

    customPlot->xAxis->setRange(0,251*TIMER_SENSOR);
    timeTicker->setTimeFormat("%z");

    QSharedPointer<QCPAxisTickerFixed> fixedTicker(new QCPAxisTickerFixed);
    customPlot->xAxis->setTicker(fixedTicker);
    fixedTicker->setTickStep(350);
    fixedTicker->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
    customPlot->yAxis->setTicker(fixedTickerY);
    fixedTickerY->setTickStep(3);
    fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    customPlot->xAxis->setTickLabelRotation(45);
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_start_clicked()
{
    if(timerStatusFlag){
        valvesMainControl(0);
        valvesExControl(1);
        timeFromInit=0;
        controlTimer->stop();
        sensorTimer->stop();
        plotTimer->stop();
        ui->pushButton_start->setText("Inicio.");
        timerStatusFlag=false;
        timeFromStart = 0;
        if(DEBUG_STATUS){   qDebug() << "Timer Stops."; }
    }
    else {
           // validacion();
            readedPressBottom = true;
            timeFRvTemporal = millis();
            timeMasterControl = millis();
            timeFromStart=millis();
            inspirationTimeTop = false;  // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
            vavleChange=true;
            controlTimer->start(TIMER_DELAY);
            sensorTimer->start(TIMER_SENSOR);
            plotTimer->start(TIMER_PLOT);
            ui->pushButton_start->setText("Paro.");
            timerStatusFlag=true;
            if(DEBUG_STATUS){   qDebug() << "Timer Starts."; }
            //valvesMainControl(0); // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
        }
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_morePIP_clicked()
{
    setPIP+=0.5;
    if(setPIP>=25) {setPIP  = 25;}
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minPIP_clicked()
{
    setPIP-=0.5;
    if(setPIP<=6) {setPIP  = 6;}
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minVol_clicked()
{
    setVOL-=50;
    if(setVOL<=50) {setVOL = 50;}
    ui->label_press_volsetpoint->setNum(setVOL);
    evalVel();
}

void MainWindow::on_pushButton_moreVol_clicked()
{
    setVOL+=50;
    if(setVOL>=2050) {setVOL = 2050;}
    ui->label_press_volsetpoint->setNum(setVOL);
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_moreFR_clicked()
{
    FRv=FRv+1;
    if(FRv>=150) {FRv = 150;}
    timeFRv = double(60.0/double(FRv))*1000.0;
    ui->label_fr->setNum(int(FRv));
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minFR_clicked()
{
    FRv=FRv-1;
    if(FRv<=4) {FRv = 4;}
    timeFRv = double(60.0/double(FRv))*1000.0;
    ui->label_fr->setNum(int(FRv));
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::printTimer(QString info){
    timerMillis=millis();
    if(DEBUG_STATUS){   qDebug() << info << timerMillis;}
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_mor_ie_clicked()
{
    ieRatioRef = ieRatioRef+1;
    if(ieRatioRef>=5) {ieRatioRef = 5;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_min_ie_clicked()
{
    ieRatioRef = ieRatioRef - 1;
    if(ieRatioRef<=1) {ieRatioRef = 1;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_moreValves_clicked()
{
    valvesValueControl++;
    if(valvesValueControl>=255) {valvesValueControl = 255;}
    ui->label_valves_value->setText(QString::number(valvesValueControl));
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minValves_clicked()
{
    valvesValueControl--;
    if(valvesValueControl<=1) {valvesValueControl = 1;}
    ui->label_valves_value->setText(QString::number(valvesValueControl));
}
/*------------------------------------------------------------------------------------------------------------------------------------*/








void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << "Presion: " << pressureRead();
    qDebug() << "O2: " << o2Read();
    qDebug() << "Flujo: " << flowRead();
}

void MainWindow::on_pushButton_min_maxPress_clicked()
{
    maxPressLimit -= 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}

void MainWindow::on_pushButton_mor_maxPress_clicked()
{
    maxPressLimit += 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}

void MainWindow::on_pushButton_Conf_clicked()
{
    ui->tabWidget->setCurrentIndex(1);
}






void MainWindow::activateAlarm(uint16_t number){
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << number; }
    errorFrCounter=0;
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << errorFrCounter; }

    on_pushButton_start_clicked();
    evalVel();
    AlarmOut();
    digitalWrite(RELE4,HIGH);

    switch (number) {
    case 1: {
        QMessageBox::critical(this,"Error!.","Presión alta!.","Aceptar.");
        break;
    }
    case 2: {
        QMessageBox::critical(this,"Error!.","Presión baja!.","Aceptar.");
        break;
    }
    case 3: {
        QMessageBox::critical(this,"Error!.","Frecuencia respiratoria alta!.","Aceptar.");
        break;
    }
    case 4: {
        QMessageBox::critical(this,"Error!.","Frecuencia respiratoria baja!.","Aceptar.");
        break;
    }
    case 5: {
        QMessageBox::critical(this,"Error!.","O2 alto!.","Aceptar.");
        break;
    }
    case 6: {
        QMessageBox::critical(this,"Error!.","O2 bajo!.","Aceptar.");
        break;
    }
    case 7: {
        QMessageBox::critical(this,"Error!.","Volumen bajo!.","Aceptar.");
        break;
    }
    case 8: {
        QMessageBox::critical(this,"Error!.","Volumen alto!.","Aceptar.");
        break;
    }
    case 9: {
        QMessageBox::critical(this,"Error!.","Fuga o desconexión!.","Aceptar.");
        break;
    }
    }
    digitalWrite(RELE4,LOW);
}



void MainWindow::AlarmOut(){
    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(100000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);
}


void MainWindow::valvesMainControl(uint8_t inspirationValves ){
delayMicroseconds(100);

digitalWrite(REL1,(inspirationValves &  0x01));
digitalWrite(REL2,(inspirationValves &  0x02));
digitalWrite(REL4,(inspirationValves &  0x04));
digitalWrite(REL3,(inspirationValves &  0x08));
digitalWrite(REL5,(inspirationValves &  0x10));
digitalWrite(REL6,(inspirationValves &  0x20));
digitalWrite(REL7,(inspirationValves &  0x40));
digitalWrite(REL8,(inspirationValves &  0x80));
//
delayMicroseconds(5000);
}

void MainWindow::valvesExControl(uint8_t exalationValves ){
delayMicroseconds(100);

digitalWrite(RELE1,(exalationValves &  0x01));
digitalWrite(RELE2,(exalationValves &  0x02));
digitalWrite(RELE4,(exalationValves &  0x04));
digitalWrite(RELE3,(exalationValves &  0x08));
digitalWrite(RELE5,(exalationValves &  0x10));
digitalWrite(RELE6,(exalationValves &  0x20));
delayMicroseconds(5000);
//

}










void MainWindow::on_radioButton_name_clicked()
{
    if(ui->radioButton_name->isChecked()){
        ui->lineEdit_textName->setEnabled(true);
    }
    else{
        ui->lineEdit_textName->setEnabled(false);
    }
}

void MainWindow::on_radioButton_date_clicked()
{
    if(ui->radioButton_date->isChecked()){
        ui->lineEdit_textName->setEnabled(false);
    }
    else{
        ui->lineEdit_textName->setEnabled(true);
    }
}


void MainWindow::on_pushButton_10_clicked()
{
    testTimer->start(200);
}

void MainWindow::testTimerFunction(){

    qDebug() << "Presion: " << pressureRead() << "      Flujo: " << flowRead();

}


void MainWindow::on_pushButton_stop_clicked()
{
    testTimer->stop();
}



void MainWindow::on_pushButton_2_clicked()
{
    uint8_t inalacion=0;
    uint8_t exalacion=0;
    if(ui->checkBox->isChecked()){inalacion+=0x01;}
    if(ui->checkBox_2->isChecked()){inalacion+=0x02;}
    if(ui->checkBox_3->isChecked()){inalacion+=0x04;}
    if(ui->checkBox_4->isChecked()){inalacion+=0x08;}
    if(ui->checkBox_5->isChecked()){inalacion+=0x10;}
    if(ui->checkBox_6->isChecked()){inalacion+=0x20;}
    if(ui->checkBox_7->isChecked()){inalacion+=0x40;}
    if(ui->checkBox_8->isChecked()){inalacion+=0x80;}


    if(ui->checkBox_E1->isChecked()){exalacion+=0x01;}
    if(ui->checkBox_E2->isChecked()){exalacion+=0x02;}
    if(ui->checkBox_E3->isChecked()){exalacion+=0x04;}
    if(ui->checkBox_E4->isChecked()){exalacion+=0x08;}
    if(ui->checkBox_E5->isChecked()){exalacion+=0x10;}
    if(ui->checkBox_E6->isChecked()){exalacion+=0x20;}

    qDebug() << "Inalacion: " << inalacion << " || Exalación: " << exalacion;


    valvesMainControl(inalacion);
    valvesExControl(exalacion);


}

void MainWindow::on_pushButton_5_clicked()
{

}


void MainWindow::on_horizontalSlider_sensibilidad_sliderMoved(int position)
{
    ui->label_sensibilidad->setText(QString::number(position));
}

void MainWindow::on_pushButton_clicked()
{
    for(int i=0;i<=50;i++){
        offsetPressureTemp+=pressureRead();
        offsetFlowTemp+=flowRead();
        delayMicroseconds(50000);
        ui->progressBar_offset->setValue(i);
    }
    offsetPressure=-offsetPressureTemp/51.0;
    offsetFlow=-offsetFlowTemp/51.0;
    qDebug() << "Presion offset: " << offsetPressure << " - Flujo offset: " << offsetFlowTemp;
}


void MainWindow::evalVel(){
        if(ui->tabWidget_sel->currentIndex()==1){
           // periodTime = double(60.0/double(FRv))*1000;
           // inspirationTime = periodTime/(ieRatioRef+1);
            inspirationTime = (double(60.0/double(FRv))*1000.0)/(ieRatioRef+1.0);
            valvesValueControl=uint32_t(setVOL/((inspirationTime/1000.0)*8.0*0.8));
        }
        if(ui->tabWidget_sel->currentIndex()==0){
            if(int(ieRatioRef) >= 3){
                if(FRv >=9 && FRv < 11){
                    if(setPIP <6)                   {valvesValueControl = 8; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 9; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 16; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 24; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 31; }
                    if(setPIP>=22)                  {valvesValueControl = 32; }
                }
                if(FRv >=11 && FRv < 13){
                    if(setPIP <6)                   {valvesValueControl = 9; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 10; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 18; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 25; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 31; }
                    if(setPIP>=22)                  {valvesValueControl = 32; }
                }
                if(FRv >=13 && FRv < 15){
                    if(setPIP <6)                   {valvesValueControl = 10; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 11; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 19; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 26; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 32; }
                    if(setPIP>=22)                  {valvesValueControl = 33; }
                }
                if(FRv >=15 && FRv < 17){
                    if(setPIP <6)                   {valvesValueControl = 12; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 13; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 21; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 28; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 33; }
                    if(setPIP>=22)                  {valvesValueControl = 34; }
                }
                if(FRv >=17 && FRv < 19){
                    if(setPIP <6)                   {valvesValueControl = 12; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 13; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 22; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 29; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 35; }
                    if(setPIP>=22)                  {valvesValueControl = 36; }
                }
                if(FRv >=19 && FRv < 21){
                    if(setPIP <6)                   {valvesValueControl = 12; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 13; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 23; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 30; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 36; }
                    if(setPIP>=22)                  {valvesValueControl = 37; }
                }
            }
            if(int(ieRatioRef) == 2){
                if(FRv >=9 && FRv < 11){
                    if(setPIP <6)                   {valvesValueControl = 7; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 8; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 14; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 21; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 27; }
                    if(setPIP>=22)                  {valvesValueControl = 28; }
                }
                if(FRv >=11 && FRv < 13){
                    if(setPIP <6)                   {valvesValueControl = 8; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 9; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 15; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 22; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 29; }
                    if(setPIP>=22)                  {valvesValueControl = 30; }
                }
                if(FRv >=13 && FRv < 15){
                    if(setPIP <6)                   {valvesValueControl = 8; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 9; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 16; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 23; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 30; }
                    if(setPIP>=22)                  {valvesValueControl = 31; }
                }
                if(FRv >=15 && FRv < 17){
                    if(setPIP <6)                   {valvesValueControl = 9; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 10; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 17; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 25; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 31; }
                    if(setPIP>=22)                  {valvesValueControl = 32; }
                }
                if(FRv >=17 && FRv < 19){
                    if(setPIP <6)                   {valvesValueControl = 9; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 10; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 18; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 26; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 31; }
                    if(setPIP>=22)                  {valvesValueControl = 32; }
                }
                if(FRv >=19 && FRv < 21){
                    if(setPIP <6)                   {valvesValueControl = 10; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 11; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 19; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 27; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 32; }
                    if(setPIP>=22)                  {valvesValueControl = 33; }
                }
            }
            if(int(ieRatioRef) == 1){
                if(FRv >=9 && FRv < 11){
                    if(setPIP <6)                   {valvesValueControl = 4; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 5; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 10; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 15; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 22; }
                    if(setPIP>=22)                  {valvesValueControl = 23; }
                }
                if(FRv >=11 && FRv < 13){
                    if(setPIP <6)                   {valvesValueControl = 5; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 6; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 11; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 16; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 23; }
                    if(setPIP>=22)                  {valvesValueControl = 24; }
                }
                if(FRv >=13 && FRv < 15){
                    if(setPIP <6)                   {valvesValueControl = 5; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 6; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 11; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 17; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 24; }
                    if(setPIP>=22)                  {valvesValueControl = 25; }
                }
                if(FRv >=15 && FRv < 17){
                    if(setPIP <6)                   {valvesValueControl = 6; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 7; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 12; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 19; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 24; }
                    if(setPIP>=22)                  {valvesValueControl = 25; }
                }
                if(FRv >=17 && FRv < 19){
                    if(setPIP <6)                   {valvesValueControl = 6; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 7; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 14; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 19; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 26; }
                    if(setPIP>=22)                  {valvesValueControl = 27; }
                }
                if(FRv >=19 && FRv < 21){
                    if(setPIP <6)                   {valvesValueControl = 6; }
                    if(setPIP>=6  && setPIP <10)    {valvesValueControl = 7; }
                    if(setPIP>=10 && setPIP <14)    {valvesValueControl = 14; }
                    if(setPIP>=14 && setPIP <18)    {valvesValueControl = 20; }
                    if(setPIP>=18 && setPIP <22)    {valvesValueControl = 26; }
                    if(setPIP>=22)                  {valvesValueControl = 27; }
                }
            }
        }
        qDebug() << "Nuevo valor de velocidad: " << valvesValueControl;
}


void MainWindow::on_pushButton_11_clicked()
{
    inspirationDetected = true;
}


void MainWindow::on_pushButton_alarmTest_clicked()
{
    activateAlarm(1);
}
