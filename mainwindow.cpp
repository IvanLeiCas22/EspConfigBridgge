#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <stdio.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort(this);
    settingPorts = new SettingsDialog(this);
    estadoSerial = new QLabel(this);
    estadoSerial->setText("Desconectado ......");
    ui->statusbar->addWidget(estadoSerial);
    ui->actionDisconnect->setEnabled(false);
    UdpSocket1 = new QUdpSocket(this);
    ui->pushButtonSend_2->setEnabled(false);
    estadoProtocoloUdp = START;
    estadoProtocolo = START;
    QTimer1 = new QTimer(this);
    QPaintBox1 = new QPaintBox(0, 0, ui->widgetRadar);

    connect(ui->pushButtonSend,&QPushButton::clicked,this,&MainWindow::sendDataSerial);
    connect(ui->pushButtonSend_2,&QPushButton::clicked,this,&MainWindow::sendDataUDP);
    connect(serial,&QSerialPort::readyRead,this,&MainWindow::dataRecived);
    connect(ui->actionScanPorts,&QAction::triggered,settingPorts,&SettingsDialog::show);
    connect(ui->actionConnect_Device, &QAction::triggered,this,&MainWindow::openSerialPorts);
    connect(ui->actionDisconnect,&QAction::triggered,this,&MainWindow::closeSerialPorts);
    connect(ui->actionExit,&QAction::triggered,this,&MainWindow::close);
    connect(UdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(QTimer1, &QTimer::timeout, this, &MainWindow::OnQTimer1);

    ui->comboBoxCom->addItem("ALIVE", 0xF0);
    ui->comboBoxCom->addItem("FIRMWARE", 0xF1);
    //ui->comboBoxCom->addItem("LEDS STATE", 0x10);
    ui->comboBoxCom->addItem("SWITCHS STATE", 0x12);
    ui->comboBoxCom->addItem("DISTANCE", 0xA3);
    ui->comboBoxCom->addItem("IRs STATE", 0xA0);
    ui->comboBoxCom->addItem("IRs BLACK CONFIG", 0xA6);
    ui->comboBoxCom->addItem("IRs WHITE CONFIG", 0xA7);
    ui->comboBoxCom->addItem("SERVO ANGLE", 0xA2);
    ui->comboBoxCom->addItem("SERVO CONFIG", 0xA5);
    ui->comboBoxCom->addItem("MOTOR POWER", 0xA1);
    ui->comboBoxCom->addItem("SPEED", 0xA4);

    //C:/Users/GAMING/Documents/Microcontroladores/Qt_nuevo/EspConfigBridgge/EspConfigBridgge/background3.jpg
    QPixmap bkgnd("C:/Users/ivanl/OneDrive/Documentos/Microcontroladores/DESKTOP_COMPUTER/EspConfigBridgge/EspConfigBridgge/background3.jpg");
    bkgnd = bkgnd.scaled(this->size(), Qt::IgnoreAspectRatio);
    QPalette palette;
    palette.setBrush(QPalette::Window, bkgnd);
    this->setPalette(palette);
}

MainWindow::~MainWindow()
{
    delete ui;
    //delete QPaintBox1;
}

void MainWindow::openSerialPorts()
{
    if(UdpSocket1->isOpen()){
        UdpSocket1->close();
        ui->pushButtonUdpOpen->setText("OPEN");
        QTimer1->stop();
        alive = false;
        reiniciarUI();
        return;
    }

    const SettingsDialog::Settings p = settingPorts->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    serial->open(QSerialPort::ReadWrite);
    if (serial->isOpen()){
        ui->actionDisconnect->setEnabled(true);
        estadoSerial->setText(tr("Connected to %1 : %2, %3, %4, %5, %6, %7")
                                .arg(p.name)
                                .arg(p.stringBaudRate)
                                .arg(p.stringDataBits)
                                .arg(p.stringParity)
                                .arg(p.stringStopBits)
                                .arg(p.stringFlowControl)
                                .arg(p.fabricante)
                               );
        QTimer1->start(10);
        inicializaciones();
        connectionType = SERIALCONNECTION;
    } else {
       QMessageBox::critical(this, tr("Error"), serial->errorString());
    }
}

void MainWindow::closeSerialPorts()
{
    ui->actionDisconnect->setEnabled(false);
    estadoSerial->setText("Desconectado ......");
    if (serial->isOpen()) {
        serial->close();
        QTimer1->stop();
        alive = false;
        reiniciarUI();
    }
}

void MainWindow::on_pushButtonUdpOpen_clicked()
{
    int Port;
    bool ok;

    ui->actionDisconnect->setEnabled(false);
    estadoSerial->setText("Desconectado ......");
    if (serial->isOpen()) {
        serial->close();
        QTimer1->stop();
        alive = false;
        reiniciarUI();
        return;
    }

    if(UdpSocket1->isOpen()){
        UdpSocket1->close();
        ui->pushButtonUdpOpen->setText("OPEN");
        QTimer1->stop();
        alive = false;
        reiniciarUI();
        return;
    }

    Port = ui->lineEditLocalPort->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        UdpSocket1->abort();
        UdpSocket1->bind(Port);
        UdpSocket1->open(QUdpSocket::ReadWrite);
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }
    ui->pushButtonUdpOpen->setText("CLOSE");
    ui->pushButtonSend_2->setEnabled(true);
    if (UdpSocket1->isOpen()){
        if(clientAddress.isNull())
            clientAddress.setAddress(ui->lineEditIP->text());
        if(puertoremoto==0)
            puertoremoto=ui->lineEditPort->text().toInt();
        UdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
        QTimer1->start(10);
        inicializaciones();
        connectionType = UDPCONNECTION;
    }
}

void MainWindow::dataRecived() //!< RECIBIR DATOS POR SERIAL
{
    unsigned char *incomingBuffer;
    int count;

    count = serial->bytesAvailable();
    if(count<=0)
        return;
    incomingBuffer = new unsigned char[count];
    serial->read((char *)incomingBuffer,count);
    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textBrowser->append("MBED-->SERIAL-->PC (" + str + ")");

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
            case START:
                if (incomingBuffer[i]=='U'){
                    estadoProtocolo=HEADER_1;
                    rxData.cheksum=0;
                }
                break;
            case HEADER_1:
                if (incomingBuffer[i]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    i--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (incomingBuffer[i]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    i--;
                   estadoProtocolo=START;
                }
                break;
            case HEADER_3:
                if (incomingBuffer[i]=='R')
                    estadoProtocolo=NBYTES;
                else{
                    i--;
                   estadoProtocolo=START;
                }
                break;
            case NBYTES:
                rxData.nBytes=incomingBuffer[i];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (incomingBuffer[i]==':'){
                    estadoProtocolo=PAYLOAD;
                    rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                    rxData.payLoad[0]=rxData.nBytes;
                    rxData.index=1;
                }
                else{
                    i--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (rxData.nBytes>1){
                    rxData.payLoad[rxData.index++]=incomingBuffer[i];
                    rxData.cheksum^=incomingBuffer[i];
                }
                rxData.nBytes--;
                if(rxData.nBytes==0){
                    estadoProtocolo=START;
                    if(rxData.cheksum==incomingBuffer[i]){
                        decodeData(&rxData.payLoad[0],SERIE);
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::OnUdpRxData(){ //!< RECIBIR DATOS POR UDP
    qint64          count=0;
    unsigned char   *incomingBuffer;

    while(UdpSocket1->hasPendingDatagrams()){
        count = UdpSocket1->pendingDatagramSize();
        incomingBuffer = new unsigned char[count];
        UdpSocket1->readDatagram( reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
    }

    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textBrowser->append("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    ui->textBrowser->append(" adr " + adress);
    ui->lineEditIP->setText(RemoteAddress.toString().right((RemoteAddress.toString().length())-7));
    ui->lineEditPort->setText(QString().number(RemotePort,10));

    for(int i=0;i<count; i++){
        switch (estadoProtocoloUdp) {
            case START:
                if (incomingBuffer[i]=='U'){
                    estadoProtocoloUdp=HEADER_1;
                    rxDataUdp.cheksum=0;
                }
                break;
            case HEADER_1:
                if (incomingBuffer[i]=='N')
                   estadoProtocoloUdp=HEADER_2;
                else{
                    i--;
                    estadoProtocoloUdp=START;
                }
                break;
            case HEADER_2:
                if (incomingBuffer[i]=='E')
                    estadoProtocoloUdp=HEADER_3;
                else{
                    i--;
                   estadoProtocoloUdp=START;
                }
                break;
            case HEADER_3:
                if (incomingBuffer[i]=='R')
                    estadoProtocoloUdp=NBYTES;
                else{
                    i--;
                   estadoProtocoloUdp=START;
                }
                break;
            case NBYTES:
                rxDataUdp.nBytes=incomingBuffer[i];
                estadoProtocoloUdp=TOKEN;
                break;
            case TOKEN:
                if (incomingBuffer[i]==':'){
                   estadoProtocoloUdp=PAYLOAD;
                   rxDataUdp.cheksum='U'^'N'^'E'^'R'^ rxDataUdp.nBytes^':';
                   rxDataUdp.payLoad[0]=rxDataUdp.nBytes;
                   rxDataUdp.index=1;
                }
                else{
                    i--;
                    estadoProtocoloUdp=START;
                }
                break;
            case PAYLOAD:
                if (rxDataUdp.nBytes>1){
                    rxDataUdp.payLoad[rxDataUdp.index++]=incomingBuffer[i];
                    rxDataUdp.cheksum^=incomingBuffer[i];
                }
                rxDataUdp.nBytes--;
                if(rxDataUdp.nBytes==0){
                    estadoProtocoloUdp=START;
                    if(rxDataUdp.cheksum==incomingBuffer[i]){
                        decodeData(&rxDataUdp.payLoad[0],UDP);
                    }else{
                        ui->textBrowser->append(" CHK DISTINTO!!!!! ");
                    }
                }
                break;
            default:
                estadoProtocoloUdp=START;
                break;
        }
    }
}

void MainWindow::decodeData(uint8_t *datosRx, uint8_t source)
{
    int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    uint8_t i=0;
    QString str, str2;
    _work myWorker;

    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    ui->textBrowser->append("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
        case LAST_ADC://     ANALOGSENSORS=0xA0,
            for (uint8_t i=0; i<16; i+=2) {
                myWorker.u8[0] = datosRx[i+2];
                myWorker.u8[1] = datosRx[i+3];
                irSensorsMeasure[i/2] = myWorker.u16[0];
            }

            for (uint8_t i=0; i<8; i++) {
                ui->txtBrowserCMD->append(QString("--> IR Nº%1 = %2").arg(i).arg(irSensorsMeasure[i]));
            }

            ui->ir_0->display(QString("%1").arg(irSensorsMeasure[0], 2, 10, QChar('0')));
            ui->ir_1->display(QString("%1").arg(irSensorsMeasure[1], 2, 10, QChar('0')));
            ui->ir_2->display(QString("%1").arg(irSensorsMeasure[2], 2, 10, QChar('0')));
            ui->ir_3->display(QString("%1").arg(irSensorsMeasure[3], 2, 10, QChar('0')));
            ui->ir_4->display(QString("%1").arg(irSensorsMeasure[4], 2, 10, QChar('0')));
            ui->ir_5->display(QString("%1").arg(irSensorsMeasure[5], 2, 10, QChar('0')));
            ui->ir_6->display(QString("%1").arg(irSensorsMeasure[6], 2, 10, QChar('0')));
            ui->ir_7->display(QString("%1").arg(irSensorsMeasure[7], 2, 10, QChar('0')));
            break;
        case SETMOTORTEST://     MOTORTEST=0xA1,
            if (datosRx[4] == ACK)
                ui->txtBrowserCMD->append("POWER HAS BEEN SET");
            break;
        case SETSERVOANGLE://     SERVOANGLE=0xA2,
            if (datosRx[4] == ACK) {
                ui->txtBrowserCMD->append("MOVING SERVO");
                servoIsMoving = true;
            }
            if (datosRx[4] == SERVOMOVESTOP) {
                ui->txtBrowserCMD->append("SERVO MOVED");
                servoIsMoving = false;
            }
            break;
        case GETSERVOANGLE://    GETSERVOANGLE=0xA8,
            servoAngleToShow = datosRx[4];
            ui->angleLCD->display(QString("%1").arg(servoAngleToShow, 2, 10, QChar('0')));
            break;
        case GETDISTANCE://     GETDISTANCE=0xA3,
            for (uint8_t apendice = 0; apendice < 4; apendice++) {
                myWorker.u8[apendice] = datosRx[apendice+4];
            }
            distancia = floor((myWorker.u32/58)+0.5);

            ui->txtBrowserCMD->append(QString("--> Distancia = %1").arg(distancia, 0, 10, QChar('0')).toUpper());

            ui->distanceLCD->display(QString("%1").arg(distancia, 2, 10, QChar('0')));
            break;
        case GETSPEED://     GETSPEED=0xA4,
            for (uint8_t i=0; i<4; i++) {
                myWorker.u8[i] = datosRx[i+4];
            }
            hSensors.rightSensorPulses = myWorker.u32;
            for (uint8_t i=0; i<4; i++) {
                myWorker.u8[i] = datosRx[i+8];
            }
            hSensors.leftSensorPulses = myWorker.u32;

            ui->txtBrowserCMD->append(QString("--> PulsosDerecha = %1").arg(hSensors.rightSensorPulses, 0, 10, QChar('0')).toUpper());
            ui->txtBrowserCMD->append(QString("--> PulsosIzquierda = %1").arg(hSensors.leftSensorPulses, 0, 10, QChar('0')).toUpper());

            ui->leftPulsesLCD->display(QString("%1").arg((hSensors.leftSensorPulses*3), 2, 10, QChar('0')));
            ui->rightPulsesLCD->display(QString("%1").arg((hSensors.rightSensorPulses*3), 2, 10, QChar('0')));
            break;
        case GETSWITCHES: //GETSWITCHES=0xA5
            switches[0].status = datosRx[4];
            str2 = "SW0: ";
            if(switches[0].status) {
                str2 = str2 + "UP";
                ui->EstadoBotonLabel->setText("UP");
            } else {
                str2 = str2 + "DOWN";
                ui->EstadoBotonLabel->setText("DOWN");
            }

            ui->txtBrowserCMD->append(str2);
            break;
        case GETALIVE://     GETALIVE=0xF0,
            if(datosRx[2]==ACK){
                if(source)
                    str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
                else{
                    contadorAlive++;
                     str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
                }
            }else{
                str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
            }
            ui->AliveLabel->setStyleSheet("border: 1px solid gray;border-color: black;border-radius: 2px;background-color: green;color: white;");
            ui->AliveLabel->setText("YES");
            alive = true;
            aliveTimeOut = 0;
            ui->textBrowser->append(str);
            ui->txtBrowserCMD->append(str);
            break;
        case GETFIRMWARE://     GETFIRMWARE=0xF1
            while (datosRx[i+4]) {
                firmwareCadena[i] = datosRx[i+4];
                i++;
            }
            ui->FirmwareLabel->setText(firmwareCadena);
            break;
        case SETLEDS:

            break;
        case STARTCONFIG:
            if(datosRx[4]==ACK){
                if(source)
                    str="DATOS CONFIG WIFI ACK VIA *SERIE* RECIBIDO!!!!";
                else
                    str="DATOS CONFIG WIFI ACK VIA *UDP* RECIBIDO!!!";
            }else{
                str="DATOS CONFIG WIFI NO ACK!!!";
            }
            ui->txtBrowserCMD->append(str);
            break;
        case SETSERVOLIMITS:
            myWorker.u8[0] = datosRx[4];
            myWorker.u8[1] = datosRx[5];
            servoConfig.maxMsServo = myWorker.u16[0];
            myWorker.u8[0] = datosRx[6];
            myWorker.u8[1] = datosRx[7];
            servoConfig.minMsServo = myWorker.u16[0];

            ui->txtBrowserCMD->append(QString("max: %1").arg(servoConfig.maxMsServo, 0, 10, QChar('0')));
            ui->txtBrowserCMD->append(QString("min: %1").arg(servoConfig.minMsServo, 0, 10, QChar('0')));

            ui->ServoMaxValueLCD->display(QString("%1").arg(servoConfig.maxMsServo, 0, 10, QChar('0')));
            ui->ServoMinValueLCD->display(QString("%1").arg(servoConfig.minMsServo, 0, 10, QChar('0')));
            break;
        case SETBLACKCOLORDETECTED:
            myWorker.u8[0] = datosRx[4];
            myWorker.u8[1] = datosRx[5];
            irConfig.blackLeft = myWorker.u16[0];
            myWorker.u8[0] = datosRx[6];
            myWorker.u8[1] = datosRx[7];
            irConfig.blackCenter = myWorker.u16[0];
            myWorker.u8[0] = datosRx[8];
            myWorker.u8[1] = datosRx[9];
            irConfig.blackRight = myWorker.u16[0];

            ui->txtBrowserCMD->append(QString("leftBlack: %1").arg(irConfig.blackLeft, 0, 10, QChar('0')));
            ui->txtBrowserCMD->append(QString("centerBlack: %1").arg(irConfig.blackCenter, 0, 10, QChar('0')));
            ui->txtBrowserCMD->append(QString("rightBlack: %1").arg(irConfig.blackRight, 0, 10, QChar('0')));

            ui->NegroConfigLCD->display(QString("%1").arg(irConfig.blackLeft, 0, 10, QChar('0')));
            break;
        case SETWHITECOLORDETECTED:
            myWorker.u8[0] = datosRx[4];
            myWorker.u8[1] = datosRx[5];
            irConfig.whiteLeft = myWorker.u16[0];
            myWorker.u8[0] = datosRx[6];
            myWorker.u8[1] = datosRx[7];
            irConfig.whiteCenter = myWorker.u16[0];
            myWorker.u8[0] = datosRx[8];
            myWorker.u8[1] = datosRx[9];
            irConfig.whiteRight = myWorker.u16[0];

            ui->txtBrowserCMD->append(QString("leftWhite: %1").arg(irConfig.whiteLeft, 0, 10, QChar('0')));
            ui->txtBrowserCMD->append(QString("centerWhite: %1").arg(irConfig.whiteCenter, 0, 10, QChar('0')));
            ui->txtBrowserCMD->append(QString("rightWhite: %1").arg(irConfig.whiteRight, 0, 10, QChar('0')));

            ui->BlancoConfigLCD->display(QString("%1").arg(irConfig.whiteLeft, 0, 10, QChar('0')));
            break;
        default:
            str = str + "Comando DESCONOCIDO!!!!";
            ui->txtBrowserCMD->append(str);
    }
}

void MainWindow::sendDataSerial()
{
    uint8_t cmdId;
    _work w;
    bool ok;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str="";

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    //dato[indice++]=0x0C;
    //dato[indice++]=0x00;

    if (isASelectedCmd) {
        cmdId = auxComand;
    } else
        cmdId = ui->comboBoxCom->currentData().toInt();

    switch (cmdId) {
        case SETMOTORTEST://MOTORTEST=0xA1,
            dato[indice++] = SETMOTORTEST;
            w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
            if(!ok)
                break;
            dato[indice++] = w.u8[0];
            dato[indice++] = w.u8[1];
            dato[indice++] = w.u8[2];
            dato[indice++] = w.u8[3];
            w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
            if(!ok)
                break;
            dato[indice++] = w.u8[0];
            dato[indice++] = w.u8[1];
            dato[indice++] = w.u8[2];
            dato[indice++] = w.u8[3];
            dato[NBYTES]= 0x0C;
            break;
        case SETSERVOANGLE://SERVOANGLE=0xA2,
            dato[indice++] = SETSERVOANGLE;
            if (isASelectedCmd) {
                w.i32 = servoAngle;
            } else {
                w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, 0, 180, 1, &ok);
                if(!ok)
                    break;
            }
            dato[indice++] = w.i8[0];
            dato[NBYTES]= 0x05;
            break;
        case GETALIVE:
        case GETDISTANCE://GETDISTANCE=0xA3
        case GETSPEED://GETSPEED=0xA4
        case GETSWITCHES://GETSWITCHES=0xA5
        case GETFIRMWARE:// GETFIRMWARE=0xF1
        case LAST_ADC://ANALOGSENSORS=0xA0
        case GETSERVOANGLE://GETSERVOANGLE=0xA8
        case SETLEDS:
            dato[indice++] = cmdId;
            dato[NBYTES] = 0x02;
            break;
        case SETSERVOLIMITS:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                servoConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = servoConfig.modo;
            if (servoConfig.modo) {
                servoConfig.maxMsServo = QInputDialog::getInt(this, "MAX MS", "Elegir ms maximos del servo", 2000, 2000, 2450, 1, &ok);
                if (ok) {
                    w.u16[0] = servoConfig.maxMsServo;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                servoConfig.minMsServo = QInputDialog::getInt(this, "MIN MS", "Elegir ms minimos del servo", 1000, 480, 1000, 1, &ok);
                if (ok) {
                    w.u16[0] = servoConfig.minMsServo;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x09;
            }
            else
                dato[NBYTES] = 0x05;
            break;
        case SETBLACKCOLORDETECTED:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                irConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = irConfig.modo;
            if (irConfig.modo) {
                irConfig.blackLeft = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro izquierdo:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackLeft;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.blackCenter = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro centro:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackCenter;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.blackRight = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro derecho:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackRight;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x0B;
            } else
                dato[NBYTES] = 0x05;
            break;
        case SETWHITECOLORDETECTED:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                irConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = irConfig.modo;
            if (irConfig.modo) {
                irConfig.whiteLeft = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco izquierdo:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteLeft;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.whiteCenter = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco centro:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteCenter;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.whiteRight = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco derecho:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteRight;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x0B;
            } else
                dato[NBYTES] = 0x05;
            break;
        default:
            return;
    }

    isASelectedCmd = false;

    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;

    if(serial->isWritable()){
        serial->write(reinterpret_cast<char *>(dato),dato[NBYTES]+PAYLOAD);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }

    uint16_t valor=dato[NBYTES]+PAYLOAD;
    ui->textBrowser->append("INDICE ** " +QString().number(indice,10) + " **" );
    ui->textBrowser->append("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    ui->textBrowser->append("CHECKSUM ** " +QString().number(chk,16) + " **" );
    ui->textBrowser->append("PC--SERIAL-->MBED ( " + str + " )");

}

void MainWindow::sendDataUDP()
{
    uint8_t cmdId;
    _work w;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str;
    int puerto=0;
    bool ok;

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    //dato[indice++]=0x0C;
   // dato[indice++]=0x00;

    if (isASelectedCmd) {
        cmdId = auxComand;
    } else
        cmdId = ui->comboBoxCom->currentData().toInt();

    switch (cmdId) {
        case SETMOTORTEST://MOTORTEST=0xA1,
            dato[indice++] = SETMOTORTEST;
            w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
            if(!ok)
                break;
            dato[indice++] = w.u8[0];
            dato[indice++] = w.u8[1];
            dato[indice++] = w.u8[2];
            dato[indice++] = w.u8[3];
            w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
            if(!ok)
                break;
            dato[indice++] = w.u8[0];
            dato[indice++] = w.u8[1];
            dato[indice++] = w.u8[2];
            dato[indice++] = w.u8[3];
            dato[NBYTES]= 0x0C;
            break;
        case SETSERVOANGLE://SERVOANGLE=0xA2,
            dato[indice++] = SETSERVOANGLE;
            if (isASelectedCmd) {
                w.i32 = servoAngle;
            } else {
                w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, 0, 180, 1, &ok);
                if(!ok)
                    break;
            }
            dato[indice++] = w.i8[0];
            dato[NBYTES]= 0x05;
            break;
        case GETALIVE:
        case GETDISTANCE://GETDISTANCE=0xA3,
        case GETSPEED://GETSPEED=0xA4,
        case GETSWITCHES://GETSWITCHES=0xA5
        case GETFIRMWARE:// GETFIRMWARE=0xF1
        case LAST_ADC://ANALOGSENSORS=0xA0,
        case GETSERVOANGLE://GETSERVOANGLE=0xA8
        case SETLEDS:
            dato[indice++] = cmdId;
            dato[NBYTES] = 0x02;
            break;
        case SETSERVOLIMITS:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                servoConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = servoConfig.modo;
            if (servoConfig.modo) {
                servoConfig.maxMsServo = QInputDialog::getInt(this, "MAX MS", "Elegir ms maximos del servo", 2000, 2000, 2450, 1, &ok);
                if (ok) {
                    w.u16[0] = servoConfig.maxMsServo;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                servoConfig.minMsServo = QInputDialog::getInt(this, "MIN MS", "Elegir ms minimos del servo", 1000, 480, 1000, 1, &ok);
                if (ok) {
                    w.u16[0] = servoConfig.minMsServo;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x09;
            }
            else
                dato[NBYTES] = 0x05;
            break;
        case SETBLACKCOLORDETECTED:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                irConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = irConfig.modo;
            if (irConfig.modo) {
                irConfig.blackLeft = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro izquierdo:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackLeft;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.blackCenter = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro centro:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackCenter;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.blackRight = QInputDialog::getInt(this, "BLACK COLOR", "Deteccion negro derecho:", 6000, 2000, 60000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.blackRight;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x0B;
            } else
                dato[NBYTES] = 0x05;
            break;
        case SETWHITECOLORDETECTED:
            dato[indice++] = cmdId;
            if (!isASelectedCmd) {
                irConfig.modo = QInputDialog::getInt(this, "ELEGIR MODO", " 0->LEER   1->CONFIGURAR ", 0, 0, 1, 1, &ok);
                if (!ok)
                    break;
            }
            dato[indice++] = irConfig.modo;
            if (irConfig.modo) {
                irConfig.whiteLeft = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco izquierdo:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteLeft;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.whiteCenter = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco centro:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteCenter;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                irConfig.whiteRight = QInputDialog::getInt(this, "WHITE COLOR", "Deteccion blanco derecho:", 3500, 1000, 20000, 1, &ok);
                if (ok) {
                    w.u16[0] = irConfig.whiteRight;
                    dato[indice++] = w.u8[0];
                    dato[indice++] = w.u8[1];
                } else
                    return;
                dato[NBYTES] = 0x0B;
            } else
                dato[NBYTES] = 0x05;
            break;
        default:
            return;
    }

    isASelectedCmd = false;

    puerto=ui->lineEditPort->text().toInt();
    puertoremoto=puerto;
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;
    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEditIP->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(UdpSocket1->isOpen()){
        UdpSocket1->writeDatagram(reinterpret_cast<const char *>(dato), (dato[4]+7), clientAddress, puertoremoto);
    }
    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);
    ui->textBrowser->append("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::OnQTimer1()
{
    if (timeLecturaSensoresTask10ms >= 50) {
        lecturaSensores();
        timeLecturaSensoresTask10ms = 0;
    } else
        timeLecturaSensoresTask10ms++;

    if (timeClearLog10ms >= 1000) {
        ui->txtBrowserCMD->clear();
        ui->textBrowser->clear();
        timeClearLog10ms = 0;
    } else
        timeClearLog10ms++;

    if (aliveTimeOut >= 1000) {
        if (connectionType == 1) {
            closeSerialPorts();
        } else {
            if (connectionType == 2) {
                on_pushButtonUdpOpen_clicked();
            }
        }
        alive = false;
        aliveTimeOut = 0;
    } else
        aliveTimeOut++;

    if (radarDrawing)
            RadarRun();
}

void MainWindow::inicializaciones()
{
    distancia = 0;
    servoAngleToShow = 90;
    //irSensors.RightSensor = 0;
    //irSensors.CenterSensor = 0;
    //irSensors.LeftSensor = 0;
    hSensors.leftSensorPulses = 0;
    hSensors.rightSensorPulses = 0;
    switches[0].status = true;
    aliveTimeOut = 0;
    recibirConfig = true;
    DibujarFondoRadar();
}

void MainWindow::lecturaSensores()
{
    auxComand = GETDISTANCE;
    isASelectedCmd = true;
    sendDataSerial();
    isASelectedCmd = true;
    sendDataUDP();

    auxComand = GETSERVOANGLE;
    isASelectedCmd = true;
    sendDataSerial();
    isASelectedCmd = true;
    sendDataUDP();

    auxComand = LAST_ADC;
    isASelectedCmd = true;
    sendDataSerial();
    isASelectedCmd = true;
    sendDataUDP();

    auxComand = GETSPEED;
    isASelectedCmd = true;
    sendDataSerial();
    isASelectedCmd = true;
    sendDataUDP();

    auxComand = GETSWITCHES;
    isASelectedCmd = true;
    sendDataSerial();
    isASelectedCmd = true;
    sendDataUDP();

    if (alive && recibirConfig) {
        recibirConfig = false;

//        auxComand = GETFIRMWARE;
//        isASelectedCmd = true;
//        sendDataUDP();
//        isASelectedCmd = true;
//        sendDataSerial();

        ui->FirmwareLabel->setText("20221211-v1.0");

        auxComand = SETSERVOLIMITS;
        servoConfig.modo = 0;
        isASelectedCmd = true;
        sendDataUDP();
        isASelectedCmd = true;
        sendDataSerial();

        auxComand = SETBLACKCOLORDETECTED;
        irConfig.modo = 0;
        isASelectedCmd = true;
        sendDataSerial();
        isASelectedCmd = true;
        sendDataUDP();

        auxComand = SETWHITECOLORDETECTED;
        irConfig.modo = 0;
        isASelectedCmd = true;
        sendDataSerial();
        isASelectedCmd = true;
        sendDataUDP();
    }

    if (connectionType == 1) {
        auxComand = GETALIVE;
        isASelectedCmd = true;
        sendDataSerial();
    }

    ui->rangeLCD->display(QString("%1").arg((100*MINRANGOSLIDER)/ui->scaleSlider->value(), 2, 10, QChar('0')));
}

void MainWindow::reiniciarUI()
{
    ui->distanceLCD->display(0);
    ui->angleLCD->display(0);
    //ui->ir_THREE->display(0);
    //ui->ir_TWO->display(0);
    //ui->ir_ONE->display(0);
    ui->leftPulsesLCD->display(0);
    ui->rightPulsesLCD->display(0);
    ui->BlancoConfigLCD->display(0);
    ui->NegroConfigLCD->display(0);
    ui->ServoMaxValueLCD->display(0);
    ui->ServoMinValueLCD->display(0);
    ui->EstadoBotonLabel->setText("UP");
    ui->txtBrowserCMD->clear();
    ui->textBrowser->clear();
    ui->AliveLabel->setStyleSheet("border: 1px solid gray;border-color: black;border-radius: 2px;background-color: red;color: white;");
    ui->AliveLabel->setText("NO");
//    ui->lineEditIP->clear();
//    ui->lineEditPort->clear();
    ui->FirmwareLabel->clear();
}

void MainWindow::on_radarButton_clicked()
{
    if (radarDrawing) {
        auxComand = SETSERVOANGLE;
        servoAngle = 90;
        isASelectedCmd = true;
        sendDataSerial();
        isASelectedCmd = true;
        sendDataUDP();
        radarDrawing = false;
        ui->radarButton->setText("RUN RADAR");
        ui->scaleSlider->setEnabled(true);
    } else {
        radarDrawing = true;
        ui->radarButton->setText("STOP RADAR");
        DibujarFondoRadar();
        servoAngle = 0;
        radarStatus = MOVIENDO_SERVO;
        ui->scaleSlider->setEnabled(false);
    }
}

void MainWindow::DibujarFondoRadar() {
    QPen pen;
    QRadialGradient gradient(0, 0, 300);
    gradient.setColorAt(0, QColor(0, 20, 0, 255));
    gradient.setColorAt(1, QColor(0, 50, 0, 255));
    QBrush brush(gradient);
    QPainter painter(QPaintBox1->getCanvas());
    uint8_t painterAngle = 10;

    QPaintBox1->getCanvas()->fill(QColor(238,215,243,255));

    pen.setWidth(1);
    pen.setColor(QColorConstants::Svg::darkgreen);
    brush.setStyle(Qt::BrushStyle::RadialGradientPattern);
    painter.setPen(pen);
    painter.setBrush(brush);
    painter.translate(ui->widgetRadar->width()/2, ui->widgetRadar->height());
    painter.drawEllipse(QPointF(0,0), ui->widgetRadar->width()/2, ui->widgetRadar->height());
    painter.setBrush(Qt::NoBrush);
    for (uint8_t i=0; i<10; i++) {
        painter.drawEllipse(QPointF(0,0), (ui->widgetRadar->width()/2) - (i*30+30), (ui->widgetRadar->height()) - (i*30+30));
    }
    for (uint8_t i=0; i<18; i++) {
        painter.drawLine(0, 0, cos(painterAngle*M_PI/180)*ui->widgetRadar->width()/2, sin(painterAngle*M_PI/180)*(-(ui->widgetRadar->height())));
        painterAngle += 10;
//        painter.drawLine(0, 0, (ui->widgetRadar->width()/2), 0);
//        painter.rotate(-10);
    }
    QPaintBox1->update();
}

void MainWindow::RadarRun()
{
    switch (radarStatus)
    {
        case MOVIENDO_SERVO:
            auxComand = SETSERVOANGLE;
            isASelectedCmd = true;
            sendDataSerial();
            isASelectedCmd = true;
            sendDataUDP();
            radarStatus = MIDIENDO;
        break;
        case MIDIENDO:
            if (!servoIsMoving) {
                auxComand = GETDISTANCE;
                isASelectedCmd = true;
                sendDataSerial();
                isASelectedCmd = true;
                sendDataUDP();
                radarStatus = DIBUJANDO;
            }
        break;
        case DIBUJANDO:
            if (distancia <= 100) {
                DibujarDeteccion();
                servoAngle += 1;
                radarStatus = MOVIENDO_SERVO;
            } else {
                servoAngle += 1;
                radarStatus = MOVIENDO_SERVO;
            }
            if (servoAngle > 180) {
                radarStatus = FINALIZANDO;
            }
        break;
        case FINALIZANDO:
            auxComand = SETSERVOANGLE;
            servoAngle = 90;
            isASelectedCmd = true;
            sendDataSerial();
            isASelectedCmd = true;
            sendDataUDP();
            radarDrawing = false;
            ui->radarButton->setText("RUN RADAR");
            ui->scaleSlider->setEnabled(true);
        break;
    }
}

void MainWindow::DibujarDeteccion() {
    QPen pen;
    QBrush brush;
    QPainter painter(QPaintBox1->getCanvas());
    QColor colorVerde(0, 255, 0, 80);
    QColor colorRojo(255, 0, 0, 80);
    int16_t X;
    int16_t Y;

    painter.setRenderHint(QPainter::Antialiasing);

    if (distancia*(ui->scaleSlider->value())/MINRANGOSLIDER <= 100) {
        X = (floor(cos(servoAngle*M_PI/180)*(distancia)*(ui->scaleSlider->value())/MINRANGOSLIDER + 0.5))*3.6;
        Y = (floor(sin(servoAngle*M_PI/180)*(distancia)*(ui->scaleSlider->value())/MINRANGOSLIDER + 0.5))*(-3.6);
        painter.translate(ui->widgetRadar->width()/2, ui->widgetRadar->height());
        pen.setWidth(1);
        pen.setColor(colorVerde);
        painter.setPen(pen);
        painter.drawLine(0, 0, X, Y);
        pen.setColor(colorRojo);
        painter.setPen(pen);
        painter.drawLine(X, Y, cos(servoAngle*M_PI/180)*ui->widgetRadar->width()/2, sin(servoAngle*M_PI/180)*(-(ui->widgetRadar->height())));

        pen.setColor(QColorConstants::Svg::black);
        painter.setPen(pen);
        brush.setColor(QColorConstants::Svg::darkgreen);
        brush.setStyle(Qt::SolidPattern);
        painter.setBrush(brush);
        painter.drawEllipse(QPoint(X, Y), 4, 4);
    }
    QPaintBox1->update();
}
