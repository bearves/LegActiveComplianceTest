#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_remoteAddr = QHostAddress(REMOTE_IP_STRING);
    m_remotePort = REMOTE_PORT;
    m_sendCount = 0;
    m_recvCount = 0;
    m_isConnected = false;

    m_timer = new QTimer(this);
    m_timer->setInterval((int)(1000.0 / CONNECTION_FREQ));

    ui->setupUi(this);

    ui->pushButtonConnect->connect(
                ui->pushButtonConnect,
                SIGNAL(clicked()),
                this,
                SLOT(OnPushbuttonConnectClicked()));

    ui->lineEditCmd->connect(
                ui->lineEditCmd,
                SIGNAL(returnPressed()),
                this,
                SLOT(OnCommandLineReturned()));

    m_tcpSocket = new QTcpSocket();
    m_tcpSocket->connect(
                m_tcpSocket,
                SIGNAL(readyRead()),
                this,
                SLOT(OnDatagramReceived()));

    m_timer->connect(
                m_timer,
                SIGNAL(timeout()),
                this,
                SLOT(OnTimerTick()));

    ui->lineEditIP->setText(REMOTE_IP_STRING);

    QString txt;
    ui->lineEditPort->setText(txt.sprintf("%d", REMOTE_PORT));
    ui->lineEditCmd->setFocus();
    
}

MainWindow::~MainWindow()
{
    m_timer->stop();
    if (m_tcpSocket->state() == QAbstractSocket::ConnectedState)
        m_tcpSocket->disconnectFromHost();
    m_tcpSocket->close();
    delete ui;
    delete m_tcpSocket;
    delete m_timer;
}

void MainWindow::OnCommandLineReturned()
{
    QString txt = ui->lineEditCmd->text();
    ui->plainTextEditCmdLog->appendPlainText(txt);
    ui->lineEditCmd->clear();
    this->ProcessCommand(txt);
}

void MainWindow::OnPushbuttonConnectClicked()
{
    std::cout << m_isConnected << std::endl;
    if (m_isConnected == false){
        QHostAddress tmpAddr;
        int tmpPort;
        bool flag = false;

        if (! tmpAddr.setAddress(ui->lineEditIP->text())){
            ui->plainTextEditMsgLog->appendPlainText("Invalid IP address");
            return;
        }

        tmpPort = ui->lineEditPort->text().toInt(&flag);
        if (! flag || tmpPort <= 0 || tmpPort > 32767){
            ui->plainTextEditMsgLog->appendPlainText("Invalid port");
            return;
        }
        m_remoteAddr = tmpAddr;
        m_remotePort = tmpPort;

        //std::cout << m_remoteAddr << std::endl;
        std::cout << m_remotePort << std::endl;

        m_timer->start();
        m_tcpSocket->connectToHost(m_remoteAddr, m_remotePort);

        m_isConnected = true;
        ui->pushButtonConnect->setEnabled(false);
        ui->lineEditIP->setEnabled(false);
        ui->lineEditPort->setEnabled(false);
        ui->lineEditCmd->setFocus();
    }
}

void MainWindow::OnDatagramReceived()
{
    int byteRead;
    head_t msgHead;
    QString recvMsg;
    while(!m_tcpSocket->atEnd())
    {
        // read msg header
        byteRead = m_tcpSocket->read(msgHead.header, MSG_HEADER_LENGTH);
        m_robotMsgReceive.SetLength(msgHead.dataLength);
        m_robotMsgReceive.SetMsgID(msgHead.msgID);
        m_recvCount++;
        // read the msg body
        if (msgHead.dataLength > 0)
            byteRead = m_tcpSocket->read(m_robotMsgReceive.GetDataAddress(), msgHead.dataLength);

        // determine what to do with the msg
        switch (m_robotMsgReceive.GetMsgID()){
        case RMID_MESSAGE_ACK:
            recvMsg.sprintf("MSG[R]: ID=%d Len=%d ACK", m_robotMsgReceive.GetMsgID(), byteRead);
            ui->plainTextEditMsgLog->appendPlainText(recvMsg);
            break;

        case RMID_MESSAGE_DATA_REPORT:
            m_robotMsgReceive.Paste((void *)&m_machineData, m_robotMsgReceive.GetLength());
            this->DisplayDeviceData(m_machineData);
            break;
        }
    }
}

void MainWindow::DisplayDeviceData(Aris::RT_CONTROL::CMachineData &machineData)
{
    QString txt;
    ui->textBrowserDevStatus->setTextColor(QColor("black"));
    ui->textBrowserDevStatus->setFontUnderline(true);
    ui->textBrowserDevStatus->setFontWeight(QFont::Bold);
    txt.sprintf("No.     SW    OM      TR            VEL            POS   READY");
    ui->textBrowserDevStatus->setText(txt);
    ui->textBrowserDevStatus->setTextColor(QColor("blue"));
    ui->textBrowserDevStatus->setFontUnderline(false);
    ui->textBrowserDevStatus->setFontWeight(QFont::Normal);
    //for(int i = 0; i < ACTUAL_MOTOR_NUMBER; i++)
    for(int i = 0; i < 12; i++)
    {
        txt.sprintf("%2d%8s%6d%8d%15d%15d%8s",
                    i,
                    MOTOR_STATE_DISPLAY_STRING[machineData.motorsStates[i]],
                    machineData.motorsModesDisplay[i],
                    machineData.feedbackData[i].Torque,
                    machineData.feedbackData[i].Velocity,
                    machineData.feedbackData[i].Position,
                    machineData.isMotorHomed[i] == true? "TRUE":"FALSE");
        ui->textBrowserDevStatus->append(txt);
    }
    ui->textBrowserDevStatus->setTextColor(QColor("black"));
    ui->textBrowserDevStatus->setFontUnderline(true);
    ui->textBrowserDevStatus->setFontWeight(QFont::Bold);
    txt.sprintf("No.         Value1      Value2      Value3      Value4        ");
    ui->textBrowserDevStatus->append(txt);
    ui->textBrowserDevStatus->setTextColor(QColor("blue"));
    ui->textBrowserDevStatus->setFontUnderline(false);
    ui->textBrowserDevStatus->setFontWeight(QFont::Normal);
    for(int i = 0; i < ACTUAL_ATI_FORCE_SENSOR_NUM ; i++)
    {
        txt.sprintf("%3d%9.3f%9.3f%9.3f%9.3f%9.3f%9.3f",
                    i,
                machineData.forceData[i].forceValues[0]/1000.0,
                machineData.forceData[i].forceValues[1]/1000.0,
                machineData.forceData[i].forceValues[2]/1000.0,
                machineData.forceData[i].forceValues[3]/1.0,
                machineData.forceData[i].forceValues[4]/1.0,
                machineData.forceData[i].forceValues[5]/1.0);
        ui->textBrowserDevStatus->append(txt);
    }
    txt.sprintf("IMU:       R       P       Y      WX      WY      WZ          ");
    ui->textBrowserDevStatus->append(txt);
    ui->textBrowserDevStatus->setTextColor(QColor("blue"));
    ui->textBrowserDevStatus->setFontUnderline(false);
    ui->textBrowserDevStatus->setFontWeight(QFont::Normal);
    //txt.sprintf("%12.3lf%8.3lf%8.3lf%8.3lf%8.3lf%8.3lf",
    //       deviceData.m_imuData.EulerAngle.EulerStruct.roll,
    //       deviceData.m_imuData.EulerAngle.EulerStruct.pitch,
    //       deviceData.m_imuData.EulerAngle.EulerStruct.yaw,
    //       deviceData.m_imuData.AngularVelocity[0],
    //       deviceData.m_imuData.AngularVelocity[1],
    //       deviceData.m_imuData.AngularVelocity[2]
    //       );

}

void MainWindow::ProcessCommand(QString cmd)
{
    QString processMsg;
    int byteSent;
    bool hasMessageToSend = false;

    cmd = cmd.simplified();

    if(cmd.length() == 2){
        if (cmd[0] == 'z' && cmd[1] == 'o')
        {
            m_robotMsgToSend.SetMsgID(RMID_CLEAR_FORCE);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'p' && cmd[1] == 'd')
        {
            m_robotMsgToSend.SetMsgID(RMID_POWEROFF);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 's' && cmd[1] == 'p')
        {
            m_robotMsgToSend.SetMsgID(RMID_STOP);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'e' && cmd[1] == 'n')
        {
            m_robotMsgToSend.SetMsgID(RMID_ENABLE);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'h' && cmd[1] == '1')
        {
            m_robotMsgToSend.SetMsgID(RMID_GOHOME_1);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'h' && cmd[1] == '2')
        {
            m_robotMsgToSend.SetMsgID(RMID_GOHOME_2);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'p' && cmd[1] == 's')
        {
            m_robotMsgToSend.SetMsgID(RMID_RUNNING);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'p' && cmd[1] == '1')
        {
            m_robotMsgToSend.SetMsgID(RMID_HOME2START_1);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'p' && cmd[1] == '2')
        {
            m_robotMsgToSend.SetMsgID(RMID_HOME2START_2);
            hasMessageToSend = true;
        }
        //else if (cmd[0] == 'p' && cmd[1] == 'i')
        //{
        //    m_robotMsgToSend.BuildMsg(RMID_SM_GO_INIT_POS, (const char *)&emptyPackage, sizeof(emptyPackage));
        //    hasMessageToSend = true;
        //}
        else if (cmd[0] == 'b' && cmd[1] == 'i')
        {
            m_robotMsgToSend.SetMsgID(RMID_ONLINEGAIT);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'b' && cmd[1] == 'g')
        {
            m_robotMsgToSend.SetMsgID(RMID_ONLINEBEGIN);
            hasMessageToSend = true;
        }
        else if (cmd[0] == 'b' && cmd[1] == 't')
        {
            m_robotMsgToSend.SetMsgID(RMID_ONLINEEND);
            hasMessageToSend = true;
        }
    }
    if (hasMessageToSend){
        byteSent = m_tcpSocket->write(m_robotMsgToSend.GetHeaderAddress(), m_robotMsgToSend.GetLength() + MSG_HEADER_LENGTH);
        processMsg.sprintf("MSG[T]: ID=%d Len=%d", m_robotMsgToSend.GetMsgID(), byteSent);
        ui->plainTextEditMsgLog->appendPlainText(processMsg);
        m_sendCount++;
    }
}

void MainWindow::OnTimerTick()
{
    m_tickCount++;
    if(m_tickCount % 4 == 0){
        DisplayDeviceData(m_machineData);
        std::cout << "SOCK: " << m_tcpSocket->state() << std::endl;
    }
}
