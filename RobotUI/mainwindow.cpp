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

    memset((void *)&m_machineData, 0, sizeof(m_machineData));

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

    m_tcpSocket->connect(
                m_tcpSocket,
                SIGNAL(disconnected()),
                this,
                SLOT(OnSocketDisconnected()));

    ui->lineEditIP->setText(REMOTE_IP_STRING);

    QString txt;
    ui->lineEditPort->setText(txt.sprintf("%d", REMOTE_PORT));
    ui->lineEditCmd->setFocus();

    InitializePloting();
}


void MainWindow::InitializePloting()
{
    m_plotWidgets.clear();
    m_plotWidgets.push_back(ui->widget_3);
    m_plotWidgets.push_back(ui->widget_2);
    m_plotWidgets.push_back(ui->widget);
    
    m_GetSourceData = RetriveMotorData;
    m_sourceIndex = 0;

    for( int i = 0; i < 3; i++)
    {
        plotData.push_back(std::shared_ptr<QVector<double>>(new QVector<double>()));
    }
    
    for( int i = 0; i < 101; i++ )
    {
        timeLine.push_back(i * 0.2);
        for ( auto datIter = plotData.begin(); datIter < plotData.end(); datIter++)
        {
            (*datIter)->push_back(0);
        }
    }
    
    for (int i = 0; i < 3; i++)
    {
        m_plotWidgets[i]->addGraph();
        m_plotWidgets[i]->graph(0)->setData(timeLine, *(plotData[i]));
        m_plotWidgets[i]->rescaleAxes();
        m_plotWidgets[i]->replot();
    }
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
    m_timer->start();
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

            // There are two force sensors have a different scale of data
            for (int i = 0; i < 6; ++i)
            {
                m_machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[0]].forceValues[i] /= 1000.0;
                m_machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[1]].forceValues[i] /= 1000.0;
            }
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
    for(int i = 0; i < ACTUAL_MOTOR_NUMBER; i++)
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
    txt.sprintf("No.   Value1   Value2   Value3   Value4   Value5   Value6 ");
    ui->textBrowserDevStatus->append(txt);
    ui->textBrowserDevStatus->setTextColor(QColor("blue"));
    ui->textBrowserDevStatus->setFontUnderline(false);
    ui->textBrowserDevStatus->setFontWeight(QFont::Normal);
    for(int i = 0; i < ACTUAL_ATI_FORCE_SENSOR_NUM ; i++)
    {
        txt.sprintf("%3d%9.2f%9.2f%9.2f%9.2f%9.2f%9.2f",
                    i,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[0]/1000.0,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[1]/1000.0,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[2]/1000.0,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[3]/1.0,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[4]/1.0,
                machineData.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[5]/1.0);
        ui->textBrowserDevStatus->append(txt);
    }

    ui->textBrowserDevStatus->setTextColor(QColor("black"));
    ui->textBrowserDevStatus->setFontUnderline(true);
    ui->textBrowserDevStatus->setFontWeight(QFont::Bold);
    txt.sprintf("IMU:       R       P       Y      WX      WY      WZ          ");
    ui->textBrowserDevStatus->append(txt);
    ui->textBrowserDevStatus->setTextColor(QColor("blue"));
    ui->textBrowserDevStatus->setFontUnderline(false);
    ui->textBrowserDevStatus->setFontWeight(QFont::Normal);
    txt.sprintf("%12.3lf%8.3lf%8.3lf%8.3lf%8.3lf%8.3lf",
           machineData.imuData.EulerAngle[0],
           machineData.imuData.EulerAngle[1],
           machineData.imuData.EulerAngle[2],
           machineData.imuData.AngularVel[0],
           machineData.imuData.AngularVel[1],
           machineData.imuData.AngularVel[2]
           );
    ui->textBrowserDevStatus->append(txt);
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
        else if (cmd[0] == 'b' && cmd[1] == 'c')
        {
            m_robotMsgToSend.SetMsgID(RMID_ONLINEGAIT_2);
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
    if (cmd.left(4) == "show")
    {
        ChangePlottingDataSource(cmd);
    }

    if (cmd.left(4) == "move")
    {
        if (cmd.mid(5, 7) == "default")
        {
            // use default param and start
            m_robotMsgToSend.SetMsgID(RMID_SET_PARA_CXB);
            RobotHighLevelControl::ParamCXB param;

            param.gaitCommand      = RobotHighLevelControl::GAIT_SUB_COMMAND::GSC_START;
            param.totalPeriodCount = 10;
            param.stepLength       = 0;
            param.Lside            = 0;
            param.rotationAngle    = 0;
            param.duty             = 0.52;
            param.stepHeight       = 60; //positive value
            param.T                = 1.0;
            param.standHeight      = 700;
            param.tdDeltaMidLeg    = 3;
            param.tdDeltaSideLeg   = 3;

            m_robotMsgToSend.SetLength(sizeof(param));
            m_robotMsgToSend.Copy(&param, sizeof(param));
            hasMessageToSend = true;
        }
        else 
        {
            // get new param
        }
    }

    
    if (hasMessageToSend){
        if (m_tcpSocket->state() == QAbstractSocket::ConnectedState){
            byteSent = m_tcpSocket->write(m_robotMsgToSend.GetHeaderAddress(), m_robotMsgToSend.GetLength() + MSG_HEADER_LENGTH);
            processMsg.sprintf("MSG[T]: ID=%d Len=%d", m_robotMsgToSend.GetMsgID(), byteSent);
            ui->plainTextEditMsgLog->appendPlainText(processMsg);
            m_sendCount++;
        }
        else{
            processMsg.sprintf("Cannot send msg as the connection is broken");
            ui->plainTextEditMsgLog->appendPlainText(processMsg);
        }
    }
}

void MainWindow::OnTimerTick()
{
    m_tickCount++;
    QVector<double> retrivedData(3);
    if(m_tickCount % 1 == 0){
        DisplayDeviceData(m_machineData);

        m_GetSourceData(m_machineData, retrivedData, m_sourceIndex);

        for(int i = 0; i < 3; i++){
            plotData[i]->pop_front();
            plotData[i]->push_back(retrivedData[i]);
            
            m_plotWidgets[i]->graph(0)->setData(timeLine, *plotData[i]);
            m_plotWidgets[i]->rescaleAxes();
            m_plotWidgets[i]->replot();
        }

        // check the connection status 
        if(m_tickCount % 40 == 0){
            if (m_tcpSocket->state() != QAbstractSocket::ConnectedState && m_isConnected == true)
                OnSocketDisconnected();
        }
    }
}

void MainWindow::OnSocketDisconnected()
{
    m_isConnected = false;
    QString processMsg = "Connection is down, please reconnect again";

    ui->pushButtonConnect->setEnabled(true);
    ui->lineEditIP->setEnabled(true);
    ui->lineEditPort->setEnabled(true);
    ui->pushButtonConnect->setFocus();
    ui->plainTextEditMsgLog->appendPlainText(processMsg);
}

void MainWindow::ChangePlottingDataSource(QString cmd)
{
    int index = 0;
    bool flag = false;;

    if(cmd.left(11).compare("show motor ", Qt::CaseInsensitive) == 0){
        bool ok;
        index = cmd.mid(11,2).toInt(&ok, 10);
        if (ok && index < ACTUAL_MOTOR_NUMBER && index >= 0)
        {
            m_GetSourceData = RetriveMotorData;
            m_sourceIndex = index;
            flag = true;
        }
    }
    else if(cmd.left(11).compare("show force ", Qt::CaseInsensitive) == 0){
        bool ok;
        index = cmd.mid(11,2).toInt(&ok, 10);
        if (ok && index < ACTUAL_ATI_FORCE_SENSOR_NUM && index >= 0)
        {
            m_GetSourceData = RetriveForceData;
            m_sourceIndex = index;
            flag = true;
        }
        
    }
    else if(cmd.left(11).compare("show torqu ", Qt::CaseInsensitive) == 0){
        bool ok;
        index = cmd.mid(11,2).toInt(&ok, 10);
        if (ok && index < ACTUAL_ATI_FORCE_SENSOR_NUM && index >= 0)
        {
            m_GetSourceData = RetriveTorquData;
            m_sourceIndex = index;
            flag = true;
        }
    }
    else if(cmd.left(10).compare("show angle", Qt::CaseInsensitive) == 0){
        m_GetSourceData = RetriveAngleData;
        flag = true;
    }
    else if(cmd.left(10).compare("show omega", Qt::CaseInsensitive) == 0){
        m_GetSourceData = RetriveOmegaData;
        flag = true;
    }
    else if(cmd.left(10).compare("show accel", Qt::CaseInsensitive) == 0){
        m_GetSourceData = RetriveAccelData;
        flag = true;
    }
    if (flag)
    {
        ui->plainTextEditMsgLog->appendPlainText("Plotting souce changed");
    }
    else
    {
        ui->plainTextEditMsgLog->appendPlainText("Invalid syntax");
    }
}

int MainWindow::RetriveMotorData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }
    if (index >= ACTUAL_MOTOR_NUMBER)
         return -1;
    result[0] = source.feedbackData[index].Position;
    result[1] = source.feedbackData[index].Velocity;
    result[2] = source.feedbackData[index].Torque;

    return 0;
}

int MainWindow::RetriveForceData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }
    if (index >= ACTUAL_ATI_FORCE_SENSOR_NUM)
         return -1;

    for (int i = 0; i < 3; i++)
    {
        result[i] = source.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[i]/1000.0;
    }
    return 0;
}

int MainWindow::RetriveTorquData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }
    if (index >= ACTUAL_ATI_FORCE_SENSOR_NUM)
         return -1;

    for (int i = 0; i < 3; i++)
    {
        result[i] = source.forceData[RobotHighLevelControl::MapAbsToPhyForceSensor[i]].forceValues[i+3];
    }
    return 0;
}

int MainWindow::RetriveAngleData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }

    for (int i = 0; i < 3; i++)
    {
        result[i] = source.imuData.EulerAngle[i];
    }
    return -1;
}

int MainWindow::RetriveOmegaData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }

    for (int i = 0; i < 3; i++)
    {
        result[i] = source.imuData.AngularVel[i];
    }
    return -1;
}

int MainWindow::RetriveAccelData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index)
{
    if (result.length() != 3){
        result.resize(3);
    }

    for (int i = 0; i < 3; i++)
    {
        result[i] = source.imuData.LinearAcc[i];
    }
    return -1;
}
