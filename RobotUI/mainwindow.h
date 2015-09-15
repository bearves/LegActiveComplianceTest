#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ConnectionSetting.h"
#include <QMainWindow>
#include <QtNetwork>
#include <QTimer>
#include <iostream>
#include "Aris_Message.h"
#include "Aris_ControlData.h"
#include "GlobalConfiguration.h"
#include "Gait.h"
#include "qcustomplot.h"
#include <vector>
#include <memory>
#include "ParamSetWindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    static const int DATAGRAM_BUFFER_SIZE = 4096;
    Ui::MainWindow *ui;
    QTimer *m_timer;
    QTcpSocket *m_tcpSocket;
    ParamSetWindow *m_paramSetWindow;
    QHostAddress m_remoteAddr;
    QVector<QCustomPlot *> m_plotWidgets;

    Aris::Core::MSG m_robotMsgReceive;
    Aris::Core::MSG m_robotMsgToSend;
    Aris::RT_CONTROL::CMachineData m_machineData;
    int m_remotePort;
    int m_isConnected;
    int m_sendCount;
    int m_recvCount;
    int m_tickCount;
    int m_sourceIndex;

    char m_datagram[DATAGRAM_BUFFER_SIZE];

    void DisplayDeviceData(Aris::RT_CONTROL::CMachineData& machineData);
    void ProcessCommand(QString cmd);
    void ChangePlottingDataSource(QString cmd);
    void InitializePloting();

    QVector<double> timeLine;
    QVector<std::shared_ptr<QVector<double> > > plotData;

    typedef int (* RetriveDataFromSourceFunction)(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    RetriveDataFromSourceFunction m_GetSourceData;

    static int RetriveMotorData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    static int RetriveForceData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    static int RetriveTorquData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    static int RetriveAngleData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    static int RetriveOmegaData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);
    static int RetriveAccelData(const Aris::RT_CONTROL::CMachineData& source, QVector<double>& result, int index);

private slots:
    void OnPushbuttonConnectClicked();
    void OnSocketDisconnected();
    void OnDatagramReceived();
    void OnCommandLineReturned();
    void OnTimerTick();
};

#endif // MAINWINDOW_H
