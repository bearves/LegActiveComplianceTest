#include "ParamSetWindow.h"
#include "ui_ParamSetWindow.h"

ParamSetWindow::ParamSetWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ParamSetWindow)
{
    ui->setupUi(this);
    ui->pushButton_OK->connect(
                ui->pushButton_OK,
                SIGNAL(clicked()),
                this,
                SLOT(OnPushbuttonOKClicked()));

    ui->pushButton_cancel->connect(
                ui->pushButton_cancel,
                SIGNAL(clicked()),
                this,
                SLOT(OnPushbuttonCancelClicked()));
}

ParamSetWindow::~ParamSetWindow()
{
    delete ui;
}

void ParamSetWindow::OnPushbuttonOKClicked()
{
    RobotHighLevelControl::ParamCXB temp;
    bool flag = true;
    temp.duty             = ui->lineEditDuty->text().toDouble(&flag);
    if (!flag || temp.duty > 1 || temp.duty < 0)
    {
        ShowMessageBox("Wrong duty setting!");
        return;
    }
    temp.stepHeight       = ui->lineEditStepHeight->text().toDouble(&flag);
    if (!flag || temp.stepHeight < 0)
    {
        ShowMessageBox("Wrong stepHeight setting!");
        return;
    }
    temp.stepLength       = ui->lineEditStepLength->text().toDouble(&flag);
    if (!flag || temp.stepLength < 0)
    {
        ShowMessageBox("Wrong stepLength setting!");
        return;
    }
    temp.standHeight      = ui->lineEditStandHeight->text().toDouble(&flag);
    if (!flag || temp.standHeight < 450)
    {
        ShowMessageBox("Wrong standHeight setting!");
        return;
    }
    temp.tdDeltaMidLeg    = ui->lineEditStepDpMid->text().toDouble(&flag);
    if (!flag)
    {
        ShowMessageBox("Wrong Step DP Mid setting!");
        return;
    }
    temp.tdDeltaSideLeg   = ui->lineEditStepDpSide->text().toDouble(&flag);
    if (!flag)
    {
        ShowMessageBox("Wrong Step DP Side setting!");
        return;
    }
    temp.totalPeriodCount = ui->lineEditPeriodCount->text().toUInt(&flag);
    if (!flag)
    {
        ShowMessageBox("Wrong total period count setting!");
        return;
    }
    temp.T                = ui->lineEditPeriodTime->text().toDouble(&flag);
    if (!flag || temp.T < 0)
    {
        ShowMessageBox("Wrong period time setting!");
        return;
    }
    temp.Lside            = ui->lineEditLside->text().toDouble(&flag);
    if (!flag)
    {
        ShowMessageBox("Wrong Lside setting!");
        return;
    }

    m_param = temp;
    
    this->close();
}

void ParamSetWindow::ShowMessageBox(QString msg)
{
    QMessageBox msgBox;
    msgBox.setText(msg);
    msgBox.exec();
}

void ParamSetWindow::OnPushbuttonCancelClicked()
{
    this->close();
}

void ParamSetWindow::show()
{
    QMainWindow::show();
    ui->lineEditDuty->setText(QString::number(m_param.duty));
    ui->lineEditPeriodTime->setText(QString::number(m_param.T));
    ui->lineEditLside->setText(QString::number(m_param.Lside));
    ui->lineEditStepDpMid->setText(QString::number(m_param.tdDeltaMidLeg));
    ui->lineEditStepDpSide->setText(QString::number(m_param.tdDeltaSideLeg));
    ui->lineEditStepHeight->setText(QString::number(m_param.stepHeight));
    ui->lineEditStepLength->setText(QString::number(m_param.stepLength));
    ui->lineEditStandHeight->setText(QString::number(m_param.standHeight));
    ui->lineEditPeriodCount->setText(QString::number(m_param.totalPeriodCount));
}

void ParamSetWindow::SetParamData(RobotHighLevelControl::ParamCXB param)
{
    m_param = param;
}

RobotHighLevelControl::ParamCXB ParamSetWindow::GetParamData()
{
    return m_param;
}
