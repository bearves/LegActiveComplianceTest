/********************************************************************************
** Form generated from reading UI file 'ParamSetWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PARAMSETWINDOW_H
#define UI_PARAMSETWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ParamSetWindow
{
public:
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_8;
    QLabel *labelPeriodCount;
    QLineEdit *lineEditPeriodCount;
    QHBoxLayout *horizontalLayout_7;
    QLabel *labelStepLength;
    QLineEdit *lineEditStepLength;
    QHBoxLayout *horizontalLayout_6;
    QLabel *labelStepHeight;
    QLineEdit *lineEditStepHeight;
    QHBoxLayout *horizontalLayout_5;
    QLabel *labelStandHeight;
    QLineEdit *lineEditStandHeight;
    QHBoxLayout *horizontalLayout_11;
    QLabel *labelDuty;
    QLineEdit *lineEditDuty;
    QHBoxLayout *horizontalLayout_9;
    QLabel *labelPeriodTime;
    QLineEdit *lineEditPeriodTime;
    QHBoxLayout *horizontalLayout_10;
    QLabel *labelLSide;
    QLineEdit *lineEditLside;
    QHBoxLayout *horizontalLayout_2;
    QLabel *labelStepDpMid;
    QLineEdit *lineEditStepDpMid;
    QHBoxLayout *horizontalLayout_3;
    QLabel *labelStepDpSide;
    QLineEdit *lineEditStepDpSide;
    QHBoxLayout *horizontalLayout_12;
    QLabel *labelRotationAngle;
    QLineEdit *lineRotationAngle;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_OK;
    QPushButton *pushButton_cancel;

    void setupUi(QMainWindow *ParamSetWindow)
    {
        if (ParamSetWindow->objectName().isEmpty())
            ParamSetWindow->setObjectName(QStringLiteral("ParamSetWindow"));
        ParamSetWindow->resize(313, 472);
        ParamSetWindow->setContextMenuPolicy(Qt::NoContextMenu);
        centralWidget = new QWidget(ParamSetWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayoutWidget = new QWidget(centralWidget);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 10, 291, 458));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        labelPeriodCount = new QLabel(horizontalLayoutWidget);
        labelPeriodCount->setObjectName(QStringLiteral("labelPeriodCount"));
        labelPeriodCount->setMinimumSize(QSize(150, 0));
        labelPeriodCount->setMaximumSize(QSize(150, 16777215));
        QFont font;
        font.setPointSize(12);
        labelPeriodCount->setFont(font);

        horizontalLayout_8->addWidget(labelPeriodCount);

        lineEditPeriodCount = new QLineEdit(horizontalLayoutWidget);
        lineEditPeriodCount->setObjectName(QStringLiteral("lineEditPeriodCount"));
        lineEditPeriodCount->setFont(font);

        horizontalLayout_8->addWidget(lineEditPeriodCount);


        verticalLayout->addLayout(horizontalLayout_8);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        labelStepLength = new QLabel(horizontalLayoutWidget);
        labelStepLength->setObjectName(QStringLiteral("labelStepLength"));
        labelStepLength->setMinimumSize(QSize(150, 0));
        labelStepLength->setMaximumSize(QSize(150, 16777215));
        labelStepLength->setFont(font);

        horizontalLayout_7->addWidget(labelStepLength);

        lineEditStepLength = new QLineEdit(horizontalLayoutWidget);
        lineEditStepLength->setObjectName(QStringLiteral("lineEditStepLength"));
        lineEditStepLength->setFont(font);

        horizontalLayout_7->addWidget(lineEditStepLength);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        labelStepHeight = new QLabel(horizontalLayoutWidget);
        labelStepHeight->setObjectName(QStringLiteral("labelStepHeight"));
        labelStepHeight->setMinimumSize(QSize(150, 0));
        labelStepHeight->setMaximumSize(QSize(150, 16777215));
        labelStepHeight->setFont(font);

        horizontalLayout_6->addWidget(labelStepHeight);

        lineEditStepHeight = new QLineEdit(horizontalLayoutWidget);
        lineEditStepHeight->setObjectName(QStringLiteral("lineEditStepHeight"));
        lineEditStepHeight->setFont(font);

        horizontalLayout_6->addWidget(lineEditStepHeight);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        labelStandHeight = new QLabel(horizontalLayoutWidget);
        labelStandHeight->setObjectName(QStringLiteral("labelStandHeight"));
        labelStandHeight->setMinimumSize(QSize(150, 0));
        labelStandHeight->setMaximumSize(QSize(150, 16777215));
        labelStandHeight->setFont(font);

        horizontalLayout_5->addWidget(labelStandHeight);

        lineEditStandHeight = new QLineEdit(horizontalLayoutWidget);
        lineEditStandHeight->setObjectName(QStringLiteral("lineEditStandHeight"));
        lineEditStandHeight->setFont(font);

        horizontalLayout_5->addWidget(lineEditStandHeight);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        labelDuty = new QLabel(horizontalLayoutWidget);
        labelDuty->setObjectName(QStringLiteral("labelDuty"));
        labelDuty->setMinimumSize(QSize(150, 0));
        labelDuty->setMaximumSize(QSize(150, 16777215));
        labelDuty->setFont(font);

        horizontalLayout_11->addWidget(labelDuty);

        lineEditDuty = new QLineEdit(horizontalLayoutWidget);
        lineEditDuty->setObjectName(QStringLiteral("lineEditDuty"));
        lineEditDuty->setFont(font);

        horizontalLayout_11->addWidget(lineEditDuty);


        verticalLayout->addLayout(horizontalLayout_11);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        labelPeriodTime = new QLabel(horizontalLayoutWidget);
        labelPeriodTime->setObjectName(QStringLiteral("labelPeriodTime"));
        labelPeriodTime->setMinimumSize(QSize(150, 0));
        labelPeriodTime->setMaximumSize(QSize(150, 16777215));
        labelPeriodTime->setFont(font);

        horizontalLayout_9->addWidget(labelPeriodTime);

        lineEditPeriodTime = new QLineEdit(horizontalLayoutWidget);
        lineEditPeriodTime->setObjectName(QStringLiteral("lineEditPeriodTime"));
        lineEditPeriodTime->setFont(font);

        horizontalLayout_9->addWidget(lineEditPeriodTime);


        verticalLayout->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        labelLSide = new QLabel(horizontalLayoutWidget);
        labelLSide->setObjectName(QStringLiteral("labelLSide"));
        labelLSide->setMinimumSize(QSize(150, 0));
        labelLSide->setMaximumSize(QSize(150, 16777215));
        labelLSide->setFont(font);

        horizontalLayout_10->addWidget(labelLSide);

        lineEditLside = new QLineEdit(horizontalLayoutWidget);
        lineEditLside->setObjectName(QStringLiteral("lineEditLside"));
        lineEditLside->setFont(font);

        horizontalLayout_10->addWidget(lineEditLside);


        verticalLayout->addLayout(horizontalLayout_10);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        labelStepDpMid = new QLabel(horizontalLayoutWidget);
        labelStepDpMid->setObjectName(QStringLiteral("labelStepDpMid"));
        labelStepDpMid->setMinimumSize(QSize(150, 0));
        labelStepDpMid->setMaximumSize(QSize(150, 16777215));
        labelStepDpMid->setFont(font);

        horizontalLayout_2->addWidget(labelStepDpMid);

        lineEditStepDpMid = new QLineEdit(horizontalLayoutWidget);
        lineEditStepDpMid->setObjectName(QStringLiteral("lineEditStepDpMid"));
        lineEditStepDpMid->setFont(font);

        horizontalLayout_2->addWidget(lineEditStepDpMid);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        labelStepDpSide = new QLabel(horizontalLayoutWidget);
        labelStepDpSide->setObjectName(QStringLiteral("labelStepDpSide"));
        labelStepDpSide->setMinimumSize(QSize(150, 0));
        labelStepDpSide->setMaximumSize(QSize(150, 16777215));
        labelStepDpSide->setFont(font);

        horizontalLayout_3->addWidget(labelStepDpSide);

        lineEditStepDpSide = new QLineEdit(horizontalLayoutWidget);
        lineEditStepDpSide->setObjectName(QStringLiteral("lineEditStepDpSide"));
        lineEditStepDpSide->setFont(font);

        horizontalLayout_3->addWidget(lineEditStepDpSide);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        labelRotationAngle = new QLabel(horizontalLayoutWidget);
        labelRotationAngle->setObjectName(QStringLiteral("labelRotationAngle"));
        labelRotationAngle->setMinimumSize(QSize(150, 0));
        labelRotationAngle->setMaximumSize(QSize(150, 16777215));
        labelRotationAngle->setFont(font);

        horizontalLayout_12->addWidget(labelRotationAngle);

        lineRotationAngle = new QLineEdit(horizontalLayoutWidget);
        lineRotationAngle->setObjectName(QStringLiteral("lineRotationAngle"));
        lineRotationAngle->setFont(font);

        horizontalLayout_12->addWidget(lineRotationAngle);


        verticalLayout->addLayout(horizontalLayout_12);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        pushButton_OK = new QPushButton(horizontalLayoutWidget);
        pushButton_OK->setObjectName(QStringLiteral("pushButton_OK"));

        horizontalLayout->addWidget(pushButton_OK);

        pushButton_cancel = new QPushButton(horizontalLayoutWidget);
        pushButton_cancel->setObjectName(QStringLiteral("pushButton_cancel"));

        horizontalLayout->addWidget(pushButton_cancel);


        verticalLayout->addLayout(horizontalLayout);


        horizontalLayout_4->addLayout(verticalLayout);

        ParamSetWindow->setCentralWidget(centralWidget);

        retranslateUi(ParamSetWindow);

        QMetaObject::connectSlotsByName(ParamSetWindow);
    } // setupUi

    void retranslateUi(QMainWindow *ParamSetWindow)
    {
        ParamSetWindow->setWindowTitle(QApplication::translate("ParamSetWindow", "Param settings", 0));
        labelPeriodCount->setText(QApplication::translate("ParamSetWindow", "Period Count:", 0));
        labelStepLength->setText(QApplication::translate("ParamSetWindow", "Step Length:", 0));
        labelStepHeight->setText(QApplication::translate("ParamSetWindow", "Step Height:", 0));
        labelStandHeight->setText(QApplication::translate("ParamSetWindow", "Stand Height:", 0));
        labelDuty->setText(QApplication::translate("ParamSetWindow", "Duty:", 0));
        labelPeriodTime->setText(QApplication::translate("ParamSetWindow", "Period Time:", 0));
        labelLSide->setText(QApplication::translate("ParamSetWindow", "LSide:", 0));
        labelStepDpMid->setText(QApplication::translate("ParamSetWindow", "Step Depth (Mid):", 0));
        labelStepDpSide->setText(QApplication::translate("ParamSetWindow", "Step Depth (Side):", 0));
        labelRotationAngle->setText(QApplication::translate("ParamSetWindow", "RotationAngle:", 0));
        pushButton_OK->setText(QApplication::translate("ParamSetWindow", "OK", 0));
        pushButton_cancel->setText(QApplication::translate("ParamSetWindow", "Cancel", 0));
    } // retranslateUi

};

namespace Ui {
    class ParamSetWindow: public Ui_ParamSetWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARAMSETWINDOW_H
