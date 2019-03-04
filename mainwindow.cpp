#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QDebug>
#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
    ui->setupUi(this);

    initializeUi();

    client = new QTcpSocket(this);
    connect(client, &QTcpSocket::connected, this, &MainWindow::acceptConnection);
    connect(client, &QTcpSocket::disconnected, this, &MainWindow::on_pushButtonDisconnect_clicked);

    timer = new QTimer(this);
}

MainWindow::~MainWindow() {
    on_pushButtonDisconnect_clicked();
    delete ui;
}

void MainWindow::acceptConnection() {
    ui->pushButtonConnect->setEnabled(false);
    ui->pushButtonDisconnect->setEnabled(true);

    ui->statusBar->showMessage("connected", 3000);
    connect(client, &QTcpSocket::readyRead, this, &MainWindow::receivedMessage);

    timer->start(100);
    connect(timer, &QTimer::timeout, this, &MainWindow::onTimer);
}

void MainWindow::receivedMessage() {
    QByteArray recv = client->readAll();
    if (recv.size() == 1108) {
        std::vector<char> buffer(recv.data(), recv.data() + recv.size());
        packet = UR10::convert(buffer);
    }
}

void MainWindow::on_pushButtonConnect_clicked() {
    QHostAddress address(ui->lineEditAddress->text());
    quint16 port = ui->spinBoxIp->value();
    client->connectToHost(address, port);
}

void MainWindow::on_pushButtonDisconnect_clicked() {
    client->disconnectFromHost();
    client->close();
    ui->pushButtonConnect->setEnabled(true);
    ui->pushButtonDisconnect->setEnabled(false);
    timer->stop();
}

void MainWindow::onTimer() {
    if (packet.m_dMsgSize == UR10::PACKET_SIZE) {
        for (int i = 0; i < 6; i++) {
            actualJointPositionEdits.at(i)->setText(QString::number(packet.m_fArrQActual[i] * RAD2DEG, 'f', 3));
            actualJointVelocityEdits.at(i)->setText(QString::number(packet.m_fArrQDActual[i] * RAD2DEG, 'f', 3));
            actualJointCurrentEdits.at(i)->setText(QString::number(packet.m_fArrIActual[i] * RAD2DEG, 'f', 3));

            actualTcpPositionEdits.at(i)->setText(QString::number(packet.m_fArrToolVectorActual[i], 'f', 3));
            actualTcpVelocityEdits.at(i)->setText(QString::number(packet.m_fArrTCPSpeedActual[i], 'f', 3));
            actualTcpForceEdits.at(i)->setText(QString::number(packet.m_fArrTCPForce[i], 'f', 3));
        }
    }
}

void MainWindow::initializeUi() {

    actualJointPositionEdits << ui->lineEditPositionJ1
                             << ui->lineEditPositionJ2
                             << ui->lineEditPositionJ3
                             << ui->lineEditPositionJ4
                             << ui->lineEditPositionJ5
                             << ui->lineEditPositionJ6;

    actualJointVelocityEdits << ui->lineEditVelocityJ1
                             << ui->lineEditVelocityJ2
                             << ui->lineEditVelocityJ3
                             << ui->lineEditVelocityJ4
                             << ui->lineEditVelocityJ5
                             << ui->lineEditVelocityJ6;

    actualJointCurrentEdits << ui->lineEditCurrentJ1
                            << ui->lineEditCurrentJ2
                            << ui->lineEditCurrentJ3
                            << ui->lineEditCurrentJ4
                            << ui->lineEditCurrentJ5
                            << ui->lineEditCurrentJ6;

    actualTcpPositionEdits << ui->lineEditPositionX
                           << ui->lineEditPositionY
                           << ui->lineEditPositionZ
                           << ui->lineEditPositionRx
                           << ui->lineEditPositionRy
                           << ui->lineEditPositionRz;

    actualTcpVelocityEdits << ui->lineEditVelocityX
                           << ui->lineEditVelocityY
                           << ui->lineEditVelocityZ
                           << ui->lineEditVelocityRx
                           << ui->lineEditVelocityRy
                           << ui->lineEditVelocityRz;

    actualTcpForceEdits << ui->lineEditForceX
                        << ui->lineEditForceY
                        << ui->lineEditForceZ
                        << ui->lineEditForceRx
                        << ui->lineEditForceRy
                        << ui->lineEditForceRz;
}
