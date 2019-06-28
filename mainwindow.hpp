#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QTcpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QLineEdit>
#include <QList>

#include <cmath>

const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void acceptConnection();
    void receivedMessage();

private slots:
    void on_pushButtonConnect_clicked();
    void on_pushButtonDisconnect_clicked();

private:
    void initializeUi();

private:
    Ui::MainWindow *ui;

    QTcpSocket *client;

    QList<QLineEdit *> actualJointPositionEdits;
    QList<QLineEdit *> actualJointVelocityEdits;
    QList<QLineEdit *> actualJointCurrentEdits;

    QList<QLineEdit *> actualTcpPositionEdits;
    QList<QLineEdit *> actualTcpVelocityEdits;
    QList<QLineEdit *> actualTcpForceEdits;
};

#endif // MAINWINDOW_HPP
