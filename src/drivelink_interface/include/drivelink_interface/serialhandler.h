#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <QElapsedTimer>
#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QThread>
#include <deque>

#include "commands.h"

class SerialHandler : public QObject
{
    Q_OBJECT

private:
    QSerialPort *serial;

    const uint8_t header = 0xaa;

    bool auto_read_command_flag = true;

    bool isHeader(uint8_t *);

    std::deque<uint8_t> buffer;

private slots:
    void parseBuffer();

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler();

    bool connectSerial(const QString &port_name, int baud_rate,
                       QString *error_message);
    bool disconnectSerial();
    bool checkConnection();

    qint64 ping(Command command);
    bool sendCommand(Command command);
    bool sendCommandandFlush(Command command);
    bool readCommand(Command &command, int timeoutMs = 1000);
    bool sendData(const uint8_t *data, size_t length);
    bool sendData(const std::vector<uint8_t> &data);
    std::vector<uint8_t> readData(size_t length, int timeoutMs = 1000);
    void setAutoReadCommandFlag(bool flag);
    QList<QSerialPortInfo> getAvailablePorts();

public slots:
    void handleReadyRead();

signals:
    void commandReceived(uint8_t command);
    void parseBufferSignal();
};

#endif // SERIALHANDLER_H
