#include "serialhandler.h"
#include <QDebug>

SerialHandler::SerialHandler(QObject *parent)
    : QObject{parent}, serial(new QSerialPort(this))
{
    connect(serial, &QSerialPort::readyRead, this,
            &SerialHandler::handleReadyRead);
    connect(this, &SerialHandler::parseBufferSignal, this,
            &SerialHandler::parseBuffer, Qt::QueuedConnection);
}

SerialHandler::~SerialHandler()
{
    if (serial->isOpen())
        serial->close();
}

bool SerialHandler::connectSerial(const QString &port_name, int baud_rate,
                                  QString *error_message)
{
    if (serial->isOpen())
    {
        serial->close();
    }

    serial->setPortName(port_name);
    serial->setBaudRate(static_cast<QSerialPort::BaudRate>(baud_rate));
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadWrite))
    {
        *error_message = "";
        return true;
    }
    else
    {
        *error_message = serial->errorString();
        return false;
    }
}

bool SerialHandler::disconnectSerial()
{
    if (serial->isOpen())
    {
        serial->close();
        return true;
    }
    return false;
}

bool SerialHandler::checkConnection()
{
    if (!serial)
    {
        return false;
    }
    return serial->isOpen();
}

QList<QSerialPortInfo> SerialHandler::getAvailablePorts()
{
    return QSerialPortInfo::availablePorts();
}

bool SerialHandler::isHeader(uint8_t *inp)
{
    return (inp[0] == header && inp[1] == header && inp[2] == header);
}

bool SerialHandler::sendCommand(Command command)
{
    QByteArray arr(3, header);
    arr.append(static_cast<uint8_t>(command));
    bool status = serial->write(arr) == 4;
    return status;
}

bool SerialHandler::sendCommandandFlush(Command command)
{
    QByteArray arr(3, header);
    arr.append(static_cast<uint8_t>(command));
    bool status = serial->write(arr) == 4;
    serial->flush();
    return status;
}

qint64 SerialHandler::ping(Command command)
{
    QByteArray data = serial->readAll();
    buffer.insert(buffer.end(), data.begin(), data.end());
    serial->clear();

    sendCommand(command);
    serial->flush();

    QElapsedTimer timer;
    timer.start();

    if (readCommand(command))
    {
        if (command == Command::PING)
        {
            qint64 elapsed_time = timer.elapsed();
            return elapsed_time;
        }
        return -1;
    }
    else
        return -1;
}

bool SerialHandler::readCommand(Command &command, int timeoutMs)
{
    if (!serial || !serial->isOpen())
        return false;

    QElapsedTimer timer;
    timer.start();

    uint8_t temp[3];

    // Wait for the header byte
    while (timer.elapsed() < timeoutMs)
    {
        if (serial->waitForReadyRead(1))
        {
            if (serial->read(reinterpret_cast<char *>(temp), 3) == 3 &&
                isHeader(temp))
            {
                // Header found, read the command byte
                while (timer.elapsed() < timeoutMs)
                {
                    if (serial->read(reinterpret_cast<char *>(temp), 1) == 1)
                    {
                        command = static_cast<Command>(temp[0]);
                        return true;
                    }
                }
                return false; // Failed to read the command byte
            }
        }
    }

    return false; // Timeout waiting for header
}

bool SerialHandler::sendData(const uint8_t *data, size_t length)
{
    if (serial->write(reinterpret_cast<const char *>(data), length) == length)
    {
        return true;
    }
    return false;
}

bool SerialHandler::sendData(const std::vector<uint8_t> &data)
{
    if (serial->write(reinterpret_cast<const char *>(data.data()), data.size()) ==
        data.size())
    {
        return true;
    }
    return false;
}

std::vector<uint8_t> SerialHandler::readData(size_t length, int timeoutMs)
{
    std::vector<uint8_t> data;
    data.reserve(length);

    if (!serial || !serial->isOpen())
        return data;

    QElapsedTimer timer;
    timer.start();

    while (buffer.size() < length && timer.elapsed() < timeoutMs)
    {
        serial->waitForReadyRead(10);
        QByteArray temp = serial->readAll();

        if (!temp.isEmpty())
        {
            buffer.insert(buffer.end(), temp.begin(), temp.end());
        }
    }

    if (buffer.size() >= length)
    {
        data.insert(data.end(), buffer.begin(), buffer.begin() + length);
        buffer.erase(buffer.begin(), buffer.begin() + length);
    }
    emit parseBufferSignal();
    return data;
}

void SerialHandler::parseBuffer()
{
    while (buffer.size() > 3)
    {
        auto it = std::find(buffer.begin(), buffer.end(), header);

        if (it == buffer.end())
        {
            buffer.clear();
            break;
        }
        size_t header_index = std::distance(buffer.begin(), it);
        if (buffer.size() <= header_index + 3)
        {
            break;
        }
        if (!isHeader(&(*it)))
        {
            buffer.erase(buffer.begin(), buffer.begin() + header_index + 1);
            continue;
        }

        uint8_t command = buffer[header_index + 3];

        buffer.erase(buffer.begin(), buffer.begin() + header_index + 4);

        emit commandReceived(command);
    }
}

void SerialHandler::handleReadyRead()
{
    if (auto_read_command_flag) [[likely]]
    {
        QByteArray data = serial->readAll();
        buffer.insert(buffer.end(), data.begin(), data.end());
        parseBuffer();
    }
}

void SerialHandler::setAutoReadCommandFlag(bool flag)
{
    auto_read_command_flag = flag;
}
