#ifndef TIMESYNCCLIENT_H
#define TIMESYNCCLIENT_H

#include <vector>
#include <cstdint>

#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <QElapsedTimer>

#include <rclcpp/rclcpp.hpp>

#include "serialhandler.h"
#include "communicationinterface.h"
#include "commands.h"

class TimeSyncClient : public QObject
{
    Q_OBJECT
public:
    explicit TimeSyncClient(SerialHandler *serialHandler = nullptr,
                            CommunicationInterface *communicationInterface = nullptr,
                            rclcpp::Clock *clock = nullptr,
                            QObject *parent = nullptr);

private:
    SerialHandler *serialHandler;
    CommunicationInterface *communicationInterface;
    QTimer syncTimer;
    int64_t t0, t3;
    int64_t avg_delta = 0;
    bool has_previous_sync = false;
    constexpr static double alpha = 0.2;

    bool isSyncing = false;

    rclcpp::Clock *clock;

    uint16_t crc16_ccitt(const uint8_t *data, size_t length);

public slots:
    void startSync();
    void stopSync();
private slots:
    void sendSyncRequest();
    void readSyncReply();

signals:
    void syncCompleted(int64_t avg_delta);

};

#endif // TIMESYNCCLIENT_H
