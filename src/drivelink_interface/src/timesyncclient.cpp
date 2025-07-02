#include "timesyncclient.h"

#include <QDebug>

TimeSyncClient::TimeSyncClient(SerialHandler *serialHandler,
                               CommunicationInterface *communicationInterface,
                               rclcpp::Clock *clock,
                               QObject *parent)
    : QObject(parent), serialHandler(serialHandler), communicationInterface(communicationInterface),
      clock(clock)
{
    connect(&syncTimer, &QTimer::timeout, this, &TimeSyncClient::sendSyncRequest);
    connect(communicationInterface, &CommunicationInterface::timeSyncReplyReceived,
            this, &TimeSyncClient::readSyncReply, Qt::DirectConnection);

    syncTimer.start(3000); // 3 seconds interval for time synchronization
    startSync();
}

uint16_t TimeSyncClient::crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void TimeSyncClient::startSync()
{
    isSyncing = true;
}

void TimeSyncClient::stopSync()
{
    isSyncing = false;
}

void TimeSyncClient::sendSyncRequest()
{
    if (!isSyncing)
        return;

    t0 = clock->now().nanoseconds();
    serialHandler->sendCommandandFlush(Command::SYNC_TIME);
}

void TimeSyncClient::readSyncReply()
{
    std::vector<uint8_t> data = serialHandler->readData(10);
    if (data.size() < 10)
        return;

    // Extract t1
    uint64_t &t1 = *reinterpret_cast<uint64_t *>(data.data());

    // Extract CRC and verify
    uint16_t &received_crc = *reinterpret_cast<uint16_t *>(data.data() + 8);
    uint16_t computed_crc = crc16_ccitt(data.data(), 8);
    if (received_crc != computed_crc)
        return;

    // Record arrival time (t3)
    t3 = clock->now().nanoseconds();

    int64_t sync_sys_time_ns = (t0 + t3) / 2;
    int64_t sync_mcu_time_ns = t1 * 1000;

    int64_t delta = sync_sys_time_ns - sync_mcu_time_ns;
    // int64_t delay = t3 - t0;

    if (has_previous_sync)
    {
        avg_delta = static_cast<int64_t>(alpha * delta + (1.0 - alpha) * avg_delta);
    }
    else
    {
        avg_delta = delta;
        has_previous_sync = true;
    }

    emit syncCompleted(delta);
}
