#ifndef STRUCTSERIALISER_H
#define STRUCTSERIALISER_H

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QString>

#include "structs.h"

class StructSerialiser : public QObject {
    Q_OBJECT
private:
    static QString controlModeToString(ControlMode mode);
    static ControlMode controlModeFromString(const QString &str);

public:
    explicit StructSerialiser(QObject *parent = nullptr);

    // pid_constants_t
    static QJsonObject toJson(const pid_constants_t &);
    static pid_constants_t fromJson_pid(const QJsonObject &);

    // limits_pwm_t
    static QJsonObject toJson(const limits_pwm_t &);
    static limits_pwm_t fromJson_limits_pwm(const QJsonObject &);

    // connections_wheel_t
    static QJsonObject toJson(const connections_wheel_t &);
    static connections_wheel_t fromJson_connections_wheel(const QJsonObject &);

    // odo_broadcast_flags_t
    static QJsonObject toJson(const odo_broadcast_flags_t &);
    static odo_broadcast_flags_t
    fromJson_odo_broadcast_flags(const QJsonObject &);

    // odometry_t
    static QJsonObject toJson(const odometry_t &);
    static odometry_t fromJson_odometry(const QJsonObject &);

    // setpoint_t
    static QJsonObject toJson(const setpoint_t &);
    static setpoint_t fromJson_setpoint(const QJsonObject &);

    // wheel_update_frequencies_t
    static QJsonObject toJson(const wheel_update_frequencies_t &);
    static wheel_update_frequencies_t
    fromJson_wheel_update_frequencies(const QJsonObject &);

    // ControlMode enum
    static QJsonValue controlModeToJson(ControlMode mode);
    static ControlMode controlModeFromJson(const QJsonValue &value);

    // wheel_data_t
    static QJsonObject toJson(const wheel_data_t &);
    static wheel_data_t fromJson_wheel_data(const QJsonObject &);

    // update_frequencies_t
    static QJsonObject toJson(const update_frequencies_t &);
    static update_frequencies_t fromJson_update_frequencies(const QJsonObject &);

    // controller_properties_t
    static QJsonObject toJson(const controller_properties_t &);
    static controller_properties_t
    fromJson_controller_properties(const QJsonObject &);

    // serial_conn_details_t
    static QJsonObject toJson(const serial_conn_details_t &scd);
    static serial_conn_details_t
    fromJson_serial_conn_details(const QJsonObject &obj);

    // controller_data_t
    static QJsonObject toJson(const controller_data_t &);
    static controller_data_t fromJson_controller_data(const QJsonObject &);

signals:
};

#endif // STRUCTSERIALISER_H
