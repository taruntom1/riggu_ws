#include "structserialiser.h"
#include <QString>

StructSerialiser::StructSerialiser(QObject *parent) : QObject{parent} {}

QJsonObject StructSerialiser::toJson(const pid_constants_t &pid) {
    QJsonObject obj;
    obj["p"] = pid.p;
    obj["i"] = pid.i;
    obj["d"] = pid.d;
    return obj;
}
pid_constants_t StructSerialiser::fromJson_pid(const QJsonObject &obj) {
    pid_constants_t pid;
    pid.p = float(obj.value("p").toDouble());
    pid.i = float(obj.value("i").toDouble());
    pid.d = float(obj.value("d").toDouble());
    return pid;
}

// limits_pwm_t
QJsonObject StructSerialiser::toJson(const limits_pwm_t &lim) {
    QJsonObject obj;
    obj["min"] = lim.min;
    obj["max"] = lim.max;
    return obj;
}
limits_pwm_t StructSerialiser::fromJson_limits_pwm(const QJsonObject &obj) {
    limits_pwm_t lim;
    lim.min = int8_t(obj.value("min").toInt());
    lim.max = int8_t(obj.value("max").toInt());
    return lim;
}

// connections_wheel_t
QJsonObject StructSerialiser::toJson(const connections_wheel_t &cw) {
    QJsonObject obj;
    obj["dir"] = cw.dir;
    obj["pwm"] = cw.pwm;
    obj["enc_a"] = cw.enc_a;
    obj["enc_b"] = cw.enc_b;
    return obj;
}
connections_wheel_t
StructSerialiser::fromJson_connections_wheel(const QJsonObject &obj) {
    connections_wheel_t cw;
    cw.dir = uint8_t(obj.value("dir").toInt());
    cw.pwm = uint8_t(obj.value("pwm").toInt());
    cw.enc_a = uint8_t(obj.value("enc_a").toInt());
    cw.enc_b = uint8_t(obj.value("enc_b").toInt());
    return cw;
}

// odo_broadcast_flags_t
QJsonObject StructSerialiser::toJson(const odo_broadcast_flags_t &flags) {
    QJsonObject obj;
    obj["angle"] = flags.angle;
    obj["speed"] = flags.speed;
    obj["pwm_value"] = flags.pwm_value;
    return obj;
}
odo_broadcast_flags_t
StructSerialiser::fromJson_odo_broadcast_flags(const QJsonObject &obj) {
    odo_broadcast_flags_t flags;
    flags.angle = obj.value("angle").toBool(false);
    flags.speed = obj.value("speed").toBool(false);
    flags.pwm_value = obj.value("pwm_value").toBool(false);
    return flags;
}

// odometry_t
QJsonObject StructSerialiser::toJson(const odometry_t &odo) {
    QJsonObject obj;
    obj["angle"] = odo.angle;
    obj["rpm"] = odo.angular_velocity;
    return obj;
}
odometry_t StructSerialiser::fromJson_odometry(const QJsonObject &obj) {
    odometry_t odo;
    odo.angle = angle_t(obj.value("angle").toInt());
    odo.angular_velocity = angularvelocity_t(obj.value("rpm").toDouble());
    return odo;
}

// setpoint_t
QJsonObject StructSerialiser::toJson(const setpoint_t &sp) {
    QJsonObject obj;
    obj["angle"] = sp.angle;
    obj["rpm"] = sp.rpm;
    return obj;
}
setpoint_t StructSerialiser::fromJson_setpoint(const QJsonObject &obj) {
    setpoint_t sp;
    sp.angle = angle_t(obj.value("angle").toInt());
    sp.rpm = angularvelocity_t(obj.value("rpm").toDouble());
    return sp;
}

// wheel_update_frequencies_t
QJsonObject StructSerialiser::toJson(const wheel_update_frequencies_t &freq) {
    QJsonObject obj;
    obj["pwm"] = int(freq.pwm);
    obj["angle_pid"] = int(freq.angle_pid);
    obj["speed_pid"] = int(freq.speed_pid);
    return obj;
}
wheel_update_frequencies_t
StructSerialiser::fromJson_wheel_update_frequencies(const QJsonObject &obj) {
    wheel_update_frequencies_t freq;
    freq.pwm = uint16_t(obj.value("pwm").toInt());
    freq.angle_pid = uint16_t(obj.value("angle_pid").toInt());
    freq.speed_pid = uint16_t(obj.value("speed_pid").toInt());
    return freq;
}

// ControlMode enum
QString StructSerialiser::controlModeToString(ControlMode mode) {
    switch (mode) {
    case ControlMode::POSITION_CONTROL:
        return "POSITION_CONTROL";
    case ControlMode::SPEED_CONTROL:
        return "SPEED_CONTROL";
    case ControlMode::PWM_DIRECT_CONTROL:
        return "PWM_DIRECT_CONTROL";
    case ControlMode::OFF:
        return "OFF";
    default:
        return "OFF";
    }
}

ControlMode StructSerialiser::controlModeFromString(const QString &str) {
    if (str == "POSITION_CONTROL")
        return ControlMode::POSITION_CONTROL;
    if (str == "SPEED_CONTROL")
        return ControlMode::SPEED_CONTROL;
    if (str == "PWM_DIRECT_CONTROL")
        return ControlMode::PWM_DIRECT_CONTROL;
    return ControlMode::OFF;
}

QJsonValue StructSerialiser::controlModeToJson(ControlMode mode) {
    return QJsonValue(controlModeToString(mode));
}

ControlMode StructSerialiser::controlModeFromJson(const QJsonValue &value) {
    return controlModeFromString(value.toString());
}

// struct wheel_data_t
QJsonObject StructSerialiser::toJson(const wheel_data_t &wd) {
    QJsonObject obj;
    obj["motor_id"] = wd.motor_id;
    obj["control_mode"] = controlModeToString(wd.control_mode);
    obj["anglePIDConstants"] = toJson(wd.anglePIDConstants);
    obj["speedPIDConstants"] = toJson(wd.speedPIDConstants);
    obj["motorConnections"] = toJson(wd.motorConnections);
    obj["odoBroadcastStatus"] = toJson(wd.odoBroadcastStatus);
    obj["radians_per_tick"] = wd.radians_per_tick;
    return obj;
}

wheel_data_t StructSerialiser::fromJson_wheel_data(const QJsonObject &obj) {
    wheel_data_t wd;
    wd.motor_id = uint8_t(obj.value("motor_id").toInt());
    wd.control_mode = controlModeFromString(obj.value("control_mode").toString());
    wd.anglePIDConstants =
        fromJson_pid(obj.value("anglePIDConstants").toObject());
    wd.speedPIDConstants =
        fromJson_pid(obj.value("speedPIDConstants").toObject());
    wd.motorConnections =
        fromJson_connections_wheel(obj.value("motorConnections").toObject());
    wd.odoBroadcastStatus =
        fromJson_odo_broadcast_flags(obj.value("odoBroadcastStatus").toObject());
    wd.radians_per_tick = float(obj.value("radians_per_tick").toDouble());
    return wd;
}

// update_frequencies_t
QJsonObject StructSerialiser::toJson(const update_frequencies_t &obj) {
    QJsonObject json;
    json["control_run_frequency"] = obj.control_run_frequency;
    json["odo_broadcast_frequency"] = obj.odoBroadcastFrequency;
    return json;
}
update_frequencies_t
StructSerialiser::fromJson_update_frequencies(const QJsonObject &obj) {
    update_frequencies_t wd;
    wd.control_run_frequency = obj.value("control_run_frequency").toInt();
    wd.odoBroadcastFrequency = obj.value("odo_broadcast_frequency").toInt();

    return wd;
}

// controller_properties_t
QJsonObject StructSerialiser::toJson(const controller_properties_t &cp) {
    QJsonObject obj;
    obj["numMotors"] = cp.numMotors;
    obj["updateFrequencies"] = toJson(cp.updateFrequencies);
    return obj;
}

controller_properties_t
StructSerialiser::fromJson_controller_properties(const QJsonObject &obj) {
    controller_properties_t cp;
    cp.numMotors = uint8_t(obj.value("numMotors").toInt());
    cp.updateFrequencies = fromJson_update_frequencies(
        obj.value("updateFrequencies")
            .toObject()); // Adjust function name if needed!
    return cp;
}

// serial_conn_details_t
QJsonObject StructSerialiser::toJson(const serial_conn_details_t &scd) {
    QJsonObject obj;
    obj["port"] = QString::fromStdString(scd.port);
    obj["baud"] = scd.baud;
    return obj;
}

serial_conn_details_t
StructSerialiser::fromJson_serial_conn_details(const QJsonObject &obj) {
    serial_conn_details_t scd;
    scd.port = QString(obj.value("port").toString()).toStdString();
    scd.baud = obj.value("baud").toInt();
    return scd;
}

// controller_data_t

QJsonObject StructSerialiser::toJson(const controller_data_t &cd) {
    QJsonObject obj;

    // Serialize wheelData as a JSON array
    QJsonArray wheelArray;
    for (const auto &wheel : cd.wheelData)
        wheelArray.append(toJson(wheel));
    obj["wheelData"] = wheelArray;

    // Serialize controllerProperties
    obj["controllerProperties"] = toJson(cd.controllerProperties);

    // Serialize serialConnDetails
    obj["serialConnDetails"] = toJson(cd.conn_details);

    return obj;
}

controller_data_t
StructSerialiser::fromJson_controller_data(const QJsonObject &obj) {
    controller_data_t cd;

    // Deserialize wheelData array
    QJsonArray wheelArray = obj.value("wheelData").toArray();
    cd.wheelData.clear();
    cd.wheelData.reserve(wheelArray.size());
    for (const QJsonValue &v : wheelArray)
        cd.wheelData.push_back(fromJson_wheel_data(v.toObject()));

    // Deserialize controllerProperties
    cd.controllerProperties = fromJson_controller_properties(
        obj.value("controllerProperties").toObject());

    // Deserialize serialConnDetails
    cd.conn_details =
        fromJson_serial_conn_details(obj.value("serialConnDetails").toObject());

    return cd;
}
