#pragma once
#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"
#include "MyDataProto.hpp"
#include "MySerial.hpp"


class Base_Robot : public isaac::alice::Codelet {
public:
    void start() override;
    void tick() override;
    void stop() override;

    // 串口类
    Serial serial;
    //数据协议结构体
    TranData serial_data;
    size_t DataSize = sizeof(serial_data);
    uint8_t * serial_data_buf = (uint8_t *)&serial_data;

    // 串口号
    ISAAC_PARAM(std::string, serial_port, "/dev/ttyUSB1");

    ISAAC_PARAM(bool, flag, true);



    // 接受虚拟手柄数据
    ISAAC_PROTO_RX(JoystickStateProto, js_state);

    // 接受路径规划的坐标
    ISAAC_PROTO_RX(StateProto, cmd);

    // 发布假装速度
    ISAAC_PROTO_TX(DifferentialBaseStateProto, segway_state);

    // 发布IMU数据
    
};
ISAAC_ALICE_REGISTER_CODELET(Base_Robot);
