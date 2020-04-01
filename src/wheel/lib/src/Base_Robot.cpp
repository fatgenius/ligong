#include "../inc/Base_Robot.hpp"
#include <cstdio>
#include "engine/gems/state/io.hpp"
#include "messages/math.hpp"
#include "messages/state/differential_base.hpp"

void Base_Robot::start() {
    tickPeriodically();
    serial.SerialOpen((char *)get_serial_port().c_str(),57600);


}
void Base_Robot::tick() {

    // 读串口数据
    serial.SerialRecv(serial_data_buf,DataSize);
    show("LinearSpeed",serial_data.Communicate[0]);  
    show("AngularSpeed",serial_data.Communicate[1]);  

    // 发布里程计  
    auto message = tx_segway_state().initProto();
    message.setLinearAcceleration(0);
    message.setLinearSpeed(serial_data.Communicate[0]);
    message.setAngularSpeed(-serial_data.Communicate[1]);
    message.setAngularAcceleration(0);
    tx_segway_state().publish();

    serial_data.Communicate[2] = 0;
    serial_data.Communicate[3] = 0;
    // 读取消息
    if (get_flag())
    {
       if (rx_js_state().available())
        {
            auto proto = rx_js_state().getProto();
            ::capnp::List<Vector2dProto>::Reader axes = proto.getAxes();
            serial_data.Communicate[0] = -0.6*axes[0].getY();
            serial_data.Communicate[1] = -0.32*2*axes[1].getX();

        } 
    }
    else
    {
        if(rx_cmd().available())
        {
            isaac::messages::DifferentialBaseControl command;
            ASSERT(FromProto(rx_cmd().getProto(), command), "Failed to parse rx_segway_cmd");
            double linear_speed = command.linear_speed();
            double angular_speed = command.angular_speed();
            if (linear_speed > 0.0)
                angular_speed = - angular_speed;

            std::printf("%f,%f\n",linear_speed,angular_speed);
            serial_data.Communicate[0] = linear_speed;
            serial_data.Communicate[1] = angular_speed;
        }
    }

    
    serial.SendData(serial_data_buf,DataSize);
}
void Base_Robot::stop() {

}


